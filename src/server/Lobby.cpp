#include "server/Lobby.h"

#include <algorithm>
#include <chrono>
#include <cstdio>

#include "net/SceneSpec.h"

namespace tactics::server {

using net::Json;

namespace {

bool ValidRoomName(const std::string& s) {
  if (s.empty() || s.size() > 32) return false;
  for (char c : s) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '-' || c == '_';
    if (!ok) return false;
  }
  return true;
}

// Room codes are short enough to read out loud; no 0/O/1/I/L lookalikes.
constexpr char kCodeAlphabet[] = "ABCDEFGHJKMNPQRSTUVWXYZ23456789";
constexpr int kCodeLength = 5;

Json Typed(const char* type) {
  Json j;
  j.Set("t", type);
  return j;
}

// Optional client-chosen match setup on a bot-room join (splash map + seed).
std::string MapKindOf(const Json& message) {
  const std::string kind = message["map"].IsString() ? message["map"].AsString() : "";
  return kind == "urban" || kind == "urban-x4" || kind == "hills" ? kind : "";
}
bool CtfOf(const Json& message) { return message["ctf"].IsBool() && message["ctf"].AsBool(); }
std::optional<uint32_t> SeedOf(const Json& message) {
  if (!message["seed"].IsNumber()) return std::nullopt;
  const double s = message["seed"].AsNumber();
  if (s < 0 || s > 4294967295.0 || s != static_cast<double>(static_cast<uint32_t>(s))) {
    return std::nullopt;
  }
  return static_cast<uint32_t>(s);
}

// Anonymous per-device token (client-generated): 16-64 of [A-Za-z0-9_-].
std::string TokenOf(const Json& message) {
  if (!message["token"].IsString()) return "";
  const std::string& t = message["token"].AsString();
  if (t.size() < 16 || t.size() > 64) return "";
  for (char c : t) {
    const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                    c == '-' || c == '_';
    if (!ok) return "";
  }
  return t;
}

}  // namespace

void Lobby::SendError(int conn, const std::string& why) {
  Json j = Typed("error");
  j.Set("error", why);
  Send(conn, j);
}

void Lobby::OnMessage(int conn, const std::string& text) {
  Json message;
  if (!Json::Parse(text, &message) || !message.IsObject()) {
    SendError(conn, "malformed message");
    return;
  }
  if (message["t"].IsString() &&
      HandleSceneMessage(conn, message["t"].AsString(), message)) {
    return;
  }
  if (message["t"].IsString() && message["t"].AsString() == "my_games") {
    HandleMyGames(conn, message);
    return;
  }
  auto it = members_.find(conn);
  if (it == members_.end()) {
    if (message["t"].IsString() && message["t"].AsString() == "join") {
      Join(conn, message);
    } else {
      SendError(conn, "send join first");
    }
    return;
  }
  HandleAction(conn, it->second, message);
}

bool Lobby::HandleSceneMessage(int conn, const std::string& type, const Json& message) {
  if (type != "scenes_list" && type != "scene_put" && type != "scene_delete") return false;
  if (!sceneStore_) {
    SendError(conn, "scene storage is not available");
    return true;
  }
  std::string error;
  if (type == "scenes_list") {
    Json list = Typed("scenes");
    Json::Array scenes;
    for (const SceneStore::Entry& entry : sceneStore_->List()) {
      Json item;
      item.Set("id", entry.id);
      item.Set("scene", entry.scene);
      scenes.push_back(std::move(item));
    }
    list.Set("scenes", Json(std::move(scenes)));
    Send(conn, list);
    return true;
  }
  const Json& id = message["id"];
  if (!id.IsString()) {
    SendError(conn, "scene id required");
    return true;
  }
  if (type == "scene_put") {
    if (!message["scene"].IsString()) {
      SendError(conn, "scene required");
    } else if (!sceneStore_->Put(id.AsString(), message["scene"].AsString(), &error)) {
      SendError(conn, error);
    } else {
      Json saved = Typed("scene_saved");
      saved.Set("id", id.AsString());
      Send(conn, saved);
    }
    return true;
  }
  if (!sceneStore_->Delete(id.AsString(), &error) && !error.empty()) {
    SendError(conn, error);
  } else {
    Json deleted = Typed("scene_deleted");
    deleted.Set("id", id.AsString());
    Send(conn, deleted);
  }
  return true;
}

std::string Lobby::NewRoomCode() {
  if (!codeRng_) codeRng_.emplace(seed_());
  std::uniform_int_distribution<int> pick(0, sizeof(kCodeAlphabet) - 2);
  for (;;) {
    std::string code;
    for (int i = 0; i < kCodeLength; ++i) code += kCodeAlphabet[pick(*codeRng_)];
    if (!rooms_.count(code)) return code;
  }
}

void Lobby::SendSpectatorView(int conn, const Room& room) {
  Json start = Typed("start");
  start.Set("team", "spectator");
  start.Set("seed", static_cast<double>(room.session->seed()));
  if (!room.session->mapKind().empty()) start.Set("map", room.session->mapKind());
  if (room.session->ctf()) start.Set("ctf", Json(true));
  if (room.scene) start.Set("scene", room.sceneSpec);
  start.Set("protocol", net::kProtocolVersion);
  Send(conn, start);
  Send(conn, room.session->SpectatorPlansMessage());
  Send(conn, room.session->SpectatorStateMessage());
}

// A spectator attaches to a room that already has a player in it; if the
// match has not started it just waits for the start.
void Lobby::SpectatorJoin(int conn, const Json& message) {
  if (!message["room"].IsString()) {
    SendError(conn, "spectating needs a room");
    return;
  }
  const std::string name = message["room"].AsString();
  auto it = ValidRoomName(name) ? rooms_.find(name) : rooms_.end();
  if (it == rooms_.end() && ValidRoomName(name) && message["mode"].IsString() &&
      message["mode"].AsString() == "ai-v-ai") {
    CreateBotRoom(conn, name, "ai-v-ai", MapKindOf(message), SeedOf(message), CtfOf(message));  // Jev vs Jev: the first spectator opens the room.
    return;
  }
  if (it == rooms_.end()) {
    SendError(conn, "no such room");
    return;
  }
  Room& room = it->second;
  room.spectators.push_back(conn);
  members_[conn] = Member{name, Team::Blue, true};
  if (room.session) {
    SendSpectatorView(conn, room);
  } else {
    Json waiting = Typed("waiting");
    waiting.Set("room", name);
    Send(conn, waiting);
  }
}

void Lobby::Join(int conn, const Json& message) {
  const std::string token = TokenOf(message);
  if (message["game"].IsString()) {
    Resume(conn, message["game"].AsString(), token);
    return;
  }
  const std::string mode = message["mode"].IsString() ? message["mode"].AsString() : "";
  if (!mode.empty()) {
    if (mode != "player-v-ai" && mode != "ai-v-ai") {
      SendError(conn, "unknown mode");
      return;
    }
    if (!bots_ || !bots_->enabled()) {
      SendError(conn, "Jev is not configured on this server");
      return;
    }
    if (mode == "ai-v-ai" && !message["spectate"].AsBool()) {
      SendError(conn, "Jev vs Jev is watched: join as a spectator");
      return;
    }
  }
  if (message["spectate"].AsBool()) {
    SpectatorJoin(conn, message);
    return;
  }
  // "intent" (optional): "create" opens a room nobody else is in -- with no
  // "room" the server picks an unused code; "join" only enters a room that
  // already exists. Without it, join-or-create by name (split screen, tests).
  const std::string intent = message["intent"].IsString() ? message["intent"].AsString() : "";
  if (!intent.empty() && intent != "create" && intent != "join") {
    SendError(conn, "unknown join intent");
    return;
  }
  std::string name = "default";
  if (intent == "create" && !message.Has("room")) {
    name = NewRoomCode();
  } else if (message.Has("room")) {
    if (!message["room"].IsString() || !ValidRoomName(message["room"].AsString())) {
      SendError(conn, "invalid room name (1-32 chars of A-Z a-z 0-9 - _)");
      return;
    }
    name = message["room"].AsString();
  }
  const bool exists = rooms_.count(name) > 0;
  // A token that owns a seat in a stored match resumes it, whatever the intent.
  if (exists && !token.empty() && !rooms_[name].gameId.empty()) {
    const Room& existing = rooms_[name];
    if (const auto seat = gameStore_ ? gameStore_->TeamOf(existing.gameId, token) : std::nullopt) {
      
      Resume(conn, existing.gameId, token);
      return;
    }
  }
  if (intent == "create" && exists) {
    SendError(conn, "room code already in use");
    return;
  }
  if (intent == "join" && !exists) {
    SendError(conn, "no such room");
    return;
  }
  if (mode == "player-v-ai") {
    if (exists) {
      SendError(conn, "room code already in use");
      return;
    }
    CreateBotRoom(conn, name, mode, MapKindOf(message), SeedOf(message), CtfOf(message), token);
    return;
  }
  Room& room = rooms_[name];
  if (!room.id) room.id = nextRoomId_++;
  if (!room.botMode.empty() || room.session || (room.conns[0] >= 0 && room.conns[1] >= 0)) {
    SendError(conn, "room is full");
    return;
  }
  if (room.conns[0] < 0) {
    room.conns[0] = conn;
    room.mapKind = MapKindOf(message);
    room.mapSeed = SeedOf(message);
    room.ctf = CtfOf(message);
    room.tokens[0] = token;
    members_[conn] = Member{name, Team::Blue};
    Json waiting = Typed("waiting");
    waiting.Set("room", name);
    Send(conn, waiting);
    return;
  }
  room.conns[1] = conn;
  room.tokens[1] = token;
  members_[conn] = Member{name, Team::Red};
  room.session = room.scene ? std::make_unique<GameSession>(*room.scene)
                            : std::make_unique<GameSession>(room.mapSeed ? *room.mapSeed : seed_(),
                                                        room.mapKind, room.ctf, roundLimit_);
  if (room.scene) room.session->ConfigureForTest(room.friendlyFire, room.figureOverlapCheck, room.shotRolls);
  room.name = name;
  PersistNewGame(room);
  StartMatch(room);
}

Json Lobby::StartMessage(const Room& room, Team team) const {
  Json start = Typed("start");
  start.Set("team", net::TeamName(team));
  start.Set("seed", static_cast<double>(room.session->seed()));
  if (!room.session->mapKind().empty()) start.Set("map", room.session->mapKind());
  if (room.session->ctf()) start.Set("ctf", Json(true));
  if (room.scene) start.Set("scene", room.sceneSpec);
  if (!room.gameId.empty()) start.Set("game", room.gameId);
  start.Set("protocol", net::kProtocolVersion);
  return start;
}

void Lobby::StartMatch(Room& room) {
  // A new match: answers still in flight belong to the old one.
  room.id = nextRoomId_++;
  room.nextBotMs = 0;
  for (BotSeat& seat : room.bot) {
    seat.inFlight = false;
    seat.error.clear();
  }
  for (int ti = 0; ti < 2; ++ti) {
    const Team team = static_cast<Team>(ti);
    const Json start = StartMessage(room, team);
    SendSeat(room, team, start);
    SendSeat(room, team, room.session->PlansMessage(team));
    SendSeat(room, team, room.session->StateMessage(team));
  }
  for (int spectator : room.spectators) SendSpectatorView(spectator, room);
  BroadcastBots(room);
}

void Lobby::HandleAction(int conn, Member& member, const Json& message) {
  if (message["t"].IsString() && message["t"].AsString() == "bot_control") {
    auto it = rooms_.find(member.room);
    if (it == rooms_.end()) return;
    HandleBotControl(conn, it->second, message);
    return;
  }
  if (member.spectator) {
    SendError(conn, "spectators are read-only");
    return;
  }
  auto roomIt = rooms_.find(member.room);
  if (roomIt == rooms_.end() || !roomIt->second.session) {
    SendError(conn, "waiting for an opponent");
    return;
  }
  Room& room = roomIt->second;
  GameSession& session = *room.session;
  const Team team = member.team;
  const Team other = OpposingTeam(team);

  net::Action action;
  std::string error;
  if (!net::ParseAction(message, &action, &error)) {
    SendError(conn, error);
    return;
  }
  const bool wasReady = session.Ready(team);
  const GameSession::Result result = session.Apply(
      team, action, action.kind == net::ActionKind::NewMatch ? seed_() : 0);

  Json ack = Typed("ack");
  ack.Set("seq", action.seq);
  ack.Set("ok", Json(result.ok));
  if (!result.ok) ack.Set("error", result.error);
  Send(conn, ack);
  if (action.kind == net::ActionKind::Commit && result.ok) PersistCommit(room, team);

  if (action.kind == net::ActionKind::NewMatch && result.ok) {
    PersistNewGame(room);
    StartMatch(room);
    return;
  }
  // Always answer with the authoritative plans so a rejected action (or a
  // stale client) resyncs; the opponent only hears about readiness.
  Send(conn, session.PlansMessage(team));
  for (int spectator : room.spectators) Send(spectator, session.SpectatorPlansMessage());
  if (session.Ready(team) != wasReady) {
    Json peer = Typed("peer");
    peer.Set("ready", Json(session.Ready(team)));
    SendSeat(room, other, peer);
  }
  MaybeRunRound(room);
}

void Lobby::MaybeRunRound(Room& room) {
  GameSession& session = *room.session;
  if (!session.BothReady()) return;
  const auto reports = session.RunRound();
  PersistTurn(room, reports);
  for (int ti = 0; ti < 2; ++ti) {
    const Team t = static_cast<Team>(ti);
    SendSeat(room, t, reports[ti]);
    SendSeat(room, t, session.PlansMessage(t));
    SendSeat(room, t, Json(Json::Object{{"t", Json("peer")}, {"ready", Json(false)}}));
  }
  // Match over: players saw a fogged timeline, so hand them the full one for
  // the post-game replay (spectators already received it unfiltered).
  if (session.game().Mode() == InputMode::GameOver) {
    const Json reveal = session.RevealMessage();
    for (int ti = 0; ti < 2; ++ti) SendSeat(room, static_cast<Team>(ti), reveal);
  }
  for (int spectator : room.spectators) {
    Send(spectator, reports[2]);
    Send(spectator, session.SpectatorPlansMessage());
  }
  // With nobody playing, Jev would plan rounds faster than they play back.
  if (pacePlayback_ && room.conns[0] < 0 && room.conns[1] < 0) {
    const double seconds = reports[2]["duration"].AsNumber(constants::kRoundDuration);
    room.nextBotMs = NowMs() + static_cast<int64_t>((seconds + 1.0) * 1000.0);
  }
  BroadcastBots(room);
}

int64_t Lobby::NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

void Lobby::CreateBotRoom(int conn, const std::string& name, const std::string& mode,
                          const std::string& mapKind, std::optional<uint32_t> seed, bool ctf,
                          const std::string& token) {
  int botRooms = 0;
  for (const auto& [roomName, existing] : rooms_) {
    // A stored human-v-Jev match whose player is away costs nothing (Jev idles).
    const bool idle = existing.botMode == "player-v-ai" && existing.conns[0] < 0;
    botRooms += existing.botMode.empty() || idle ? 0 : 1;
  }
  if (botRooms >= maxBotRooms_) {
    SendError(conn, "too many Jev matches are running; try again shortly");
    return;
  }
  Room& room = rooms_[name];
  room.name = name;
  room.id = nextRoomId_++;
  room.botMode = mode;
  room.session = std::make_unique<GameSession>(seed ? *seed : seed_(), mapKind, ctf);
  if (mode == "player-v-ai") {
    // The human is Blue; Jev plays Red and starts thinking as the match opens.
    room.conns[0] = conn;
    members_[conn] = Member{name, Team::Blue};
    room.bot[1].active = true;
    room.botsRunning = true;
    // With a device token the match is stored like a human one, Jev's seat
    // held by the reserved token (see GameStore::kJevToken).
    room.tokens[0] = token;
    room.tokens[1] = GameStore::kJevToken;
    PersistNewGame(room);
    StartMatch(room);
    return;
  }
  // Jev vs Jev starts paused, like the local preview; a spectator starts it.
  room.bot[0].active = room.bot[1].active = true;
  room.botsRunning = false;
  room.spectators.push_back(conn);
  members_[conn] = Member{name, Team::Blue, true};
  SendSpectatorView(conn, room);
  BroadcastBots(room);
}

void Lobby::BroadcastBots(const Room& room) {
  if (room.botMode.empty()) return;
  Json m = Typed("bots");
  m.Set("mode", room.botMode);
  m.Set("running", Json(room.botsRunning));
  Json thinking{Json::Array{}};
  std::string error;
  bool fallback = false;
  for (int ti = 0; ti < 2; ++ti) {
    const BotSeat& seat = room.bot[ti];
    if (!seat.active) continue;
    const Team team = static_cast<Team>(ti);
    if (seat.inFlight) thinking.Push(Json(net::TeamName(team)));
    if (seat.fallback) fallback = true;
    if (error.empty() && !seat.error.empty()) {
      error = std::string(team == Team::Blue ? "Blue" : "Red") + ": " + seat.error;
    }
  }
  m.Set("thinking", std::move(thinking));
  m.Set("error", error);
  m.Set("fallback", Json(fallback));
  for (int ti = 0; ti < 2; ++ti) SendSeat(room, static_cast<Team>(ti), m);
  for (int spectator : room.spectators) Send(spectator, m);
}

void Lobby::HandleBotControl(int conn, Room& room, const Json& message) {
  if (room.botMode.empty()) {
    SendError(conn, "this room has no Jev player");
    return;
  }
  if (message["run"].IsBool()) {
    if (room.botMode != "ai-v-ai") {
      SendError(conn, "only Jev vs Jev can be started or paused");
      return;
    }
    room.botsRunning = message["run"].AsBool();
  }
  if (message["fallback"].IsBool()) {
    for (BotSeat& seat : room.bot) {
      if (!seat.active) continue;
      seat.fallback = message["fallback"].AsBool();
      seat.error.clear();
    }
  }
  if (message["retry"].AsBool()) {
    for (BotSeat& seat : room.bot) seat.error.clear();
  }
  BroadcastBots(room);
}

void Lobby::AnnounceBotCommit(Room& room, Team bot) {
  for (int spectator : room.spectators) Send(spectator, room.session->SpectatorPlansMessage());
  Json peer = Typed("peer");
  peer.Set("ready", Json(true));
  SendSeat(room, OpposingTeam(bot), peer);
}

void Lobby::ScheduleBots(Room& room) {
  if (!bots_ || room.botMode.empty() || !room.session || !room.botsRunning) return;
  GameSession& session = *room.session;
  if (!session.PlanningOpen()) return;
  // A held human-v-Jev match is paused while its player is away.
  if (room.botMode == "player-v-ai" && room.conns[0] < 0) return;
  if (room.nextBotMs && NowMs() < room.nextBotMs) return;
  bool changed = false;
  for (int ti = 0; ti < 2; ++ti) {
    BotSeat& seat = room.bot[ti];
    const Team team = static_cast<Team>(ti);
    if (!seat.active || seat.inFlight || !seat.error.empty() || session.Ready(team)) continue;
    JevJob job;
    job.roomId = room.id;
    job.roomName = room.name;
    job.team = team;
    job.game = session.game();
    job.nonce = ++botNonce_;
    job.fallback = seat.fallback;
    bots_->Submit(std::move(job));
    seat.inFlight = true;
    changed = true;
  }
  if (changed) BroadcastBots(room);
}

void Lobby::Tick() {
  if (!bots_) return;
  for (JevResult& result : bots_->TakeResults()) {
    auto it = rooms_.find(result.roomName);
    if (it == rooms_.end() || it->second.id != result.roomId || !it->second.session) continue;
    Room& room = it->second;
    BotSeat& seat = room.bot[static_cast<int>(result.team)];
    GameSession& session = *room.session;
    seat.inFlight = false;
    // Paused, switched to the fallback planner, or no longer planning: drop
    // the answer; the seat asks again when it is next due.
    if (!seat.active || !room.botsRunning || seat.fallback != result.fallback ||
        !session.PlanningOpen() || session.Ready(result.team)) {
      BroadcastBots(room);
      continue;
    }
    if (!result.error.empty()) {
      seat.error = result.error;
      BroadcastBots(room);
      continue;
    }
    if (!result.nothingToAsk && result.request.round != session.game().RoundNumber()) {
      BroadcastBots(room);  // Stale (the round moved on); re-asked on the next pass.
      continue;
    }
    const GameSession::Result applied = session.ApplyBotPlan(
        result.team, result.nothingToAsk ? nullptr : &result.request, result.choices);
    if (!applied.ok) {
      seat.error = applied.error;
      BroadcastBots(room);
      continue;
    }
    PersistCommit(room, result.team);
    AnnounceBotCommit(room, result.team);
    MaybeRunRound(room);
    BroadcastBots(room);
  }
  for (auto& [name, room] : rooms_) ScheduleBots(room);
}

Json Lobby::OnControl(const Json& message) {
  auto reply = [](bool ok, const std::string& error) {
    Json j = Typed("control");
    j.Set("ok", Json(ok));
    if (!ok) j.Set("error", error);
    return j;
  };
  const std::string type = message["t"].IsString() ? message["t"].AsString() : "";
  const std::string name = message["room"].IsString() ? message["room"].AsString() : "";
  if (!ValidRoomName(name)) return reply(false, "invalid room name");
  if (type == "load_scenario") {
    if (rooms_.count(name)) return reply(false, "room already exists");
    Scene scene;
    std::string error;
    if (!net::SceneFromSpec(message["scene"], &scene, &error)) return reply(false, error);
    Room& room = rooms_[name];
    room.id = nextRoomId_++;
    room.scene = std::move(scene);
    room.sceneSpec = message["scene"];
    if (message["friendly_fire"].IsBool()) room.friendlyFire = message["friendly_fire"].AsBool();
    if (message["figure_overlap"].IsBool()) room.figureOverlapCheck = message["figure_overlap"].AsBool();
    if (message["shot_rolls"].IsArray()) {
      for (const net::Json& r : message["shot_rolls"].AsArray()) {
        if (r.IsNumber()) room.shotRolls.push_back(static_cast<float>(r.AsNumber()));
      }
    }
    return reply(true, "");
  }
  if (type == "remove_obstacle") {  // Debug: remove an obstacle mid-match.
    auto it = rooms_.find(name);
    if (it == rooms_.end() || !it->second.session) return reply(false, "no running match in room");
    Room& room = it->second;
    if (!message["obstacle"].IsNumber()) return reply(false, "obstacle index required");
    if (!room.session->DebugRemoveObstacle(static_cast<int>(message["obstacle"].AsNumber()))) {
      return reply(false, "no such live obstacle");
    }
    Json world = Typed("world");
    world.Set("world", room.session->TakeWorldDelta());
    for (int ti = 0; ti < 2; ++ti) SendSeat(room, static_cast<Team>(ti), world);
    for (int spectator : room.spectators) Send(spectator, world);
    return reply(true, "");
  }
  if (type == "dump_state") {
    auto it = rooms_.find(name);
    if (it == rooms_.end() || !it->second.session) return reply(false, "no running match in room");
    return it->second.session->DumpState();
  }
  return reply(false, "unknown control message");
}

void Lobby::OnDisconnect(int conn) {
  auto it = members_.find(conn);
  if (it == members_.end()) return;
  const Member member = it->second;
  members_.erase(it);
  auto roomIt = rooms_.find(member.room);
  if (roomIt == rooms_.end()) return;
  Room& room = roomIt->second;
  if (member.spectator) {
    room.spectators.erase(std::remove(room.spectators.begin(), room.spectators.end(), conn),
                          room.spectators.end());
    // Nobody left to watch a Jev vs Jev match: stop spending on it.
    if (room.botMode == "ai-v-ai" && room.spectators.empty()) rooms_.erase(roomIt);
    return;
  }
  // A stored, unfinished match survives a dropped player: the seat is held
  // for their token (Resume), the opponent keeps the room.
  if (!room.gameId.empty() && room.session &&
      room.session->game().Mode() != InputMode::GameOver) {
    room.conns[static_cast<int>(member.team)] = -1;
    SendSeat(room, OpposingTeam(member.team), Typed("opponent_away"));
    return;
  }
  for (int spectator : room.spectators) {
    Send(spectator, Typed("opponent_left"));
    members_.erase(spectator);
  }
  const int otherConn = room.conns[static_cast<int>(OpposingTeam(member.team))];
  if (otherConn >= 0) {
    Send(otherConn, Typed("opponent_left"));
    members_.erase(otherConn);  // The match is over; the survivor may join again.
  }
  rooms_.erase(roomIt);
}


// ---- Persistence ----

std::string Lobby::NewGameId() {
  if (!codeRng_) codeRng_.emplace(seed_());
  static const char* kHex = "0123456789abcdef";
  for (;;) {
    std::string id = "g";
    for (int i = 0; i < 12; ++i) id += kHex[(*codeRng_)() % 16];
    if (!gameStore_->Find(id)) return id;
  }
}

void Lobby::PersistNewGame(Room& room) {
  room.gameId.clear();
  room.turn = 1;
  room.lastRound[0].clear();
  room.lastRound[1].clear();
  const std::string& blue = room.tokens[0];
  const std::string& red = room.tokens[1];
  if (!gameStore_ || room.scene || room.botMode == "ai-v-ai" || blue.empty() || red.empty() ||
      blue == red) {
    return;
  }
  GameStore::Game game;
  game.id = NewGameId();
  game.room = room.name;
  game.seed = room.session->seed();
  game.mapKind = room.session->mapKind();
  game.ctf = room.session->ctf();
  game.roundLimit = room.session->roundLimit();
  if (!gameStore_->CreateGame(game, blue, red)) {
    std::fprintf(stderr, "tbgwaf_server: cannot store game: %s\n", gameStore_->error().c_str());
    return;
  }
  room.gameId = game.id;
}

void Lobby::PersistCommit(Room& room, Team team) {
  if (room.gameId.empty()) return;
  if (!gameStore_->PutOrders(room.gameId, room.turn, static_cast<int>(team),
                             room.session->OrdersFor(team).Dump(/*exact=*/true))) {
    std::fprintf(stderr, "tbgwaf_server: cannot store orders: %s\n", gameStore_->error().c_str());
  }
}

void Lobby::PersistTurn(Room& room, const std::array<Json, 3>& reports) {
  if (room.gameId.empty()) return;
  GameStore::Turn turn;
  turn.round = room.turn++;
  for (int i = 0; i < 3; ++i) turn.messages[i] = reports[i].Dump();
  turn.stateHash = room.session->StateChecksum();
  room.lastRound[0] = turn.messages[0];
  room.lastRound[1] = turn.messages[1];
  const GameSession& session = *room.session;
  const bool over = session.game().Mode() == InputMode::GameOver;
  const std::string winner =
      !over ? "" : (session.game().Winner() ? net::TeamName(*session.game().Winner()) : "none");
  if (!gameStore_->PutTurn(room.gameId, turn, over, winner)) {
    std::fprintf(stderr, "tbgwaf_server: cannot store turn: %s\n", gameStore_->error().c_str());
  }
}

std::unique_ptr<GameSession> Lobby::ReplayGame(const GameStore::Game& game, bool pending,
                                               int* nextTurn, std::string* error) {
  auto session = std::make_unique<GameSession>(game.seed, game.mapKind, game.ctf, game.roundLimit);
  const std::vector<GameStore::Order> orders = gameStore_->Orders(game.id);
  const std::vector<GameStore::Turn> turns = gameStore_->Turns(game.id);
  auto ordersOf = [&](int round, int team) -> const GameStore::Order* {
    for (const GameStore::Order& o : orders) {
      if (o.round == round && o.team == team) return &o;
    }
    return nullptr;
  };
  auto apply = [&](int round, int team) {
    const GameStore::Order* order = ordersOf(round, team);
    Json json;
    if (!order || !Json::Parse(order->json, &json)) {
      *error = "missing orders for round " + std::to_string(round);
      return false;
    }
    const GameSession::Result result = session->ApplyOrders(static_cast<Team>(team), json);
    if (!result.ok) *error = "round " + std::to_string(round) + ": " + result.error;
    return result.ok;
  };
  int round = 1;
  for (const GameStore::Turn& turn : turns) {
    if (turn.round != round || !apply(round, 0) || !apply(round, 1)) {
      if (error->empty()) *error = "turn gap at round " + std::to_string(round);
      return nullptr;
    }
    session->RunRound();
    if (session->StateChecksum() != turn.stateHash) {
      *error = "replay diverged at round " + std::to_string(round);
      return nullptr;
    }
    ++round;
  }
  if (pending) {
    for (int team = 0; team < 2; ++team) {
      if (ordersOf(round, team) && !apply(round, team)) return nullptr;
    }
  }
  *nextTurn = round;
  return session;
}

int Lobby::SetGameStore(GameStore* store) {
  gameStore_ = store;
  if (!store || !store->ok()) return 0;
  int restored = 0;
  for (const GameStore::Game& game : store->ActiveGames()) {
    if (rooms_.count(game.room)) continue;
    std::string error;
    int nextTurn = 1;
    auto session = ReplayGame(game, true, &nextTurn, &error);
    if (!session) {
      std::fprintf(stderr, "tbgwaf_server: game %s not restored: %s\n", game.id.c_str(),
                   error.c_str());
      continue;
    }
    Room& room = rooms_[game.room];
    room.name = game.room;
    room.id = nextRoomId_++;
    room.gameId = game.id;
    room.turn = nextTurn;
    room.session = std::move(session);
    if (store->TeamOf(game.id, GameStore::kJevToken)) {  // Human vs Jev: Jev is Red.
      room.botMode = "player-v-ai";
      room.bot[1].active = true;
      room.tokens[1] = GameStore::kJevToken;
    }
    const std::vector<GameStore::Turn> turns = store->Turns(game.id);
    if (!turns.empty()) {
      room.lastRound[0] = turns.back().messages[0];
      room.lastRound[1] = turns.back().messages[1];
    }
    ++restored;
  }
  return restored;
}

void Lobby::Resume(int conn, const std::string& gameId, const std::string& token) {
  const auto team = gameStore_ && !token.empty() ? gameStore_->TeamOf(gameId, token) : std::nullopt;
  const auto game = team ? gameStore_->Find(gameId) : std::nullopt;
  if (!game) {
    SendError(conn, "no such game");
    return;
  }
  const Team seat = static_cast<Team>(*team);
  if (game->finished) {
    ViewFinished(conn, *game, seat);
    return;
  }
  Room* room = nullptr;
  for (auto& [name, candidate] : rooms_) {
    if (candidate.gameId == gameId) room = &candidate;
  }
  if (!room || !room->session) {
    SendError(conn, "game is not available");
    return;
  }
  if (members_.count(conn)) {
    SendError(conn, "already in a game");
    return;
  }
  int& seatConn = room->conns[*team];
  if (seatConn >= 0) {  // The same device reconnecting before the old socket was noticed gone.
    Send(seatConn, Typed("opponent_left"));
    members_.erase(seatConn);
  }
  seatConn = conn;
  members_[conn] = Member{room->name, seat};
  Json start = StartMessage(*room, seat);
  start.Set("resumed", Json(true));
  Send(conn, start);
  Send(conn, room->session->StateMessage(seat));
  // The latest resolved turn plays back first (as it did live), then the
  // authoritative plans: they carry this player's commit flag, which a round
  // clears on the client.
  if (!room->lastRound[*team].empty()) send_(conn, room->lastRound[*team]);
  Send(conn, room->session->PlansMessage(seat));
  Json peer = Typed("peer");
  peer.Set("ready", Json(room->session->Ready(OpposingTeam(seat))));
  Send(conn, peer);
  SendSeat(*room, OpposingTeam(seat), Typed("opponent_back"));
  BroadcastBots(*room);
}

void Lobby::ViewFinished(int conn, const GameStore::Game& game, Team team) {
  // Everything comes from the stored turns (no re-simulation): the final
  // state is the last round message's own state body, the world log is the
  // rounds' changes in order, and the reveal is the unfiltered timeline.
  const std::vector<GameStore::Turn> turns = gameStore_->Turns(game.id);
  Json last;
  if (turns.empty() || !Json::Parse(turns.back().messages[static_cast<int>(team)], &last)) {
    SendError(conn, "game cannot be replayed: no stored turns");
    return;
  }
  Json start = Typed("start");
  start.Set("team", net::TeamName(team));
  start.Set("seed", static_cast<double>(game.seed));
  if (!game.mapKind.empty()) start.Set("map", game.mapKind);
  if (game.ctf) start.Set("ctf", Json(true));
  start.Set("game", game.id);
  start.Set("finished", Json(true));
  start.Set("protocol", net::kProtocolVersion);
  Send(conn, start);

  Json plans = Typed("plans");
  plans.Set("round", last["state"]["round"]);
  plans.Set("ready", Json(false));
  plans.Set("plans", Json(Json::Array{}));
  if (last["state"].Has("playbook")) plans.Set("playbook", last["state"]["playbook"]);
  Send(conn, plans);

  Json world{Json::Array{}};
  Json rounds{Json::Array{}};
  for (const GameStore::Turn& turn : turns) {
    Json mine, spectator;
    if (Json::Parse(turn.messages[static_cast<int>(team)], &mine)) {
      for (const Json& change : mine["world"].AsArray()) world.Push(change);
    }
    if (Json::Parse(turn.messages[2], &spectator)) rounds.Push(std::move(spectator));
  }
  Json state = last["state"];
  state.Set("t", "state");
  state.Set("world", std::move(world));
  Send(conn, state);
  Json reveal = Typed("reveal");
  reveal.Set("rounds", std::move(rounds));
  Send(conn, reveal);
}

void Lobby::HandleMyGames(int conn, const Json& message) {
  const std::string token = TokenOf(message);
  if (token.empty()) {
    SendError(conn, "a device token is required");
    return;
  }
  Json list = Typed("games");
  Json games{Json::Array{}};
  if (gameStore_) {
    std::vector<int> teams;
    const std::vector<GameStore::Game> own = gameStore_->GamesOf(token, &teams);
    for (size_t i = 0; i < own.size(); ++i) {
      const GameStore::Game& g = own[i];
      Json item;
      item.Set("id", g.id);
      item.Set("status", g.finished ? "finished" : "active");
      item.Set("team", net::TeamName(static_cast<Team>(teams[i])));
      item.Set("round", g.round);
      if (gameStore_->TeamOf(g.id, GameStore::kJevToken)) item.Set("vs", "ai");
      if (g.finished) item.Set("winner", g.winner);
      item.Set("created", static_cast<double>(g.createdMs));
      item.Set("updated", static_cast<double>(g.updatedMs));
      games.Push(std::move(item));
    }
  }
  list.Set("games", std::move(games));
  Send(conn, list);
}

}  // namespace tactics::server
