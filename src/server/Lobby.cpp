#include "server/Lobby.h"

#include <algorithm>
#include <chrono>

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
    CreateBotRoom(conn, name, "ai-v-ai");  // Jev vs Jev: the first spectator opens the room.
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
    CreateBotRoom(conn, name, mode);
    return;
  }
  Room& room = rooms_[name];
  if (!room.id) room.id = nextRoomId_++;
  if (!room.botMode.empty() || (room.conns[0] >= 0 && room.conns[1] >= 0)) {
    SendError(conn, "room is full");
    return;
  }
  if (room.conns[0] < 0) {
    room.conns[0] = conn;
    members_[conn] = Member{name, Team::Blue};
    Json waiting = Typed("waiting");
    waiting.Set("room", name);
    Send(conn, waiting);
    return;
  }
  room.conns[1] = conn;
  members_[conn] = Member{name, Team::Red};
  room.session = room.scene ? std::make_unique<GameSession>(*room.scene)
                            : std::make_unique<GameSession>(seed_());
  if (room.scene) room.session->ConfigureForTest(room.friendlyFire, room.shotRolls);
  StartMatch(room);
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
    Json start = Typed("start");
    start.Set("team", net::TeamName(team));
    start.Set("seed", static_cast<double>(room.session->seed()));
    if (room.scene) start.Set("scene", room.sceneSpec);
    start.Set("protocol", net::kProtocolVersion);
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

  if (action.kind == net::ActionKind::NewMatch && result.ok) {
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
  for (int ti = 0; ti < 2; ++ti) {
    const Team t = static_cast<Team>(ti);
    SendSeat(room, t, reports[ti]);
    SendSeat(room, t, session.PlansMessage(t));
    SendSeat(room, t, Json(Json::Object{{"t", Json("peer")}, {"ready", Json(false)}}));
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

void Lobby::CreateBotRoom(int conn, const std::string& name, const std::string& mode) {
  int botRooms = 0;
  for (const auto& [roomName, existing] : rooms_) botRooms += existing.botMode.empty() ? 0 : 1;
  if (botRooms >= maxBotRooms_) {
    SendError(conn, "too many Jev matches are running; try again shortly");
    return;
  }
  Room& room = rooms_[name];
  room.name = name;
  room.id = nextRoomId_++;
  room.botMode = mode;
  room.session = std::make_unique<GameSession>(seed_());
  if (mode == "player-v-ai") {
    // The human is Blue; Jev plays Red and starts thinking as the match opens.
    room.conns[0] = conn;
    members_[conn] = Member{name, Team::Blue};
    room.bot[1].active = true;
    room.botsRunning = true;
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
    if (message["shot_rolls"].IsArray()) {
      for (const net::Json& r : message["shot_rolls"].AsArray()) {
        if (r.IsNumber()) room.shotRolls.push_back(static_cast<float>(r.AsNumber()));
      }
    }
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

}  // namespace tactics::server
