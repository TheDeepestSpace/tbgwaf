// "integration" scenario suite: whole-app flows against a real tbgwaf server
// (the test-control build, SQLite on a temp file, localhost).
//
//   tactics_integration_tests [--server tbgwaf_server_testctl] [--dir tests/integration]
//                             [--list] [--only NAME] [--shard i/N] [--timings]
//
// Each tests/integration/*.yaml scenario scripts one or more headless "apps"
// (alice, bob, ...). An app is what the browser client is without the GL:
// the menu flow (flow/app_flow.yaml via the generated AppFlow.h), the device
// token and last-game memory (localStorage), net::GamesClient for "My games"
// and net::RemoteClient over a native WebSocket for the match. `click` presses
// a menu button by the label DrawMenu draws (ui/MenuButtons.h), so scenarios
// walk Splash -> Game Mode -> Online / My games / Continue the way a player
// does. The server can be restarted or killed mid-scenario; its SQLite file
// survives. See tests/integration/README.md for the step vocabulary.
//
// Sharding mirrors the other suites (index % N over the scenario names), so
// CI derives the shard count from --list (scripts/ci_matrix.py).

#include <signal.h>
#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "NetRig.h"
#include "game/GameLogic.h"
#include "net/GamesClient.h"
#include "net/RemoteClient.h"
#include "ui/MenuButtons.h"

namespace fs = std::filesystem;
using namespace tactics;
using net::Json;
using testing::NowMs;
using testing::ServerProcess;
using testing::WsClient;
namespace flow = tbgwaf_flow;

namespace {

// The server replays stored matches (map generation + simulation) when it
// restarts or a game is opened, so allow far more than the unit-test default.
constexpr int kWaitMs = 120000;

const char* ScreenName(flow::State s) {
  switch (s) {
    case flow::State::Splash: return "splash";
    case flow::State::GameMode: return "game_mode";
    case flow::State::MapSelect: return "map_select";
    case flow::State::Builder: return "builder";
    case flow::State::BuilderEdit: return "builder_edit";
    case flow::State::Room: return "room";
    case flow::State::MyGames: return "my_games";
    case flow::State::Gameplay: return "gameplay";
  }
  return "?";
}

const char* StatusName(net::RemoteClient::Status s) {
  switch (s) {
    case net::RemoteClient::Status::Connecting: return "connecting";
    case net::RemoteClient::Status::Waiting: return "waiting";
    case net::RemoteClient::Status::Playing: return "playing";
    case net::RemoteClient::Status::OpponentLeft: return "opponent_left";
    case net::RemoteClient::Status::Disconnected: return "disconnected";
    case net::RemoteClient::Status::Rejected: return "rejected";
  }
  return "?";
}

// What a scenario compares: the visible state of one app's match.
struct Snapshot {
  std::string screen;
  int round = 0;
  bool selfReady = false, peerReady = false, over = false;
  std::string gameId;
  std::string units;   // id:alive:x:z of every unit the app can see.
  std::string reveal;  // The full-information timeline, once the game is over.
};

struct App {
  std::string name;
  std::string token;
  flow::State screen = flow::kInitialState;
  GameLogic game{Scene{}};  // A match's start replaces this placeholder.
  std::unique_ptr<net::RemoteClient> remote;
  net::GamesClient games;
  WsClient ws;
  bool wsOpen = false;
  bool wsIsGames = false;
  // "localStorage": survives a reload.
  std::string lastGameId;
  // Room screen: the typed code; the code shown while waiting.
  std::string typedCode;
  std::string error;
  std::string reveal;  // Last "reveal" frame received.
  int barrierSeq = 0;
  bool barrierAcked = false, barrierDone = false;
  bool fromResume = false;

  App(std::string n, std::string t) : name(std::move(n)), token(std::move(t)), games(token) {}
  net::RemoteClient* client() { return remote.get(); }
};

class Runner {
 public:
  Runner(std::string serverBinary, std::string scenarioPath)
      : serverBinary_(std::move(serverBinary)), path_(std::move(scenarioPath)) {}
  ~Runner() {
    apps_.clear();
    server_.Stop();
    for (const char* suffix : {"", "-wal", "-shm"}) std::remove((dbPath_ + suffix).c_str());
  }

  bool Run(std::vector<std::string>* failures);

 private:
  bool Fail(const std::string& why) {
    failures_.push_back("step " + std::to_string(step_ + 1) + ": " + why);
    return false;
  }
  App* Get(const YAML::Node& node, const char* key = "app") {
    const std::string name = node[key] ? node[key].as<std::string>() : "";
    auto it = apps_.find(name);
    if (it == apps_.end()) {
      Fail("unknown app '" + name + "' (declare it with `app:`)");
      return nullptr;
    }
    return it->second.get();
  }
  bool StartServer() {
    std::string error;
    std::vector<std::string> args = {"--db", dbPath_};
    if (roundLimit_ > 0) args.insert(args.end(), {"--round-limit", std::to_string(roundLimit_)});
    if (!server_.Start(serverBinary_, &error, args, kWaitMs)) return Fail("server: " + error);
    return true;
  }

  bool Pump(int ms);
  bool Settle();
  template <typename Pred>
  bool PumpUntil(Pred done) {
    const int64_t deadline = NowMs() + kWaitMs;
    while (!done()) {
      if (NowMs() > deadline) return false;
      Pump(5);
    }
    return true;
  }
  void Dispatch(App& a, const std::string& text);
  void SendOutgoing(App& a);
  bool Connect(App& a, bool games);
  void Disconnect(App& a);
  bool Barrier(App& a);
  bool Sync(App& a);
  bool CommitApp(App& a);
  bool PlayOut();
  void PlanAllLocally(App& a);
  bool PlayRound(const std::vector<App*>& who);
  bool Click(App& a, const std::string& button, int row);
  bool StartMatchClient(App& a, std::unique_ptr<net::RemoteClient> client, bool waitForMatch);
  bool Reload(App& a);
  Snapshot Take(App& a);
  bool Assert(const YAML::Node& node);
  bool Step(const YAML::Node& step);

  std::string serverBinary_, path_;
  ServerProcess server_;
  std::string dbPath_;
  std::map<std::string, std::unique_ptr<App>> apps_;
  std::map<std::string, Snapshot> snapshots_;
  std::vector<std::string> failures_;
  int step_ = 0;
  int roundLimit_ = 0;  // Scenario `round_limit`: matches end as a draw after N rounds.
};

void Runner::SendOutgoing(App& a) {
  if (!a.wsOpen) return;
  if (a.wsIsGames) {
    for (const std::string& f : a.games.TakeOutgoing()) a.ws.Send(f);
  } else if (a.remote) {
    for (const std::string& f : a.remote->TakeOutgoing()) a.ws.Send(f);
  }
}

void Runner::Dispatch(App& a, const std::string& text) {
  if (std::getenv("NETDBG")) std::fprintf(stderr, "[%s] %.200s\n", a.name.c_str(), text.c_str());
  if (a.wsIsGames) {
    a.games.OnMessage(text);
    return;
  }
  Json m;
  if (Json::Parse(text, &m)) {
    const std::string& type = m["t"].AsString();
    const int seq = static_cast<int>(m["seq"].AsNumber(0));
    const bool isBarrier = type == "ack" && seq == 1000000 + a.barrierSeq;
    if (type == "ack" && !m["ok"].AsBool(true) && !isBarrier) {
      Fail(a.name + ": server rejected an action: " + m["error"].AsString());
    }
    if (type == "error") a.error = m["error"].AsString();
    if (type == "reveal") a.reveal = text;
    if (isBarrier) a.barrierAcked = true;
    else if (type == "plans" && a.barrierAcked) a.barrierDone = true;
  }
  if (a.remote) a.remote->OnMessage(text);
}

// Receives what is pending on every open socket (waits up to `ms` on the
// first). True if any message arrived.
bool Runner::Pump(int ms) {
  bool any = false;
  for (auto& [name, a] : apps_) {
    if (!a->wsOpen) continue;
    std::vector<std::string> messages;
    if (!a->ws.Poll(ms, &messages)) {
      a->wsOpen = false;
      if (a->wsIsGames) a->games.OnClose();
      else if (a->remote) a->remote->OnClose();
    }
    for (const std::string& text : messages) Dispatch(*a, text);
    // The app remembers the match it is in (localStorage) as soon as it knows it.
    if (a->remote && !a->remote->game_id().empty()) a->lastGameId = a->remote->game_id();
    any = any || !messages.empty();
    ms = 0;
  }
  return any;
}

// Pumps until the wire has been quiet for a moment, then lets any round
// playback run out (as the frame loop would).
bool Runner::Settle() {
  int quiet = 0;
  const int64_t deadline = NowMs() + kWaitMs;
  while (quiet < 4 && NowMs() < deadline) {
    quiet = Pump(10) ? 0 : quiet + 1;
    for (auto& [name, a] : apps_) {
      if (a->remote) SendOutgoing(*a);
    }
  }
  return PlayOut();
}

bool Runner::PlayOut() {
  for (int i = 0; i < 40000; ++i) {
    bool playing = false;
    for (auto& [name, a] : apps_) {
      if (!a->remote || !a->remote->playingBack()) continue;
      playing = true;
      a->remote->Update(0.1f);
      a->game.UpdateSightingMemory(0.1f);
    }
    if (!playing) return true;
  }
  return Fail("round playback never finished");
}

bool Runner::Connect(App& a, bool games) {
  std::string error;
  if (!a.ws.Connect(server_.gamePort, &error)) return Fail(a.name + ": connect: " + error);
  a.wsOpen = true;
  a.wsIsGames = games;
  if (games) a.games.OnOpen();
  else a.remote->OnOpen();
  SendOutgoing(a);
  return true;
}

void Runner::Disconnect(App& a) {
  a.ws.Close();
  a.wsOpen = false;
  if (a.wsIsGames) a.games.OnClose();
}

bool Runner::Barrier(App& a) {
  net::Action ping;
  ping.kind = net::ActionKind::Reaction;
  ping.playbook = a.game.Playbook(*a.remote->team());
  ping.seq = 1000000 + (++a.barrierSeq);
  a.barrierAcked = a.barrierDone = false;
  a.ws.Send(net::EncodeAction(ping).Dump());
  if (!PumpUntil([&] { return a.barrierDone; })) {
    return Fail(a.name + ": barrier timed out (server stopped answering?)");
  }
  return true;
}

bool Runner::Sync(App& a) {
  a.remote->Update(0.0f);
  SendOutgoing(a);
  return Barrier(a);
}

bool Runner::CommitApp(App& a) {
  if (!a.game.CanCommitRound()) return Fail(a.name + ": cannot commit: a living figure has no plan");
  a.remote->RequestCommit();
  SendOutgoing(a);
  return Barrier(a);
}

void Runner::PlanAllLocally(App& a) {
  const Team team = *a.remote->team();
  for (const Unit& u : a.game.GetScene().units) {
    if (u.team != team || !u.alive || u.plan.type != PlannedActionType::None) continue;
    bool done = false;
    for (int i = 0; i < 16 && !done; ++i) {
      const float ang = static_cast<float>(i) * 0.3927f;
      const glm::vec3 dest = u.position + glm::vec3(std::cos(ang) * 5.0f, 0.0f, std::sin(ang) * 5.0f);
      a.game.ClickUnit(u.id, team);
      a.game.ChooseMove();
      a.game.ClickGround(dest, team);
      a.game.FinishMovePlan();
      done = a.game.Mode() == InputMode::AwaitingSelection;
      if (!done) {
        a.game.CancelAction();
        a.game.CancelAction();
      }
    }
    if (!done) {
      a.game.ClickUnit(u.id, team);
      a.game.ChoosePass();
    }
  }
}

bool Runner::PlayRound(const std::vector<App*>& who) {
  for (App* a : who) {
    PlanAllLocally(*a);
    if (!Sync(*a)) return false;
  }
  for (App* a : who) {
    if (!CommitApp(*a)) return false;
  }
  return Settle();
}

bool Runner::StartMatchClient(App& a, std::unique_ptr<net::RemoteClient> client, bool waitForMatch) {
  a.remote = std::move(client);
  a.remote->SetToken(a.token);
  a.error.clear();
  a.reveal.clear();
  if (!Connect(a, false)) return false;
  using Status = net::RemoteClient::Status;
  if (!PumpUntil([&] { return a.remote->status() != Status::Connecting; })) {
    return Fail(a.name + ": the server never answered the join");
  }
  if (a.remote->status() == Status::Rejected) return true;  // The caller reverts the screen.
  if (waitForMatch &&
      !PumpUntil([&] { return a.remote->status() == Status::Playing; })) {
    return Fail(a.name + ": match never started (" + a.remote->StatusText() + ")");
  }
  return Settle();
}

bool Runner::Click(App& a, const std::string& button, int row) {
  flow::Event event;
  if (a.screen == flow::State::MyGames && (button == "Resume" || button == "Replay")) {
    if (row < 0 || row >= static_cast<int>(a.games.entries().size())) {
      return Fail(a.name + ": no row " + std::to_string(row) + " on My games");
    }
    const bool finished = a.games.entries()[row].finished;
    if (finished != (button == "Replay")) {
      return Fail(a.name + ": row " + std::to_string(row) + " offers " +
                  (finished ? "Replay" : "Resume") + ", not " + button);
    }
    event = flow::Event::OpenGame;
  } else {
    const auto found = ui::FindMenuButton(a.screen, button.c_str());
    if (!found) {
      return Fail(a.name + ": no button '" + button + "' on the " + ScreenName(a.screen) + " screen");
    }
    event = *found;
  }
  using Status = net::RemoteClient::Status;
  switch (event) {
    case flow::Event::SelectMyGames:
      a.games = net::GamesClient(a.token);
      if (!Connect(a, true)) return false;
      if (!PumpUntil([&] { return a.games.loaded(); })) {
        return Fail(a.name + ": My games never loaded");
      }
      break;
    case flow::Event::Back:
      if (a.screen == flow::State::MyGames) Disconnect(a);
      break;
    case flow::Event::CreateRoom:
    case flow::Event::CreateAiRoom:
    case flow::Event::JoinRoom: {
      const bool ai = event == flow::Event::CreateAiRoom;
      const bool create = ai || event == flow::Event::CreateRoom;
      if (!create && a.typedCode.empty()) return Fail(a.name + ": Join needs a typed room code");
      a.fromResume = false;
      auto client = std::make_unique<net::RemoteClient>(
          &a.game, create ? "" : a.typedCode, create ? "create" : "join", false);
      if (ai) client->SetBotMode("player-v-ai");  // The match starts at once.
      if (!StartMatchClient(a, std::move(client), /*waitForMatch=*/!create || ai)) return false;
      if (create && !ai && a.remote->status() != Status::Waiting) {
        return Fail(a.name + ": new room did not reach 'waiting': " + a.remote->StatusText());
      }
      if (a.remote->status() == Status::Rejected) {
        a.remote.reset();
        Disconnect(a);
        return true;  // Stays on the Room screen (the error is in a.error).
      }
      break;
    }
    case flow::Event::ContinueGame:
    case flow::Event::OpenGame: {
      std::string id = a.lastGameId;
      if (event == flow::Event::OpenGame) {
        id = a.games.entries()[row].id;
      } else if (a.screen == flow::State::MyGames && a.games.LatestActive()) {
        id = a.games.LatestActive()->id;
      }
      if (id.empty()) return Fail(a.name + ": Continue is not offered (no game to resume)");
      Disconnect(a);
      a.fromResume = true;
      auto client = std::make_unique<net::RemoteClient>(&a.game, "", "", false);
      client->SetResume(id);
      if (!StartMatchClient(a, std::move(client), /*waitForMatch=*/true)) return false;
      if (a.remote->status() == Status::Rejected) {
        // As the app does: a stored game that is gone is forgotten.
        a.lastGameId.clear();
        a.screen = flow::State::MyGames;
        a.remote.reset();
        Disconnect(a);
        return true;
      }
      break;
    }
    default: break;
  }
  a.screen = flow::Next(a.screen, event);
  if (a.remote && !a.remote->game_id().empty()) a.lastGameId = a.remote->game_id();
  return true;
}

// Closing the tab / killing the app: the socket goes away without a word and
// the page starts over at the splash. The token and last game (localStorage)
// stay.
bool Runner::Reload(App& a) {
  Disconnect(a);
  a.remote.reset();
  a.screen = flow::kInitialState;
  a.typedCode.clear();
  a.error.clear();
  a.reveal.clear();
  a.games = net::GamesClient(a.token);
  a.game.Reset(Scene{});
  return true;
}

Snapshot Runner::Take(App& a) {
  Snapshot s;
  s.screen = ScreenName(a.screen);
  s.round = a.game.RoundNumber();
  s.over = a.game.Mode() == InputMode::GameOver;
  if (a.remote) {
    s.selfReady = a.remote->selfReady();
    s.peerReady = a.remote->peerReady();
    s.gameId = a.remote->game_id();
  }
  char buf[96];
  for (const Unit& u : a.game.GetScene().units) {
    if (u.position.x >= net::kHiddenCoord * 0.5f) continue;  // Unseen enemy.
    std::snprintf(buf, sizeof buf, "%d:%d:%.2f:%.2f;", u.id, u.alive ? 1 : 0, u.position.x,
                  u.position.z);
    s.units += buf;
  }
  s.reveal = a.reveal;
  return s;
}

bool Runner::Assert(const YAML::Node& node) {
  App* a = Get(node);
  if (!a) return false;
  Settle();
  bool ok = true;
  auto check = [&](bool cond, const std::string& what) {
    if (!cond) ok = Fail(a->name + ": expected " + what);
  };
  const Snapshot now = Take(*a);
  if (node["screen"]) {
    check(now.screen == node["screen"].as<std::string>(),
          "screen " + node["screen"].as<std::string>() + ", got " + now.screen);
  }
  if (node["status"]) {
    const std::string got = a->remote ? StatusName(a->remote->status()) : "none";
    check(got == node["status"].as<std::string>(),
          "status " + node["status"].as<std::string>() + ", got " + got);
  }
  if (node["round"]) {
    check(now.round == node["round"].as<int>(),
          "round " + node["round"].as<std::string>() + ", got " + std::to_string(now.round));
  }
  if (node["self_ready"]) check(now.selfReady == node["self_ready"].as<bool>(), "self_ready mismatch");
  if (node["peer_ready"]) check(now.peerReady == node["peer_ready"].as<bool>(), "peer_ready mismatch");
  if (node["peer_away"]) {
    check((a->remote && a->remote->peerAway()) == node["peer_away"].as<bool>(), "peer_away mismatch");
  }
  if (node["game_over"]) check(now.over == node["game_over"].as<bool>(), "game_over mismatch");
  if (node["has_reveal"]) {
    check(!a->reveal.empty() == node["has_reveal"].as<bool>(), "has_reveal mismatch");
  }
  if (node["error"]) {
    check(a->error.find(node["error"].as<std::string>()) != std::string::npos,
          "error containing '" + node["error"].as<std::string>() + "', got '" + a->error + "'");
  }
  if (node["games_total"]) {
    check(static_cast<int>(a->games.entries().size()) == node["games_total"].as<int>(),
          "games_total " + node["games_total"].as<std::string>() + ", got " +
              std::to_string(a->games.entries().size()));
  }
  auto countStatus = [&](bool finished) {
    int n = 0;
    for (const auto& e : a->games.entries()) n += e.finished == finished ? 1 : 0;
    return n;
  };
  if (node["games_active"]) check(countStatus(false) == node["games_active"].as<int>(), "games_active mismatch");
  if (node["games_finished"]) check(countStatus(true) == node["games_finished"].as<int>(), "games_finished mismatch");
  if (node["same_as"]) {
    const std::string name = node["same_as"].as<std::string>();
    auto it = snapshots_.find(name);
    if (it == snapshots_.end()) return Fail("no snapshot named '" + name + "'");
    const Snapshot& then = it->second;
    const bool compareReveal = !node["ignore_reveal"] || !node["ignore_reveal"].as<bool>();
    check(now.round == then.round, "same round as snapshot " + name);
    check(now.units == then.units, "same units as snapshot " + name + "\n    then: " + then.units +
                                       "\n    now:  " + now.units);
    check(now.selfReady == then.selfReady && now.peerReady == then.peerReady && now.over == then.over,
          "same ready/over flags as snapshot " + name);
    check(now.gameId == then.gameId, "same game id as snapshot " + name);
    if (compareReveal) check(now.reveal == then.reveal, "same reveal as snapshot " + name);
  }
  if (node["game_id_differs_from"]) {
    App* other = Get(node, "game_id_differs_from");
    if (!other) return false;
    check(!other->lastGameId.empty() && a->lastGameId != other->lastGameId,
          "a different game than " + other->name + "'s");
    for (const auto& e : a->games.entries()) {
      check(e.id != other->lastGameId, "my games not to list " + other->name + "'s game");
    }
  }
  return ok;
}

bool Runner::Step(const YAML::Node& step) {
  if (!step.IsMap() || step.size() != 1) return Fail("a step is a one-key map");
  const std::string verb = step.begin()->first.as<std::string>();
  const YAML::Node arg = step.begin()->second;
  auto appsOf = [&](const YAML::Node& list, std::vector<App*>* out) {
    for (const YAML::Node& n : list) {
      auto it = apps_.find(n.as<std::string>());
      if (it == apps_.end()) return Fail("unknown app '" + n.as<std::string>() + "'");
      out->push_back(it->second.get());
    }
    return true;
  };
  if (verb == "server") {
    const std::string what = arg.as<std::string>();
    if (what == "start") return StartServer();
    if (what == "stop") {
      server_.Stop();
      return Settle();
    }
    if (what == "restart" || what == "crash") {
      if (what == "crash") server_.Crash();
      else server_.Stop();
      Pump(50);  // Clients notice the closed sockets.
      return StartServer();
    }
    return Fail("unknown server action " + what);
  }
  if (verb == "app") {
    for (const YAML::Node& n : arg) {
      const std::string name = n.as<std::string>();
      // A fixed per-app device token (what localStorage would hold).
      const std::string token = "device-token-" + name + "-0123456789";
      apps_[name] = std::make_unique<App>(name, token);
    }
    return true;
  }
  if (verb == "click") {
    App* a = Get(arg);
    return a && Click(*a, arg["button"].as<std::string>(), arg["row"] ? arg["row"].as<int>() : -1);
  }
  if (verb == "type_room_code") {
    App* a = Get(arg);
    App* from = Get(arg, "from");
    if (!a || !from) return false;
    if (!from->remote || from->remote->room_code().empty()) {
      return Fail(from->name + " has no room code to type");
    }
    a->typedCode = from->remote->room_code();
    return true;
  }
  if (verb == "round") {
    std::vector<App*> who;
    if (!appsOf(arg["apps"], &who)) return false;
    const int count = arg["count"] ? arg["count"].as<int>() : 1;
    for (int i = 0; i < count; ++i) {
      if (!PlayRound(who)) return false;
    }
    return true;
  }
  if (verb == "plan" || verb == "commit") {
    // plan: plans (and ships) one app's figures; commit: plans and commits it.
    auto it = apps_.find(arg.as<std::string>());
    if (it == apps_.end()) return Fail("unknown app '" + arg.as<std::string>() + "'");
    App* a = it->second.get();
    if (!a->remote) return Fail(a->name + " is not in a match");
    PlanAllLocally(*a);
    if (!Sync(*a)) return false;
    if (verb == "commit" && !CommitApp(*a)) return false;
    return Settle();
  }
  if (verb == "reload") {
    auto it = apps_.find(arg.as<std::string>());
    if (it == apps_.end()) return Fail("unknown app");
    return Reload(*it->second);
  }
  if (verb == "settle") return Settle();
  if (verb == "snapshot") {
    App* a = Get(arg);
    if (!a) return false;
    Settle();
    snapshots_[arg["name"].as<std::string>()] = Take(*a);
    return true;
  }
  if (verb == "assert") return Assert(arg);
  return Fail("unknown step '" + verb + "'");
}

bool Runner::Run(std::vector<std::string>* failures) {
  YAML::Node root;
  try {
    root = YAML::LoadFile(path_);
  } catch (const YAML::Exception& e) {
    failures->push_back("cannot parse " + path_ + ": " + e.what());
    return false;
  }
  dbPath_ = (fs::temp_directory_path() / ("tbgwaf_integ_" + std::to_string(::getpid()) + "_" +
                                           fs::path(path_).stem().string() + ".db")).string();
  for (const char* suffix : {"", "-wal", "-shm"}) std::remove((dbPath_ + suffix).c_str());
  if (root["round_limit"]) roundLimit_ = root["round_limit"].as<int>();
  try {
    for (const YAML::Node& step : root["steps"]) {
      const int64_t began = NowMs();
      const bool ok = Step(step);
      if (std::getenv("INTEG_TRACE")) {
        std::fprintf(stderr, "step %d (%lld ms): %s\n", step_ + 1,
                     static_cast<long long>(NowMs() - began), YAML::Dump(step).c_str());
      }
      if (!ok || !failures_.empty()) break;
      ++step_;
    }
  } catch (const YAML::Exception& e) {
    Fail(std::string("bad step: ") + e.what());
  }
  *failures = failures_;
  return failures_.empty();
}

}  // namespace

int main(int argc, char** argv) {
  std::string server = (fs::absolute(fs::path(argv[0])).parent_path() / "tbgwaf_server_testctl").string();
  std::string dir = "tests/integration";
  std::string only;
  int shardIndex = 0, shardCount = 1;
  bool list = false, timings = false;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--server" && i + 1 < argc) server = argv[++i];
    else if (arg == "--dir" && i + 1 < argc) dir = argv[++i];
    else if (arg == "--only" && i + 1 < argc) only = argv[++i];
    else if (arg == "--list") list = true;
    else if (arg == "--timings") timings = true;
    else if (arg == "--shard" && i + 1 < argc) {
      if (std::sscanf(argv[++i], "%d/%d", &shardIndex, &shardCount) != 2 || shardCount < 1 ||
          shardIndex < 0 || shardIndex >= shardCount) {
        std::fprintf(stderr, "bad --shard (want i/N, 0 <= i < N)\n");
        return 2;
      }
    } else {
      std::fprintf(stderr,
                   "usage: %s [--server BIN] [--dir DIR] [--list] [--only NAME] [--shard i/N] "
                   "[--timings]\n", argv[0]);
      return 2;
    }
  }
  std::vector<fs::path> files;
  if (fs::is_directory(dir)) {
    for (const auto& entry : fs::directory_iterator(dir)) {
      if (entry.is_regular_file() && entry.path().extension() == ".yaml") files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {
    std::fprintf(stderr, "no integration scenarios under %s\n", dir.c_str());
    return 2;
  }
  if (list) {
    for (const auto& f : files) std::printf("%s\n", f.stem().string().c_str());
    return 0;
  }
  signal(SIGPIPE, SIG_IGN);
  int failed = 0, ran = 0;
  for (size_t i = 0; i < files.size(); ++i) {
    const std::string name = files[i].stem().string();
    if (!only.empty() && only != name) continue;
    if (static_cast<int>(i % static_cast<size_t>(shardCount)) != shardIndex) continue;
    ++ran;
    const int64_t start = NowMs();
    std::vector<std::string> failures;
    {
      Runner runner(server, files[i].string());
      runner.Run(&failures);
    }
    if (failures.empty()) {
      std::printf("PASS %s", name.c_str());
    } else {
      ++failed;
      std::printf("FAIL %s", name.c_str());
    }
    if (timings) std::printf(" (%.1fs)", static_cast<double>(NowMs() - start) / 1000.0);
    std::printf("\n");
    for (const std::string& f : failures) std::fprintf(stderr, "  %s\n", f.c_str());
  }
  if (ran == 0) {
    std::fprintf(stderr, "no scenario matched\n");
    return 2;
  }
  if (failed) {
    std::fprintf(stderr, "%d of %d integration scenario(s) failed.\n", failed, ran);
    return 1;
  }
  std::printf("All %d integration scenario(s) passed.\n", ran);
  return 0;
}
