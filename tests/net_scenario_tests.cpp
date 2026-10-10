// Networked scenario test: replays every YAML gameplay scenario through a
// real tbgwaf server process instead of an in-process GameLogic.
//
//   tactics_net_scenario_tests <tbgwaf_server_testctl> [scenario dir]
//
// Topology (see README "Testing"):
//   * the runner spawns the test-control server build on ephemeral ports;
//   * per scenario it loads the scene into a fresh room over the control tap;
//   * it connects one headless client per team on the normal game port -- a
//     GameLogic mirror + net::RemoteClient over a native WebSocket, no GL --
//     and plays the script through each actor's own mirror (the same
//     ClickUnit/ChooseMove/... calls as the in-process runner), letting
//     RemoteClient ship the resulting plans to the server;
//   * assertions are checked against the server's unfiltered state (control
//     `dump_state`), and each client's fog-filtered mirror is cross-checked
//     against it.
// Scripts, YAML and assertion semantics are shared with the in-process
// runner; what this adds is the server, room, serialization and fog path.

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>
#include <poll.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "game/GameLogic.h"
#include "net/Json.h"
#include "net/Protocol.h"
#include "net/RemoteClient.h"
#include "WsClient.h"
#include "scenario/Scenario.h"

namespace fs = std::filesystem;
using namespace tactics;
using net::Json;
using scenario::Scenario;
using scenario::ScenarioAction;
using scenario::ScenarioAssertion;
using scenario::ScenarioResult;
using testing::WsClient;

namespace {

constexpr int kWaitMs = 10000;

int64_t NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

// ---- Server process ----

struct ServerProcess {
  pid_t pid = -1;
  uint16_t gamePort = 0;
  uint16_t controlPort = 0;

  bool Start(const std::string& binary, std::string* error) {
    int fds[2];
    if (pipe(fds) != 0) return false;
    pid = fork();
    if (pid == 0) {
      dup2(fds[1], STDOUT_FILENO);
      close(fds[0]);
      close(fds[1]);
      // Scenario scripts are bursty; lift the per-connection message cap.
      execl(binary.c_str(), binary.c_str(), "--port", "0", "--control-port", "0", "--rate-limit",
            "100000", static_cast<char*>(nullptr));
      _exit(127);
    }
    close(fds[1]);
    // Read the startup banner: "...listening on A:PORT" then "...control tap on A:PORT".
    std::string text;
    const int64_t deadline = NowMs() + kWaitMs;
    while ((gamePort == 0 || controlPort == 0) && NowMs() < deadline) {
      pollfd p{fds[0], POLLIN, 0};
      if (poll(&p, 1, 100) <= 0) continue;
      char buf[256];
      const ssize_t n = read(fds[0], buf, sizeof buf);
      if (n <= 0) break;
      text.append(buf, static_cast<size_t>(n));
      auto portAfter = [&](const char* marker) -> uint16_t {
        const size_t at = text.find(marker);
        if (at == std::string::npos) return 0;
        const size_t colon = text.find(':', at);
        const size_t eol = text.find('\n', at);
        if (colon == std::string::npos || eol == std::string::npos) return 0;
        return static_cast<uint16_t>(std::atoi(text.c_str() + colon + 1));
      };
      gamePort = portAfter("listening on");
      controlPort = portAfter("control tap on");
    }
    close(fds[0]);
    if (gamePort == 0 || controlPort == 0) {
      *error = "server did not report its ports (is it the test-control build?)";
      Stop();
      return false;
    }
    return true;
  }

  void Stop() {
    if (pid <= 0) return;
    kill(pid, SIGTERM);
    int status = 0;
    waitpid(pid, &status, 0);
    pid = -1;
  }
};

// ---- One headless team client ----

struct Player {
  Team team;
  GameLogic game;
  net::RemoteClient client;
  WsClient ws;
  int barrierSeq = 0;
  bool barrierAcked = false, barrierDone = false;

  Player(Team t, const Scene& scene, const std::string& room)
      : team(t), game(scene), client(&game, room) {}
};

class Rig {
 public:
  Rig(uint16_t gamePort, WsClient* control) : gamePort_(gamePort), control_(control) {}

  std::vector<std::string> failures;

  bool Join(const Scenario& scenario, const std::string& room) {
    for (Team team : {Team::Blue, Team::Red}) {
      players_[static_cast<int>(team)] = std::make_unique<Player>(team, scenario.scene, room);
      Player& p = *players_[static_cast<int>(team)];
      std::string error;
      if (!p.ws.Connect(gamePort_, &error)) return Fail("connect: " + error);
      p.client.OnOpen();
      SendOutgoing(p);
      const auto want = team == Team::Blue ? net::RemoteClient::Status::Waiting
                                           : net::RemoteClient::Status::Playing;
      if (!PumpUntil([&] { return p.client.status() == want; })) {
        return Fail(std::string(net::TeamName(team)) + " client never reached expected status: " +
                    p.client.StatusText());
      }
    }
    return PumpUntil([&] { return Blue().client.status() == net::RemoteClient::Status::Playing; }) &&
           Barrier(Blue()) && Barrier(Red());
  }

  Player& Blue() { return *players_[0]; }
  Player& Red() { return *players_[1]; }
  Player& For(Team t) { return *players_[static_cast<int>(t)]; }

  bool Fail(const std::string& why) {
    failures.push_back(why);
    return false;
  }

  // Receives everything pending on both clients (waits up to `ms` for any).
  void Pump(int ms) {
    for (auto& p : players_) {
      if (!p) continue;
      std::vector<std::string> messages;
      if (!p->ws.Poll(ms, &messages) && p->client.status() != net::RemoteClient::Status::Disconnected) {
        p->client.OnClose();
      }
      for (const std::string& text : messages) Dispatch(*p, text);
      ms = 0;  // Only block on the first socket.
    }
  }

  template <typename Pred>
  bool PumpUntil(Pred done) {
    const int64_t deadline = NowMs() + kWaitMs;
    while (!done()) {
      if (NowMs() > deadline) return false;
      Pump(5);
    }
    return true;
  }

  void SendOutgoing(Player& p) {
    for (const std::string& frame : p.client.TakeOutgoing()) p.ws.Send(frame);
  }

  // Ships any plan changes the client's mirror holds, then waits until the
  // server has processed them: a no-op `reaction` (the client's own current
  // playbook) is acked, and answered with the authoritative `plans`, strictly
  // after everything sent before it on the same connection.
  bool Sync(Player& p) {
    p.client.Update(0.0f);
    SendOutgoing(p);
    return Barrier(p);
  }

  bool Barrier(Player& p) {
    net::Action ping;
    ping.kind = net::ActionKind::Reaction;
    ping.playbook = p.game.Playbook(p.team);
    ping.seq = 1000000 + (++p.barrierSeq);
    p.barrierAcked = p.barrierDone = false;
    p.ws.Send(net::EncodeAction(ping).Dump());
    if (!PumpUntil([&] { return p.barrierDone; })) {
      return Fail(std::string(net::TeamName(p.team)) + " barrier timed out (server stopped answering?)");
    }
    return true;
  }

  bool Commit() {
    for (Player* p : {&Blue(), &Red()}) {
      if (!p->game.CanCommitRound()) {
        return Fail("cannot commit: not every living figure (on both teams) has a plan yet");
      }
    }
    Blue().client.RequestCommit();
    SendOutgoing(Blue());
    if (!Barrier(Blue())) return false;
    Red().client.RequestCommit();
    SendOutgoing(Red());
    // Red's commit runs the round; its barrier is acked after the round
    // messages, then blue's (a separate connection) after the same.
    return Barrier(Red()) && Barrier(Blue()) && PlayOut();
  }

  // Advances both mirrors' round playback to the end, as the frame loop would.
  bool PlayOut() {
    for (int i = 0; i < 4000; ++i) {
      if (!Blue().client.playingBack() && !Red().client.playingBack()) return true;
      for (Player* p : {&Blue(), &Red()}) {
        p->client.Update(0.1f);
        p->game.UpdateSightingMemory(0.1f);
      }
    }
    return Fail("round playback never finished");
  }

  bool NewMatch() {
    if (Blue().game.Mode() != InputMode::GameOver) {
      return Fail("new_game is only valid after game over in the networked runner");
    }
    Blue().client.RequestNewMatch();
    SendOutgoing(Blue());
    if (!Barrier(Blue()) || !Barrier(Red())) return false;
    for (Player* p : {&Blue(), &Red()}) {
      p->client.ConsumeMatchStarted();
      p->game.UpdateSightingMemory(0.0f);
    }
    return true;
  }

  // Server-side ground truth for the room (control tap).
  bool DumpState(const std::string& room, Json* dump) {
    Json request;
    request.Set("t", "dump_state");
    request.Set("room", room);
    control_->Send(request.Dump());
    const int64_t deadline = NowMs() + kWaitMs;
    std::vector<std::string> messages;
    while (NowMs() < deadline) {
      if (!control_->Poll(20, &messages)) return Fail("control tap closed");
      if (!messages.empty()) {
        if (!Json::Parse(messages.front(), dump) || (*dump)["t"].AsString() != "state_dump") {
          return Fail("dump_state failed: " + messages.front());
        }
        return true;
      }
    }
    return Fail("dump_state timed out");
  }

 private:
  void Dispatch(Player& p, const std::string& text) {
    // NETDBG=1 traces every server message a client receives.
    if (std::getenv("NETDBG")) std::fprintf(stderr, "[%s] %.300s\n", net::TeamName(p.team), text.c_str());
    Json m;
    if (Json::Parse(text, &m)) {
      const std::string& type = m["t"].AsString();
      const int seq = static_cast<int>(m["seq"].AsNumber(0));
      const bool isBarrier = type == "ack" && seq == 1000000 + p.barrierSeq;
      if (type == "ack" && !m["ok"].AsBool(true) && !isBarrier) {
        Fail(std::string("server rejected ") + net::TeamName(p.team) + "'s action: " + m["error"].AsString());
      }
      if (type == "error") Fail(std::string("server error to ") + net::TeamName(p.team) + ": " + m["error"].AsString());
      if (isBarrier) p.barrierAcked = true;
      else if (type == "plans" && p.barrierAcked) p.barrierDone = true;
    }
    p.client.OnMessage(text);
  }

  uint16_t gamePort_;
  WsClient* control_;
  std::unique_ptr<Player> players_[2];
};

// ---- Assertions ----

bool Hidden(const glm::vec3& p) { return p.x >= net::kHiddenCoord * 0.5f; }

// Rebuilds a GameLogic carrying the server's unfiltered state so the shared
// scenario assertion code (positions, facing, visibility, winner, round) can
// run against ground truth unchanged.
GameLogic OracleFromDump(const Scenario& scenario, const Json& dump) {
  GameLogic oracle(scenario.scene);
  GameSnapshot snap = oracle.ExportState();
  for (auto& u : snap.units) {
    for (const Json& e : dump["units"].AsArray()) {
      if (static_cast<int>(e["id"].AsNumber(-1)) != u.id) continue;
      net::DecodeVec3(e["pos"], &u.position);
      u.facingYaw = static_cast<float>(e["yaw"].AsNumber());
      u.alive = e["alive"].AsBool(true);
      u.knockdownElapsed = u.alive ? -1.0f : constants::kKnockdownDuration;
    }
  }
  snap.roundNumber = static_cast<int>(dump["round"].AsNumber(1));
  snap.mode = dump["over"].AsBool() ? InputMode::GameOver : InputMode::AwaitingSelection;
  const auto winner = net::ParseTeamName(dump["winner"].AsString());
  snap.winner = winner ? static_cast<int>(*winner) : -1;
  if (dump["flag"].IsObject()) {
    const Json& f = dump["flag"];
    snap.flag.carrierId = static_cast<int>(f["carrier"].AsNumber(-1));
    snap.flag.dropElapsed = f["dropped"].AsBool() ? 0.0f : -1.0f;
    net::DecodeVec3(f["pos"], &snap.flag.position);
  }
  oracle.ImportState(snap);
  return oracle;
}

bool DumpSees(const Json& dump, Team viewer, int unitId) {
  for (const Json& id : dump[std::string("visible_") + net::TeamName(viewer)].AsArray()) {
    if (static_cast<int>(id.AsNumber(-1)) == unitId) return true;
  }
  return false;
}

struct AssertionStats {
  int skippedMemory = 0;
};

void CheckNetAssertion(const Scenario& scenario, Rig& rig, const Json& dump,
                       const ScenarioAssertion& a, int step, AssertionStats* stats) {
  ScenarioResult result;
  GameLogic oracle = OracleFromDump(scenario, dump);

  // 1. Ground truth, via the shared assertion code (sighting memory is a
  //    client-side presentation concern, handled below).
  ScenarioAssertion truth = a;
  truth.rememberedByTeam.reset();
  truth.remembered.reset();
  truth.memoryAge.reset();
  scenario::CheckAssertion(oracle, truth, step, &result);

  if (a.unit) {
    // 2. The server's fog computation must match the oracle's recomputation.
    for (Team viewer : {Team::Blue, Team::Red}) {
      const bool expected = oracle.ComputeVisibility(viewer).UnitVisible(*a.unit);
      if (DumpSees(dump, viewer, *a.unit) != expected) {
        result.failures.push_back("step " + std::to_string(step) + " (assert): server's visibility of unit " +
                                  std::to_string(*a.unit) + " to " + net::TeamName(viewer) +
                                  " disagrees with the recomputed value");
      }
    }
    // 3. Each client's fog-filtered mirror must show exactly what its team
    //    may see: the real state if visible (or own), otherwise nothing.
    const Unit* truthUnit = oracle.FindUnit(*a.unit);
    for (Team viewer : {Team::Blue, Team::Red}) {
      const Unit* seen = rig.For(viewer).game.FindUnit(*a.unit);
      if (!truthUnit || !seen) continue;
      const bool visible = truthUnit->team == viewer || DumpSees(dump, viewer, *a.unit);
      const std::string who = std::string(net::TeamName(viewer)) + " client";
      if (visible) {
        if (seen->alive != truthUnit->alive ||
            glm::distance(seen->position, truthUnit->position) > std::max(a.tolerance, 0.05f)) {
          result.failures.push_back("step " + std::to_string(step) + " (assert): " + who +
                                    " mirror of unit " + std::to_string(*a.unit) +
                                    " disagrees with the server's state");
        }
      } else if (!Hidden(seen->position)) {
        result.failures.push_back("step " + std::to_string(step) + " (assert): " + who +
                                  " can see unit " + std::to_string(*a.unit) +
                                  " although the server's fog hides it from that team");
      }
    }
  }

  // 4. Sighting memory lives on the owning client's mirror. Like the
  //    in-process runner's follower pane, only the freshest sample (age 0)
  //    is comparable across a snapshot-fed mirror.
  if (a.rememberedByTeam) {
    if (a.memoryAge && *a.memoryAge != 0) {
      ++stats->skippedMemory;
    } else {
      ScenarioAssertion memory;
      memory.unit = a.unit;
      memory.rememberedByTeam = a.rememberedByTeam;
      memory.remembered = a.remembered;
      memory.memoryAge = a.memoryAge;
      ScenarioResult mirrorResult;
      scenario::CheckAssertion(rig.For(*a.rememberedByTeam).game, memory, step, &mirrorResult);
      for (auto& f : mirrorResult.failures) result.failures.push_back("client " + f);
    }
  }
  for (auto& f : result.failures) rig.failures.push_back(std::move(f));
}

// ---- Scenario driver ----

std::vector<std::string> RunNetScenario(const Scenario& scenario, int index, ServerProcess& server,
                                        WsClient& control, AssertionStats* stats) {
  const std::string room = "scn" + std::to_string(index);
  Rig rig(server.gamePort, &control);

  Json load;
  load.Set("t", "load_scenario");
  load.Set("room", room);
  load.Set("scene", scenario.sceneSpec);
  load.Set("friendly_fire", Json(scenario.friendlyFire));
  if (!scenario.shotRolls.empty()) {
    Json rolls{Json::Array{}};
    for (float r : scenario.shotRolls) rolls.Push(Json(static_cast<double>(r)));
    load.Set("shot_rolls", std::move(rolls));
  }
  control.Send(load.Dump());
  {
    std::vector<std::string> reply;
    const int64_t deadline = NowMs() + kWaitMs;
    while (reply.empty() && NowMs() < deadline) control.Poll(20, &reply);
    Json parsed;
    if (reply.empty() || !Json::Parse(reply.front(), &parsed) || !parsed["ok"].AsBool()) {
      return {"load_scenario failed: " + (reply.empty() ? std::string("no reply") : reply.front())};
    }
  }
  if (!rig.Join(scenario, room)) return rig.failures;
  // Scenarios default to passive playbooks (the live default reacts); set
  // them the way a player would, through each client's own mirror.
  for (Team team : {Team::Blue, Team::Red}) {
    Player& p = rig.For(team);
    p.game.SetPlaybook(team, scenario.playbooks[static_cast<int>(team)]);
    if (!rig.Sync(p)) return rig.failures;
  }

  for (int i = 0; i < static_cast<int>(scenario.steps.size()) && rig.failures.empty(); ++i) {
    const scenario::ScenarioStep& step = scenario.steps[i];
    if (step.action) {
      const ScenarioAction& action = *step.action;
      if (action.kind == ScenarioAction::Kind::Commit) {
        rig.Commit();
      } else if (action.kind == ScenarioAction::Kind::NewGame) {
        rig.NewMatch();
      } else {
        const Unit* actor = nullptr;
        for (const Unit& u : scenario.scene.units) if (u.id == action.actor) actor = &u;
        if (!actor) {
          rig.Fail("step " + std::to_string(i) + ": unknown actor " + std::to_string(action.actor));
          break;
        }
        Player& p = rig.For(actor->team);
        ScenarioResult local;
        const bool ok = scenario::ExecuteAction(p.game, scenario.scene, action, i, &local);
        for (auto& f : local.failures) rig.failures.push_back(f);
        if (ok) rig.Sync(p);
      }
    } else {
      Json dump;
      if (!rig.DumpState(room, &dump)) break;
      CheckNetAssertion(scenario, rig, dump, *step.assertion, i, stats);
    }
  }
  return rig.failures;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <tbgwaf_server_testctl> [scenario dir]\n", argv[0]);
    return 2;
  }
  const fs::path dir = argc > 2 ? fs::path(argv[2]) : fs::path("tests/scenarios");
  std::vector<fs::path> files;
  if (fs::is_directory(dir)) {
    for (const auto& entry : fs::directory_iterator(dir)) {
      const auto ext = entry.path().extension();
      if (entry.is_regular_file() && (ext == ".yaml" || ext == ".yml")) files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  if (files.empty()) {
    std::fprintf(stderr, "no scenario YAML files found under %s\n", dir.string().c_str());
    return 1;
  }

  signal(SIGPIPE, SIG_IGN);
  ServerProcess server;
  std::string error;
  if (!server.Start(argv[1], &error)) {
    std::fprintf(stderr, "FAIL: %s\n", error.c_str());
    return 1;
  }
  WsClient control;
  if (!control.Connect(server.controlPort, &error)) {
    std::fprintf(stderr, "FAIL: control tap: %s\n", error.c_str());
    server.Stop();
    return 1;
  }

  int failed = 0, index = 0;
  AssertionStats stats;
  for (const auto& file : files) {
    Scenario scenario;
    try {
      scenario = scenario::LoadScenarioFromFile(file.string());
    } catch (const std::exception& e) {
      std::fprintf(stderr, "FAIL %s: %s\n", file.string().c_str(), e.what());
      ++failed;
      continue;
    }
    const auto failures = RunNetScenario(scenario, index++, server, control, &stats);
    if (failures.empty()) {
      std::printf("PASS %s (%s)\n", scenario.name.c_str(), file.string().c_str());
    } else {
      std::fprintf(stderr, "FAIL %s (%s):\n", scenario.name.c_str(), file.string().c_str());
      for (const auto& f : failures) std::fprintf(stderr, "  %s\n", f.c_str());
      ++failed;
    }
  }
  if (stats.skippedMemory > 0) {
    std::printf("note: %d sighting-memory assertion(s) with memory_age > 0 are only checked in-process\n",
                stats.skippedMemory);
  }

  control.Send(R"({"t":"shutdown"})");
  std::vector<std::string> sink;
  control.Poll(200, &sink);
  server.Stop();
  std::printf("%d/%zu networked scenarios passed\n", static_cast<int>(files.size()) - failed, files.size());
  return failed == 0 ? 0 : 1;
}
