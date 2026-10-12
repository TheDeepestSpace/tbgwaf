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
#include "NetRig.h"
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

using testing::NowMs;
using testing::kWaitMs;
using testing::ServerProcess;
using testing::Player;
using testing::Rig;

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

  {
    std::string error;
    if (!testing::LoadRoom(control, scenario, room, &error)) return {error};
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
    // The turn timeline/replay (issue #143) is purely client-side view state,
    // with nothing to round-trip through the server; it is covered by the
    // in-process and visual runners.
    const bool usesTimeline = std::any_of(
        scenario.steps.begin(), scenario.steps.end(), [](const scenario::ScenarioStep& st) {
          if (st.assertion) return st.assertion->timelineTicks.has_value();
          using K = ScenarioAction::Kind;
          return st.action->kind == K::TimelineSeek || st.action->kind == K::TimelinePlay ||
                 st.action->kind == K::TimelinePause;
        });
    if (usesTimeline) {
      std::printf("SKIP %s (client-side timeline)\n", scenario.name.c_str());
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
