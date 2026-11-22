// Builder scenes: BuildScene / SurfaceHeightAt, the JSON form, and the
// repro-log embedding (header "builder_scene"): a session recorded on a
// hand-built scene replays from the log alone, through both the builder
// replay path and the generic one tactics_repro_replay uses. Old logs
// without the field must still parse and replay. Headless.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "game/BuilderScene.h"
#include "game/DemoScenes.h"
#include "game/GameLogic.h"
#include "game/NavMesh.h"
#include "game/ReproLog.h"

namespace {

int g_failures = 0;

void ReportFailure(const char* file, int line, const char* expr) {
  std::fprintf(stderr, "CHECK FAILED at %s:%d: %s\n", file, line, expr);
  ++g_failures;
}

#define CHECK(expr) \
  do { \
    if (!(expr)) ReportFailure(__FILE__, __LINE__, #expr); \
  } while (0)

using namespace tactics;

uint64_t g_clock = 0;
uint64_t FakeNow() { return g_clock += 16; }

// Two crates (one stacked on the other's footprint), a sniper on the tall
// one, a rifleman on the low one, red on open ground.
BuilderScene MakeTestScene() {
  BuilderScene bs;
  bs.name = "Test \"yard\"";
  bs.mapHalfExtent = 18.5f;
  bs.roundLimit = 7;
  bs.boxes.push_back({-8.0f, 0.0f, 1.5f, 1.5f, 2.0f});
  bs.boxes.push_back({-8.0f, 5.0f, 1.0f, 1.0f, 1.2f});
  bs.boxes.push_back({6.1f, -3.3f, 0.7f, 0.9f, 1.2f});
  bs.figures.push_back({Team::Blue, WeaponType::SniperRifle, -8.0f, 0.0f});
  bs.figures.push_back({Team::Blue, WeaponType::AssaultRifle, -8.0f, 5.0f});
  bs.figures.push_back({Team::Red, WeaponType::DesertEagle, 8.0f, 0.1f});
  bs.figures.push_back({Team::Red, WeaponType::AssaultRifle, 7.0f, 6.0f});
  return bs;
}

void TestBuildScene() {
  const BuilderScene bs = MakeTestScene();
  const Scene scene = BuildScene(bs);
  CHECK(scene.ground.Empty());
  CHECK(scene.mapHalfExtent == bs.mapHalfExtent);
  CHECK(scene.roundLimit == 7);
  CHECK(!scene.flag.enabled);
  CHECK(scene.obstacles.size() == 3);
  for (const Obstacle& o : scene.obstacles) {
    CHECK(o.climbable);
    CHECK(o.bounds.min.y == 0.0f);
  }
  CHECK(scene.obstacles[0].bounds.max.y == 2.0f);
  CHECK(scene.obstacles[0].bounds.min.x == -9.5f && scene.obstacles[0].bounds.max.z == 1.5f);

  CHECK(scene.units.size() == bs.figures.size());
  for (size_t i = 0; i < scene.units.size(); ++i) {
    const Unit& u = scene.units[i];
    CHECK(u.id == static_cast<int>(i));
    CHECK(u.team == bs.figures[i].team);
    CHECK(u.weapon == bs.figures[i].weapon);
    CHECK(u.position.x == bs.figures[i].x && u.position.z == bs.figures[i].z);
    CHECK(u.alive);
  }
  CHECK(scene.units[0].weapon == WeaponType::SniperRifle);
  CHECK(scene.units[2].weapon == WeaponType::DesertEagle);
  CHECK(scene.units[0].facingYaw == 0.0f);
  CHECK(scene.units[2].facingYaw > 3.0f);
  CHECK(BuildScene(BuilderScene{}).units.empty());
}

void TestSurfaceHeightAt() {
  BuilderScene bs;
  bs.boxes.push_back({0.0f, 0.0f, 2.0f, 2.0f, 1.0f});
  bs.boxes.push_back({1.0f, 0.0f, 1.0f, 1.0f, 3.0f});  // Overlaps the first, taller.
  bs.boxes.push_back({-1.0f, 0.0f, 0.5f, 0.5f, 0.4f});  // Overlaps, shorter.
  CHECK(SurfaceHeightAt(bs, 5.0f, 5.0f) == 0.0f);
  CHECK(SurfaceHeightAt(bs, -1.9f, 1.9f) == 1.0f);   // Only the wide crate.
  CHECK(SurfaceHeightAt(bs, 1.5f, 0.5f) == 3.0f);    // Highest of the overlap wins.
  CHECK(SurfaceHeightAt(bs, -1.0f, 0.0f) == 1.0f);   // Short box under a taller one.
  CHECK(SurfaceHeightAt(bs, 2.0f, 0.0f) == 3.0f);    // Edges are inclusive.
  CHECK(SurfaceHeightAt(bs, 2.01f, 0.0f) == 0.0f);   // Past every edge.
  CHECK(SurfaceHeightAt(bs, 3.0f, 0.0f) == 0.0f);    // Just off both.
  CHECK(SurfaceHeightAt(BuilderScene{}, 0.0f, 0.0f) == 0.0f);
}

void TestFigureStaysOnBoxAfterReset() {
  const BuilderScene bs = MakeTestScene();
  CHECK(SurfaceHeightAt(bs, -8.0f, 0.0f) == 2.0f);
  const Scene scene = BuildScene(bs);
  CHECK(scene.units[0].position.y == 2.0f);
  CHECK(scene.units[1].position.y == 1.2f);
  CHECK(scene.units[2].position.y == 0.0f);

  GameLogic game;
  InitBuilderGame(&game, bs, 1234u);
  const Scene& live = game.GetScene();
  CHECK(live.units.size() == 4);
  CHECK(live.units[0].position.y == 2.0f);
  CHECK(live.units[1].position.y == 1.2f);
  CHECK(live.units[2].position.y == 0.0f);
  CHECK(live.units[3].position.y == 0.0f);
  CHECK(live.mapHalfExtent == bs.mapHalfExtent);
  // A few idle frames do not drop the figures off their perches.
  for (int i = 0; i < 20; ++i) game.Update(0.0167f);
  CHECK(game.GetScene().units[0].position.y == 2.0f);
  CHECK(game.GetScene().units[1].position.y == 1.2f);
  // Same inputs -> same initial state hash.
  GameLogic other;
  InitBuilderGame(&other, bs, 1234u);
  GameLogic third;
  InitBuilderGame(&third, bs, 1234u);
  CHECK(other.StateHash() == third.StateHash());
}

void TestJsonRoundTrip() {
  BuilderScene bs = MakeTestScene();
  // Awkward floats that need all 9 digits.
  bs.mapHalfExtent = 17.123457f;
  bs.boxes.push_back({0.1f, -0.3f, 0.33333334f, 1e-3f, 12345.679f});
  bs.figures.push_back({Team::Red, WeaponType::SniperRifle, 0.1f, 1.0f / 3.0f});
  const std::string text = SerializeBuilderScene(bs);
  CHECK(text.find('\n') == std::string::npos);

  BuilderScene back;
  std::string error;
  CHECK(ParseBuilderScene(text, &back, &error));
  CHECK(back.name == bs.name);
  CHECK(back.mapHalfExtent == bs.mapHalfExtent);
  CHECK(back.roundLimit == bs.roundLimit);
  CHECK(back.boxes.size() == bs.boxes.size());
  for (size_t i = 0; i < back.boxes.size() && i < bs.boxes.size(); ++i) {
    CHECK(back.boxes[i].cx == bs.boxes[i].cx && back.boxes[i].cz == bs.boxes[i].cz);
    CHECK(back.boxes[i].hx == bs.boxes[i].hx && back.boxes[i].hz == bs.boxes[i].hz);
    CHECK(back.boxes[i].height == bs.boxes[i].height);
  }
  CHECK(back.figures.size() == bs.figures.size());
  for (size_t i = 0; i < back.figures.size() && i < bs.figures.size(); ++i) {
    CHECK(back.figures[i].team == bs.figures[i].team);
    CHECK(back.figures[i].weapon == bs.figures[i].weapon);
    CHECK(back.figures[i].x == bs.figures[i].x && back.figures[i].z == bs.figures[i].z);
  }
  CHECK(SerializeBuilderScene(back) == text);

  // Malformed / out-of-range input is rejected and leaves *out alone.
  BuilderScene untouched;
  untouched.name = "keep";
  CHECK(!ParseBuilderScene("", &untouched, &error));
  CHECK(!ParseBuilderScene("[]", &untouched, &error));
  CHECK(!ParseBuilderScene("{\"name\":\"x\"}", &untouched, &error));
  std::string bad = text;
  bad.replace(bad.find("sniper_rifle"), std::strlen("sniper_rifle"), "bazooka");
  CHECK(!ParseBuilderScene(bad, &untouched, &error));
  CHECK(error.find("bazooka") != std::string::npos);
  CHECK(!ParseBuilderScene(
      "{\"name\":\"x\",\"half_extent\":10,\"round_limit\":0,\"boxes\":[[0,0,0,1,1]],\"figures\":[]}",
      &untouched, &error));
  CHECK(untouched.name == "keep");
  CHECK(ParseBuilderScene(
      "{\"name\":\"x\",\"half_extent\":10,\"round_limit\":0,\"boxes\":[],\"figures\":[]}",
      &untouched, &error));
}

// Blue sniper (on the tall crate) shoots red 2 across the yard while red 3
// walks and the others pass; then a couple of planning frames and round 2.
void PlayBuilderSession(GameLogic& game) {
  game.ClickUnit(3, Team::Red);
  game.ChooseMove();
  game.ClickGround(glm::vec3(4.0f, 0.0f, 3.0f), Team::Red);
  game.FinishMovePlan();
  game.ClickUnit(0, Team::Blue);
  game.ChooseShoot();
  game.SetPlannedShotCount(3, Team::Blue);
  game.PlaceAimPoint(glm::vec3(8.0f, 1.2f, 0.1f), Team::Blue);
  game.ConfirmAim(Team::Blue);
  game.ClickUnit(1, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(2, Team::Red);
  game.ChoosePass();
  CHECK(game.CanCommitRound());
  game.CommitRound();
  const float dts[] = {0.0167f, 0.0171f, 0.0331f, 0.0162f, 0.0210f};
  int i = 0;
  while (game.Mode() == InputMode::Executing && i < 2000) {
    const float dt = dts[i % 5];
    game.Update(dt);
    game.UpdateSightingMemory(dt);
    ++i;
  }
  for (int k = 0; k < 20; ++k) game.Update(0.0167f);
}

std::string RecordBuilderSession(const BuilderScene& bs, uint32_t seed, uint64_t* finalHash) {
  GameLogic game;
  InitBuilderGame(&game, bs, seed);
  g_clock = 0;
  ReproRecorder recorder(&FakeNow);
  recorder.Start(game, BuilderSceneLogId(bs), seed, "2026-10-27T00:00:00Z",
                 SerializeBuilderScene(bs));
  game.SetInputRecorder(&recorder);
  PlayBuilderSession(game);
  game.SetInputRecorder(nullptr);
  *finalHash = game.StateHash();
  return recorder.Text();
}

void TestRecordReplayRoundTrip() {
  const BuilderScene bs = MakeTestScene();
  uint64_t liveFinal = 0;
  const std::string text = RecordBuilderSession(bs, 77u, &liveFinal);

  CHECK(text.find("\"builder_scene\":{") != std::string::npos);
  CHECK(text.find("\"version\":1") != std::string::npos);
  CHECK(text.find("\"scene\":\"builder:Test \\\"yard\\\"\"") != std::string::npos);
  CHECK(text.find("\"op\":\"commit\"") != std::string::npos);

  ReproLog parsed;
  std::string error;
  CHECK(ParseReproLog(text, &parsed, &error));
  CHECK(parsed.header.builderScene == SerializeBuilderScene(bs));
  CHECK(parsed.header.seed == 77u);
  CHECK(SerializeReproLog(parsed) == text);

  // The same session recorded twice is byte-identical.
  uint64_t again = 0;
  CHECK(RecordBuilderSession(bs, 77u, &again) == text);
  CHECK(again == liveFinal);

  // Builder replay path.
  GameLogic replay;
  const ReplayReport report = ReplayBuilderLog(text, &replay);
  if (!report.error.empty()) std::fprintf(stderr, "replay error: %s\n", report.error.c_str());
  CHECK(report.error.empty());
  CHECK(report.ok);
  CHECK(report.mismatches.empty());
  CHECK(report.finalHash == liveFinal);
  CHECK(report.inputsApplied > 8);
  CHECK(report.updatesApplied > 20);

  // Generic path (tactics_repro_replay).
  GameLogic generic;
  ReproLog genericLog;
  const ReplayReport genericReport = ReplayAnyLog(text, &generic, &genericLog);
  CHECK(genericReport.error.empty());
  CHECK(genericReport.ok);
  CHECK(genericReport.finalHash == liveFinal);
  CHECK(genericLog.header.builderScene == parsed.header.builderScene);
  CHECK(SerializeSnapshot(generic.ExportState()) == SerializeSnapshot(replay.ExportState()));

  // The demo-only path does not know builder logs, and says so.
  GameLogic demoOnly;
  CHECK(!ReplayDemoLog(text, &demoOnly).error.empty());

  // A different seed or a tampered scene diverges.
  std::string tampered = text;
  const size_t pos = tampered.find("\"seed\":77");
  CHECK(pos != std::string::npos);
  if (pos != std::string::npos) tampered.replace(pos, 9, "\"seed\":78");
  GameLogic other;
  CHECK(!ReplayBuilderLog(tampered, &other).ok);

  // A builder log header with a broken scene is a replay error.
  std::string broken = text;
  const size_t sceneAt = broken.find("\"half_extent\"");
  CHECK(sceneAt != std::string::npos);
  if (sceneAt != std::string::npos) broken.replace(sceneAt, 13, "\"half_extend\"");
  GameLogic brokenGame;
  CHECK(!ReplayBuilderLog(broken, &brokenGame).error.empty());
  // And ReplayBuilderLog refuses a demo log.
  CHECK(!ReplayBuilderLog("{\"type\":\"header\",\"version\":1,\"scene\":\"odd-ragdoll-repro\","
                          "\"seed\":1,\"initial_hash\":\"0\"}\n",
                          &brokenGame)
             .error.empty());
}

void TestOldFormatLogStillReplays() {
  // A demo log recorded exactly as before this field existed.
  const DemoScene* demo = FindDemoScene("odd-ragdoll-repro");
  CHECK(demo != nullptr);
  if (!demo) return;
  GameLogic game;
  InitDemoGame(*demo, &game);
  g_clock = 0;
  ReproRecorder recorder(&FakeNow);
  recorder.Start(game, demo->id, demo->seed, "2026-10-27T00:00:00Z");
  game.SetInputRecorder(&recorder);
  game.ClickUnit(0, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(1, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(2, Team::Red);
  game.ChoosePass();
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  game.CommitRound();
  for (int k = 0; k < 50; ++k) game.Update(0.0167f);
  game.SetInputRecorder(nullptr);
  const std::string text = recorder.Text();
  CHECK(text.find("builder_scene") == std::string::npos);

  ReproLog parsed;
  std::string error;
  CHECK(ParseReproLog(text, &parsed, &error));
  CHECK(parsed.header.builderScene.empty());
  CHECK(SerializeReproLog(parsed) == text);

  GameLogic viaDemo;
  const ReplayReport demoReport = ReplayDemoLog(text, &viaDemo);
  CHECK(demoReport.error.empty() && demoReport.ok);
  GameLogic viaAny;
  const ReplayReport anyReport = ReplayAnyLog(text, &viaAny);
  CHECK(anyReport.error.empty() && anyReport.ok);
  CHECK(anyReport.finalHash == demoReport.finalHash);

  // A non-object builder_scene is rejected by the parser.
  CHECK(!ParseReproLog("{\"type\":\"header\",\"version\":1,\"scene\":\"x\",\"seed\":1,"
                       "\"builder_scene\":\"nope\",\"initial_hash\":\"0\"}\n",
                       &parsed, &error));
}

// Stacked cubes (y > 0), explicit figure yaw, a cube taller than any old cap.
void TestStackingAndYaw() {
  BuilderScene bs;
  bs.name = "Tower";
  BuilderBox base;
  base.cx = 0.0f;
  base.cz = 0.0f;
  base.hx = base.hz = 2.0f;
  base.height = 1.2f;
  BuilderBox upper = base;
  upper.hx = upper.hz = 1.0f;
  upper.height = 9.0f;  // Past the old 6.0 editor limit.
  upper.y = 1.2f;
  bs.boxes = {base, upper};
  BuilderFigure facing{Team::Blue, WeaponType::AssaultRifle, 0.0f, 0.0f};
  facing.yaw = 1.25f;
  facing.hasYaw = true;
  bs.figures = {facing, {Team::Red, WeaponType::AssaultRifle, 9.0f, 0.0f}};

  CHECK(SurfaceHeightAt(bs, 0.0f, 0.0f) == 10.2f);
  CHECK(SurfaceHeightAt(bs, 1.5f, 0.0f) == 1.2f);
  const Scene scene = BuildScene(bs);
  CHECK(scene.obstacles[1].bounds.min.y == 1.2f);
  CHECK(scene.obstacles[1].bounds.max.y == 10.2f);
  CHECK(scene.units[0].facingYaw == 1.25f);
  CHECK(scene.units[0].position.y == 10.2f);
  CHECK(std::fabs(scene.units[1].facingYaw - 3.14159265f) < 1e-5f);  // Default: face enemy.

  const std::string text = SerializeBuilderScene(bs);
  BuilderScene back;
  std::string error;
  CHECK(ParseBuilderScene(text, &back, &error));
  CHECK(back.boxes.size() == 2 && back.boxes[1].y == 1.2f && back.boxes[0].y == 0.0f);
  CHECK(back.figures[0].hasYaw && back.figures[0].yaw == 1.25f && !back.figures[1].hasYaw);
  CHECK(SerializeBuilderScene(back) == text);
  // Ground cubes keep the original 5-number form (old logs stay byte-identical).
  CHECK(text.find("[0,0,2,2,1.20000005]") != std::string::npos);

  // A figure can path up the stack: ground -> lower top -> upper top.
  NavMesh nav;
  nav.Build(scene.obstacles, 12.0f, constants::kAgentRadius);
  std::vector<glm::vec3> path;
  CHECK(nav.FindPath(glm::vec3(8.0f, 0.0f, 0.0f), glm::vec3(0.0f, 10.2f, 0.0f), &path));
}

}  // namespace

// Issue #278: a shooter has one shot in flight at a time. A sniper that
// spots several enemies while moving must not fire at two of them within
// one bolt interval.
void RunSniperCase(float yaw, bool hasYaw) {
  BuilderScene bs;
  bs.name = "Rooftop standoff";
  bs.mapHalfExtent = 20.0f;
  bs.boxes.push_back({-6.67f, 3.82f, 1.0f, 8.0f, 6.0f});
  bs.boxes.push_back({2.74f, -3.50f, 1.0f, 4.0f, 6.0f});
  bs.figures.push_back({Team::Blue, WeaponType::SniperRifle, -9.12f, 10.28f, yaw, hasYaw});
  bs.figures.push_back({Team::Red, WeaponType::DesertEagle, 2.99f, -2.07f});
  bs.figures.push_back({Team::Red, WeaponType::AssaultRifle, 6.61f, -3.61f});
  bs.figures.push_back({Team::Red, WeaponType::AssaultRifle, 3.08f, -5.51f});
  GameLogic game;
  InitBuilderGame(&game, bs, 3079104832u);
  SquadPlaybook pb;
  pb.At(false, false) = ReactionAction::Shoot;
  pb.At(false, true) = ReactionAction::Shoot;
  pb.At(true, false) = ReactionAction::ShootContinue;
  pb.At(true, true) = ReactionAction::ShootContinue;
  game.SetPlaybook(Team::Blue, pb);
  game.ClickUnit(0, Team::Blue);
  game.ChooseMove();
  game.ClickGround(glm::vec3(-8.65f, 0.0f, -6.48f), Team::Blue);
  game.SetPlannedMoveFacing(0, yaw, Team::Blue);
  game.FinishMovePlan();
  for (int id : {1, 2, 3}) {
    game.ClickUnit(id, Team::Red);
    game.ChoosePass();
  }
  game.CommitRound();
  int steps = 0;
  while (game.Mode() == InputMode::Executing && steps++ < 10000) game.Update(0.016f);
  std::vector<float> times;
  for (const Tracer& t : game.Tracers()) {
    if (t.team == Team::Blue) times.push_back(t.at);
  }
  std::sort(times.begin(), times.end());
  for (size_t i = 1; i < times.size(); ++i) {
    CHECK(times[i] - times[i - 1] >= 1.0f - 1e-3f);
  }
}

void TestSniperNeverFiresTwiceWithinInterval() {
  RunSniperCase(0.0f, false);
  for (int i = 0; i < 16; ++i) RunSniperCase(i * 0.3926991f, true);
}

int main() {
  TestStackingAndYaw();
  TestBuildScene();
  TestSurfaceHeightAt();
  TestFigureStaysOnBoxAfterReset();
  TestJsonRoundTrip();
  TestRecordReplayRoundTrip();
  TestOldFormatLogStillReplays();
  TestSniperNeverFiresTwiceWithinInterval();
  if (g_failures == 0) {
    std::printf("All builder scene tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
