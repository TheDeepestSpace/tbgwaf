// Demo-scene repro logs (issue #240): record a session on the first demo
// scene through the same GameLogic input API the app uses, serialize the
// log, parse it back and replay it into a fresh game -- every recorded state
// hash must match. Also pins the on-disk format's round trip and that a
// tampered log is detected. Headless (no SDL/GL/ImGui).

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "game/DemoScenes.h"
#include "game/GameLogic.h"
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

// Fake wall clock: 16 ms per query.
uint64_t g_clock = 0;
uint64_t FakeNow() { return g_clock += 16; }

// Plays the "odd ragdoll" scene the way a player would: red 2 walks north
// across blue 1's line of fire, blue 1 fires a full sniper burst along it,
// everyone else passes; two rounds with varying frame times, so the log
// exercises moves, bursts, knockdowns, round ends and update batching.
void PlayOddRagdollSession(GameLogic& game) {
  game.ClickUnit(2, Team::Red);
  game.ChooseMove();
  game.ClickGround(glm::vec3(5.0f, 0.0f, 6.0f), Team::Red);
  game.FinishMovePlan();
  game.SetPlannedMoveFacing(2, 1.0f, Team::Red);
  game.ClickUnit(1, Team::Blue);
  game.ChooseShoot();
  game.SetPlannedShotCount(5, Team::Blue);
  game.PlaceAimPoint(glm::vec3(5.0f, 1.2f, 0.0f), Team::Blue);
  game.ConfirmAim(Team::Blue);
  game.ClickUnit(0, Team::Blue);
  game.ChoosePass();
  game.ClickUnit(3, Team::Red);
  game.ChoosePass();
  CHECK(game.CanCommitRound());
  game.CommitRound();
  // Irregular frame times, like a real frame loop.
  const float dts[] = {0.0167f, 0.0171f, 0.0331f, 0.0162f, 0.0210f, 0.0166f, 0.0499f};
  int i = 0;
  while (game.Mode() == InputMode::Executing && i < 2000) {
    const float dt = dts[i % 7];
    game.Update(dt);
    game.UpdateSightingMemory(dt);
    ++i;
  }
  CHECK(game.Mode() != InputMode::Executing);
  // A few planning frames (idle clocks tick), then a second round.
  for (int k = 0; k < 30; ++k) game.Update(0.0167f);
  if (game.Mode() == InputMode::AwaitingSelection) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive) continue;
      game.ClickUnit(unit.id, unit.team);
      game.ChoosePass();
    }
    game.CommitRound();
    for (int k = 0; k < 400 && (game.Mode() == InputMode::Executing || game.HasActiveKnockdown());
         ++k) {
      game.Update(0.0167f);
    }
  }
}

void TestRecordReplayEquality() {
  const DemoScene* demo = FindDemoScene("odd-ragdoll-repro");
  CHECK(demo != nullptr);
  if (!demo) return;

  GameLogic game;
  InitDemoGame(*demo, &game);
  const uint64_t initialHash = game.StateHash();
  ReproRecorder recorder(&FakeNow);
  recorder.Start(game, demo->id, demo->seed, "2026-10-27T00:00:00Z");
  game.SetInputRecorder(&recorder);
  PlayOddRagdollSession(game);
  game.SetInputRecorder(nullptr);
  const uint64_t liveFinalHash = game.StateHash();

  const std::string text = recorder.Text();
  CHECK(!text.empty());
  CHECK(text.rfind("{\"type\":\"header\"", 0) == 0);
  CHECK(text.find("\"scene\":\"odd-ragdoll-repro\"") != std::string::npos);
  CHECK(text.find("\"op\":\"commit\"") != std::string::npos);
  CHECK(text.find("\"type\":\"updates\"") != std::string::npos);

  // The session knocked someone down (the sim log says so) -- the scene is
  // only a useful repro if the shot actually lands with its pinned seed.
  CHECK(text.find("knockdown unit=") != std::string::npos);
  bool anyDead = false;
  for (const Unit& unit : game.GetScene().units) anyDead |= !unit.alive;
  CHECK(anyDead);

  // Parse -> serialize round trip is the identity.
  ReproLog parsed;
  std::string error;
  CHECK(ParseReproLog(text, &parsed, &error));
  if (!error.empty()) std::fprintf(stderr, "parse error: %s\n", error.c_str());
  CHECK(parsed.header.initialHash == initialHash);
  CHECK(parsed.header.seed == demo->seed);
  CHECK(parsed.header.version == kReproLogVersion);
  CHECK(parsed.entries.size() == recorder.Log().entries.size());
  CHECK(SerializeReproLog(parsed) == text);

  // Replay into a fresh game: every checkpoint hash and the final state match.
  GameLogic replay;
  const ReplayReport report = ReplayDemoLog(text, &replay);
  CHECK(report.error.empty());
  if (!report.error.empty()) std::fprintf(stderr, "replay error: %s\n", report.error.c_str());
  CHECK(report.ok);
  CHECK(report.mismatches.empty());
  for (const auto& m : report.mismatches) {
    std::fprintf(stderr, "mismatch at seq %d: %s vs %s\n", m.seq, HashToHex(m.expected).c_str(),
                 HashToHex(m.actual).c_str());
  }
  CHECK(report.finalHash == liveFinalHash);
  CHECK(report.inputsApplied > 10);
  CHECK(report.updatesApplied > 100);
  CHECK(SerializeSnapshot(replay.ExportState()) == SerializeSnapshot(game.ExportState()));

  // Recording the same session again yields the very same log (determinism
  // of the recording itself, not just of the replay).
  GameLogic again;
  InitDemoGame(*demo, &again);
  g_clock = 0;
  ReproRecorder recorder2(&FakeNow);
  recorder2.Start(again, demo->id, demo->seed, "2026-10-27T00:00:00Z");
  again.SetInputRecorder(&recorder2);
  PlayOddRagdollSession(again);
  CHECK(recorder2.Text() == text);
}

void TestTamperedLogIsDetected() {
  const DemoScene* demo = FindDemoScene("odd-ragdoll-repro");
  if (!demo) return;
  GameLogic game;
  InitDemoGame(*demo, &game);
  ReproRecorder recorder(&FakeNow);
  recorder.Start(game, demo->id, demo->seed);
  game.SetInputRecorder(&recorder);
  PlayOddRagdollSession(game);
  game.SetInputRecorder(nullptr);
  ReproLog log = recorder.Log();

  // Nudge the destination of the recorded move: the replay must diverge from
  // the first checkpoint after it.
  int tampered = -1;
  for (ReproEntry& entry : log.entries) {
    if (entry.kind == ReproEntry::Kind::Input && entry.input.op == InputEvent::Op::ClickGround) {
      entry.input.point.z -= 1.0f;
      tampered = entry.seq;
      break;
    }
  }
  CHECK(tampered > 0);
  GameLogic replay;
  const ReplayReport report = ReplayDemoLog(SerializeReproLog(log), &replay);
  CHECK(report.error.empty());
  CHECK(!report.ok);
  CHECK(!report.mismatches.empty());
  if (!report.mismatches.empty()) CHECK(report.mismatches.front().seq == tampered);
}

void TestParserRejectsMalformedLogs() {
  ReproLog log;
  std::string error;
  CHECK(!ParseReproLog("", &log, &error));
  CHECK(!ParseReproLog("{\"type\":\"input\",\"seq\":1}\n", &log, &error));  // No header.
  CHECK(!ParseReproLog("{\"type\":\"header\",\"version\":99,\"scene\":\"x\",\"seed\":1,"
                       "\"initial_hash\":\"0\"}\n",
                       &log, &error));
  CHECK(error.find("version") != std::string::npos);
  CHECK(!ParseReproLog("{\"type\":\"header\",\"version\":1,\"scene\":\"x\",\"seed\":1,"
                       "\"initial_hash\":\"0\"}\n{not json}\n",
                       &log, &error));
  // Unknown scene is a replay error, not a parse error.
  GameLogic game;
  const ReplayReport report = ReplayDemoLog(
      "{\"type\":\"header\",\"version\":1,\"scene\":\"no-such-scene\",\"seed\":1,"
      "\"initial_hash\":\"0\"}\n",
      &game);
  CHECK(!report.error.empty());
  CHECK(!report.ok);

  // Strings with awkward characters survive the round trip.
  ReproLog withLog;
  withLog.header.scene = "odd-ragdoll-repro";
  withLog.header.created = "quote\" backslash\\ newline\n tab\t";
  ReproEntry entry;
  entry.kind = ReproEntry::Kind::Log;
  entry.seq = 1;
  entry.message = "msg with \"quotes\" and \\ and \x01 control";
  withLog.entries.push_back(entry);
  ReproLog back;
  CHECK(ParseReproLog(SerializeReproLog(withLog), &back, &error));
  CHECK(back.header.created == withLog.header.created);
  CHECK(back.entries.size() == 1);
  if (back.entries.size() == 1) CHECK(back.entries[0].message == entry.message);
}

void TestFloatsRoundTripExactly() {
  // Every float a log carries (dt, click points, yaw) must come back
  // bit-identical, or the replay would drift.
  ReproLog log;
  log.header.scene = "odd-ragdoll-repro";
  const float values[] = {0.0166666675f, 1.0f / 3.0f, -12.3456789f, 3.14159274f, 1e-7f, 0.1f};
  ReproEntry updates;
  updates.kind = ReproEntry::Kind::Updates;
  updates.seq = 1;
  for (float v : values) updates.dts.push_back(v);
  log.entries.push_back(updates);
  ReproEntry click;
  click.kind = ReproEntry::Kind::Input;
  click.seq = 2;
  click.input.op = InputEvent::Op::ClickGround;
  click.input.point = glm::vec3(values[1], values[2], values[4]);
  click.input.team = Team::Red;
  log.entries.push_back(click);
  ReproEntry facing;
  facing.kind = ReproEntry::Kind::Input;
  facing.seq = 3;
  facing.input.op = InputEvent::Op::SetMoveFacing;
  facing.input.unit = 2;
  facing.input.value = values[3];
  log.entries.push_back(facing);

  ReproLog back;
  std::string error;
  CHECK(ParseReproLog(SerializeReproLog(log), &back, &error));
  CHECK(back.entries.size() == 3);
  if (back.entries.size() != 3) return;
  CHECK(back.entries[0].dts.size() == 6);
  for (size_t i = 0; i < back.entries[0].dts.size(); ++i) {
    CHECK(back.entries[0].dts[i] == values[i]);
  }
  CHECK(back.entries[1].input.point == click.input.point);
  CHECK(back.entries[1].input.team == Team::Red);
  CHECK(back.entries[2].input.value == values[3]);
  CHECK(back.entries[2].input.unit == 2);
}

void TestRecorderBatchesUpdatesAndSplitsOnModeChange() {
  const DemoScene* demo = FindDemoScene("odd-ragdoll-repro");
  if (!demo) return;
  GameLogic game;
  InitDemoGame(*demo, &game);
  ReproRecorder recorder(&FakeNow);
  recorder.Start(game, demo->id, demo->seed);
  game.SetInputRecorder(&recorder);
  for (int i = 0; i < 50; ++i) game.Update(0.01f);
  game.ClickUnit(1, Team::Blue);
  const ReproLog& log = recorder.Log();
  // 50 planning frames collapse into one batch, followed by the click.
  CHECK(log.entries.size() == 2);
  if (log.entries.size() == 2) {
    CHECK(log.entries[0].kind == ReproEntry::Kind::Updates);
    CHECK(log.entries[0].dts.size() == 50);
    CHECK(log.entries[1].kind == ReproEntry::Kind::Input);
    CHECK(log.entries[1].input.op == InputEvent::Op::ClickUnit);
    CHECK(log.entries[1].seq == 2);
    CHECK(log.entries[1].timeMs > log.entries[0].timeMs);
  }
  // Nested public calls (ClickAimRay -> PlaceAimPoint) record once.
  game.ChooseShoot();
  const size_t before = recorder.Log().entries.size();
  game.ClickAimRay(glm::vec3(0.0f, 20.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), Team::Blue);
  CHECK(recorder.Log().entries.size() == before + 1);
  CHECK(recorder.Log().entries.back().input.op == InputEvent::Op::ClickAimRay);

  // A copy of the game does not carry the recorder.
  GameLogic copy = game;
  CHECK(copy.GetInputRecorder() == nullptr);
  CHECK(game.GetInputRecorder() == &recorder);
}

}  // namespace

int main() {
  TestRecordReplayEquality();
  TestTamperedLogIsDetected();
  TestParserRejectsMalformedLogs();
  TestFloatsRoundTripExactly();
  TestRecorderBatchesUpdatesAndSplitsOnModeChange();
  if (g_failures == 0) {
    std::printf("All repro log tests passed.\n");
    return 0;
  }
  std::fprintf(stderr, "%d check(s) failed.\n", g_failures);
  return 1;
}
