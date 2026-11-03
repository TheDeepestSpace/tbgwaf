#pragma once

// Demo-scene repro logs (issue #240): a self-describing JSON Lines record of
// one session on a demo scene -- which scene, the RNG seed, the build, and
// every input that reached GameLogic (clicks, menu choices, commits, and
// each frame's Update(dt)) with timestamps, round numbers and a hash of the
// dynamic state after each one -- plus the simulation's own log lines
// (shots, knockdowns, round ends). The format is documented in
// docs/repro-log.md. ReplayReproLog feeds a log back through the same
// GameLogic calls and checks every recorded hash, so a session can be
// reproduced headlessly (tactics_repro_replay, repro_log_tests) or loaded
// into the app (TBGWAF_REPRO_LOG) and scrubbed on the turn timeline.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "game/GameLogic.h"

namespace tactics {

constexpr int kReproLogVersion = 1;

// Build identifier baked in at configure time (git short hash, or
// "unknown" outside a checkout).
const char* ReproBuildId();

// Header of a log: everything needed to rebuild the starting state.
struct ReproHeader {
  int version = kReproLogVersion;
  std::string scene;       // DemoScene id (see game/DemoScenes.h).
  uint32_t seed = 0;       // Shot-RNG seed applied before the first input.
  std::string build;       // ReproBuildId() of the recording build.
  std::string created;     // Free-form timestamp (informational only).
  uint64_t initialHash = 0;  // GameLogic::StateHash() before the first input.
  // Optional (Builder scenes): the SerializeBuilderScene() JSON of the
  // player-built scene this log starts from, embedded in the header as the
  // "builder_scene" object. Empty for demo-scene logs (and for every log
  // written before the field existed); `scene` is then a registry id.
  std::string builderScene;
};

// One replayable line: an input (with the post-input hash), a batch of
// per-frame updates, or a sim log line (replay skips those).
struct ReproEntry {
  enum class Kind { Input, Updates, Log };
  Kind kind = Kind::Input;
  int seq = 0;
  int round = 0;
  uint64_t timeMs = 0;       // Wall-clock ms since the recording started.
  InputEvent input;          // Kind::Input.
  std::vector<float> dts;    // Kind::Updates: consecutive Update(dt) calls.
  uint64_t hash = 0;         // State hash after the input / last update.
  std::string message;       // Kind::Log.
};

struct ReproLog {
  ReproHeader header;
  std::vector<ReproEntry> entries;
};

// JSON Lines text of a log (see docs/repro-log.md).
std::string SerializeReproLog(const ReproLog& log);
// Parses a log; returns false (and sets *error) on malformed input.
bool ParseReproLog(const std::string& text, ReproLog* out, std::string* error);

// Wire name of an input op ("click_unit", ...) and back; false if unknown.
const char* InputOpName(InputEvent::Op op);
bool ParseInputOp(const std::string& name, InputEvent::Op* out);

// Records a GameLogic session (attach with GameLogic::SetInputRecorder).
// Consecutive Update(dt) calls are batched into one Updates entry (flushed
// by the next other input, a mode change, or kMaxBatchedUpdates frames) so
// a long planning phase stays compact.
class ReproRecorder : public InputRecorder {
 public:
  static constexpr size_t kMaxBatchedUpdates = 600;

  // `nowMs` supplies the timestamps (wall clock in the app; a fake in tests).
  explicit ReproRecorder(std::function<uint64_t()> nowMs = {});

  // Starts a fresh log from `game`'s current state (which must already be
  // the demo scene with its seed applied). For a Builder scene pass its
  // SerializeBuilderScene() JSON as `builderScene` (and a descriptive
  // `sceneId`, see BuilderSceneLogId): the log then embeds the scene and
  // replays without any registry entry.
  void Start(const GameLogic& game, const std::string& sceneId, uint32_t seed,
             const std::string& created = "", const std::string& builderScene = "");
  bool Started() const { return started_; }

  void OnInput(const GameLogic& game, const InputEvent& event) override;
  void OnLog(const GameLogic& game, const std::string& message) override;

  // The log so far (pending updates are flushed into it first).
  const ReproLog& Log();
  std::string Text();  // SerializeReproLog(Log()).

 private:
  void FlushUpdates();

  std::function<uint64_t()> nowMs_;
  bool started_ = false;
  ReproLog log_;
  int nextSeq_ = 0;
  uint64_t startMs_ = 0;
  // Pending Update batch.
  std::vector<float> pendingDts_;
  uint64_t pendingHash_ = 0;
  int pendingRound_ = 0;
  uint64_t pendingTimeMs_ = 0;
  InputMode pendingMode_ = InputMode::AwaitingSelection;
  bool pendingValid_ = false;
};

// Outcome of a replay. `mismatches` lists every entry whose post-state hash
// differed from the recorded one (first one first); an empty list with
// `ok` means the log reproduced exactly.
struct ReplayReport {
  bool ok = false;
  std::string error;  // Non-empty when the replay could not even run.
  int entriesApplied = 0;
  int inputsApplied = 0;
  int updatesApplied = 0;
  struct Mismatch {
    int seq = 0;
    uint64_t expected = 0;
    uint64_t actual = 0;
  };
  std::vector<Mismatch> mismatches;
  uint64_t finalHash = 0;
};

// Replays `log` into `game`, which must already hold `scene` (the log's demo
// scene) with the seed applied -- ReplayReproLog does not look scenes up;
// see DemoScenes.h's ReplayDemoLog for the registry-aware form. `scene` is
// what a recorded New Match (reset) re-applies. Every replayable entry goes
// through the same GameLogic call the recording made. `afterEntry`, if set,
// runs after each entry (after every single Update of a batch) so a caller
// can observe the state as the app's frame loop would (e.g. feed a
// TurnTimeline). Hash mismatches are reported, not fatal.
ReplayReport ReplayReproLog(const ReproLog& log, const Scene& scene, GameLogic* game,
                            const std::function<void(const ReproEntry&, float dt)>& afterEntry = {});

// Hex (16 digits) form of a hash and back.
std::string HashToHex(uint64_t hash);
bool HexToHash(const std::string& hex, uint64_t* out);

}  // namespace tactics
