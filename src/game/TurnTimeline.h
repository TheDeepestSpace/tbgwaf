#pragma once

// Turn timeline (issue #143): match history recording + view-only replay.
//
// TurnTimeline watches the *local* GameLogic once per frame and records
// GameSnapshot frames: one at game start, a fixed-rate stream while a round
// executes (or a knockdown is still falling), and one at each round end.
// Round boundaries become labelled "ticks" (Start, T1, T2, ...) the HUD
// timeline slider snaps to. Because it only observes the local instance it
// works identically on the Blue simulator and on a Red follower (whose state
// arrives via ImportState) -- each web client records its own local history,
// and the native split-screen window keeps a single history for its single
// shared GameLogic.
//
// TimelinePlayback replays that history into its own private GameLogic
// (built from a copy of the live scene) by importing recorded frames, so
// scrubbing/replaying never mutates the live game and never publishes
// anything onto the Blue-authority/Red-mirror snapshot bus. Replay is
// strictly view-only: scrubbing back and pressing Play re-shows recorded
// frames (frame-accurate, including shot outcomes -- nothing is
// re-simulated), it never branches or truncates the live match. A "Resume
// from here" branching mode is explicitly out of scope (see issue #143).
//
// Fog of war during replay: the caller renders the replay GameLogic through
// the same ComputeVisibility/RenderPane path as the live game, so each pane
// sees the replayed moment with its own team's fog recomputed from the
// replayed positions. Sighting-memory ghost trails are rebuilt as the replay
// plays forward (ImportState drops memory when jumping backwards), so they
// approximate -- rather than exactly reproduce -- what a team remembered.

#include <memory>
#include <string>
#include <vector>

#include "game/GameLogic.h"

namespace tactics {

class TurnTimeline {
 public:
  // Game-time spacing of frames captured while a round executes. 30 Hz keeps
  // playback smooth (the renderer shows imported frames as-is, like the Red
  // mirror pane) at a few hundred small snapshots per round.
  static constexpr float kCaptureInterval = 1.0f / 30.0f;

  // A slider stop: the state at game start or at a round's end.
  struct Tick {
    std::string label;  // "Start", "T1", "T2", ...
    size_t frame = 0;   // Index into Frames().
  };

  // Restarts history from `game`'s current state (the new "Start" tick).
  // Call wherever the game itself is Reset().
  void Reset(const GameLogic& game);

  // Observes the live game once per frame (`dtSeconds` since the last call).
  // Captures executing-round frames, adds a tick when a round completes (or
  // the game ends), and self-resets when it sees a new match begin (round
  // counter went backwards, or a finished match back in play -- how a
  // follower, which gets no explicit Reset call, learns of a new game).
  // Returns true on that self-reset so callers can drop any replay in
  // progress. Auto-initializes from `game` on the very first call.
  bool Observe(const GameLogic& game, float dtSeconds);

  size_t FrameCount() const { return frames_.size(); }
  const GameSnapshot& FrameSnapshot(size_t index) const { return frames_[index].snapshot; }
  float FrameTime(size_t index) const { return frames_[index].time; }
  const std::vector<Tick>& Ticks() const { return ticks_; }
  // Index of the last tick at or before `frame` (the tick a scrubber shows
  // while playback is between ticks). 0 when there is no history yet.
  int TickForFrame(size_t frame) const;

 private:
  struct Frame {
    float time = 0.0f;  // Cumulative executed game time at capture.
    GameSnapshot snapshot;
  };

  void Capture(const GameLogic& game);

  std::vector<Frame> frames_;
  std::vector<Tick> ticks_;
  float time_ = 0.0f;
  int lastRound_ = 1;
  InputMode lastMode_ = InputMode::AwaitingSelection;
};

class TimelinePlayback {
 public:
  // Recorded frames play back against wall-clock time scaled by this factor.
  static constexpr float kReplaySpeed = 1.0f;

  // True while the replay view (not the live game) should be rendered.
  bool Active() const { return active_; }
  bool Playing() const { return active_ && playing_; }
  // The replayed state; only valid while Active().
  const GameLogic& Game() const { return *replay_; }
  size_t FrameIndex() const { return frame_; }
  int CurrentTick(const TurnTimeline& timeline) const {
    return timeline.TickForFrame(frame_);
  }
  bool AtLiveEnd(const TurnTimeline& timeline) const {
    return frame_ + 1 >= timeline.FrameCount();
  }

  // Restores the state at `tickIndex` into the replay game (slider snap).
  // Keeps playing/paused as-is, except that selecting the newest tick while
  // paused returns to the live view (the natural "scrub back to now").
  // Returns false for an out-of-range tick.
  bool SeekTick(const TurnTimeline& timeline, const GameLogic& live, int tickIndex);

  // Starts (or resumes) playback from the current position; from the live
  // view it starts a replay of the whole match from the Start tick.
  void Play(const TurnTimeline& timeline, const GameLogic& live);
  void TogglePlay(const TurnTimeline& timeline, const GameLogic& live);
  void SetPlaying(bool playing) { playing_ = active_ && playing; }

  // Advances playback by dtSeconds * kReplaySpeed, importing successive
  // recorded frames; pauses (stays on the replay view) at the live end.
  void Update(const TurnTimeline& timeline, float dtSeconds);

  // Back to the live view.
  void Deactivate() {
    active_ = false;
    playing_ = false;
  }
  // Deactivates and drops the internal replay game. Required on a new match:
  // a new map's unit ids can coincide with the old one's, so a stale replay
  // scene would not be caught by ImportState's id check.
  void Reset() {
    Deactivate();
    replay_.reset();
  }

 private:
  // Imports frame `index`; (re)builds the replay game from the live scene if
  // needed. Deactivates on an irrecoverable scene/snapshot mismatch.
  void ShowFrame(const TurnTimeline& timeline, const GameLogic& live, size_t index);

  std::unique_ptr<GameLogic> replay_;
  bool active_ = false;
  bool playing_ = false;
  size_t frame_ = 0;
  float time_ = 0.0f;
};

}  // namespace tactics
