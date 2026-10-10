#include "game/TurnTimeline.h"

#include <utility>

namespace tactics {

void TurnTimeline::Reset(const GameLogic& game) {
  frames_.clear();
  ticks_.clear();
  time_ = 0.0f;
  lastRound_ = game.RoundNumber();
  lastMode_ = game.Mode();
  Capture(game);
  ticks_.push_back(Tick{"Start", 0});
}

void TurnTimeline::Capture(const GameLogic& game) {
  Frame frame;
  frame.time = time_;
  frame.snapshot = game.ExportState();
  frames_.push_back(std::move(frame));
}

bool TurnTimeline::Observe(const GameLogic& game, float dtSeconds) {
  if (frames_.empty()) {
    Reset(game);
    return false;
  }
  // A new match looks like the round counter jumping backwards, or a
  // finished match back in play (both can reach a follower with no other
  // signal; same detection as GameLogic::ImportState).
  if (game.RoundNumber() < lastRound_ ||
      (lastMode_ == InputMode::GameOver && game.Mode() != InputMode::GameOver)) {
    Reset(game);
    return true;
  }

  // Frames advance only while something animates: the executing round
  // itself, or a knockdown still falling after the round resolved.
  if (game.Mode() == InputMode::Executing || game.HasActiveKnockdown()) {
    time_ += dtSeconds;
    if (time_ - frames_.back().time >= kCaptureInterval) Capture(game);
  }

  const bool roundCompleted = game.RoundNumber() > lastRound_;
  const bool gameEnded = game.Mode() == InputMode::GameOver && lastMode_ != InputMode::GameOver;
  if (roundCompleted || gameEnded) {
    // The round that just finished: FinishRound() bumps the counter on a
    // normal round end but leaves it untouched on game over.
    const int finishedRound = roundCompleted ? game.RoundNumber() - 1 : game.RoundNumber();
    Capture(game);
    ticks_.push_back(Tick{"T" + std::to_string(finishedRound), frames_.size() - 1});
  }

  lastRound_ = game.RoundNumber();
  lastMode_ = game.Mode();
  return false;
}

int TurnTimeline::TickForFrame(size_t frame) const {
  int tick = 0;
  for (size_t i = 0; i < ticks_.size(); ++i) {
    if (ticks_[i].frame <= frame) tick = static_cast<int>(i);
  }
  return tick;
}

size_t TurnTimeline::FrameAtTime(float t) const {
  size_t index = 0;
  while (index + 1 < frames_.size() && frames_[index + 1].time <= t) ++index;
  return index;
}

void TimelinePlayback::ShowFrame(const TurnTimeline& timeline, const GameLogic& live,
                                 size_t index) {
  const GameSnapshot& snapshot = timeline.FrameSnapshot(index);
  if (!replay_ || !replay_->ImportState(snapshot)) {
    replay_ = std::make_unique<GameLogic>(live.GetScene());
    if (!replay_->ImportState(snapshot)) {
      // History doesn't match the live scene (shouldn't happen: both reset
      // together on a new match); fail back to the live view.
      Reset();
      return;
    }
  }
  replay_->ClearAllPlans();
  replay_->RestartGhostPlayback();
  replay_->UpdateSightingMemory(0.0f);
  frame_ = index;
  time_ = timeline.FrameTime(index);
  active_ = true;
}

bool TimelinePlayback::SeekTick(const TurnTimeline& timeline, const GameLogic& live,
                                int tickIndex) {
  const auto& ticks = timeline.Ticks();
  if (tickIndex < 0 || tickIndex >= static_cast<int>(ticks.size())) return false;
  if (!playing_ && tickIndex == static_cast<int>(ticks.size()) - 1) {
    // The newest tick while paused is just "now": return to the live view.
    Deactivate();
    return true;
  }
  ShowFrame(timeline, live, ticks[tickIndex].frame);
  return active_;
}

bool TimelinePlayback::SeekFrame(const TurnTimeline& timeline, const GameLogic& live,
                                 size_t index) {
  if (index >= timeline.FrameCount()) return false;
  ShowFrame(timeline, live, index);
  return active_;
}

void TimelinePlayback::Play(const TurnTimeline& timeline, const GameLogic& live) {
  if (!active_) {
    if (timeline.FrameCount() == 0) return;
    ShowFrame(timeline, live, timeline.Ticks().front().frame);
    if (!active_) return;
  }
  playing_ = true;
}

void TimelinePlayback::TogglePlay(const TurnTimeline& timeline, const GameLogic& live) {
  if (Playing()) {
    playing_ = false;
  } else {
    Play(timeline, live);
  }
}

void TimelinePlayback::Update(const TurnTimeline& timeline, float dtSeconds) {
  if (!Playing()) return;
  time_ += dtSeconds * kReplaySpeed;
  size_t target = frame_;
  while (target + 1 < timeline.FrameCount() && timeline.FrameTime(target + 1) <= time_) {
    ++target;
  }
  if (target != frame_) {
    replay_->ImportState(timeline.FrameSnapshot(target));
    replay_->ClearAllPlans();
    frame_ = target;
  }
  // Rebuild ghost trails as the replay advances (a follower does the same
  // with its imported frames).
  replay_->UpdateSightingMemory(dtSeconds * kReplaySpeed);
  if (frame_ + 1 >= timeline.FrameCount()) {
    // Live end reached: pause there (stay on the replay view; the player
    // returns to live explicitly).
    playing_ = false;
    time_ = timeline.FrameTime(frame_);
  }
}

}  // namespace tactics
