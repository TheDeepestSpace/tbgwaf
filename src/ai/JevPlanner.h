#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "game/GameLogic.h"

namespace tactics::ai {

enum class JevActionKind { Move, Shoot, Wait };

// One action already checked through GameLogic's normal click/plan path.
// The browser only gives Jev these bounded options; Jev never invents game
// commands or coordinates.
struct JevCandidate {
  std::string id;
  std::string description;
  JevActionKind kind = JevActionKind::Wait;
  int actorId = -1;
  int targetId = -1;
  glm::vec3 destination{0.0f};
};

struct JevRequest {
  std::string id;
  std::string json;
  Team team = Team::Blue;
  int round = 0;
  int actorId = -1;
  std::vector<JevCandidate> candidates;
};

// Builds one atomic Choice request for the first living, unplanned figure on
// `team`. State contains all friendly figures but only currently-visible
// enemies. Every option has already been accepted by a copy of GameLogic.
std::optional<JevRequest> BuildJevRequest(const GameLogic& game, Team team,
                                          unsigned requestNonce);

// Same as BuildJevRequest, but candidate validation (navmesh pathing, ~100 ms
// each) runs in time-boxed Step() calls so the render loop can keep drawing
// and taking input. The builder works on its own copy of `game`; staleness is
// caught later by ApplyJevChoice. Finish() is nullopt when there is nothing
// to ask (no actor / no legal candidates).
class JevRequestBuilder {
 public:
  JevRequestBuilder(const GameLogic& game, Team team, unsigned requestNonce);
  // Validates candidates for up to ~budgetMs (at least one per call).
  // Returns true once all are validated.
  bool Step(double budgetMs);
  std::optional<JevRequest> Finish();

 private:
  GameLogic game_;
  Team team_;
  bool valid_ = false;
  JevRequest request_;
  std::vector<JevCandidate> pending_;
  size_t next_ = 0;
};

// Rechecks the response against the current round/unit and applies it through
// ClickUnit/Choose*/ClickGround. Unknown, stale, or newly-illegal choices are
// rejected without changing the game.
bool ApplyJevChoice(GameLogic* game, const JevRequest& request, const std::string& choice);

// Explicit non-Jev fallback used only after the user opts in. It prefers a
// visible shot, then progress toward the opposing deployment edge, then wait.
std::string DeterministicFallbackChoice(const JevRequest& request);

}  // namespace tactics::ai
