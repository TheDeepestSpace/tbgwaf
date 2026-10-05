#include "ai/JevPlanner.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

namespace tactics::ai {
namespace {

constexpr float kPi = 3.14159265358979323846f;

std::string JsonString(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec;
        } else {
          out << c;
        }
    }
  }
  out << '"';
  return out.str();
}

std::string PointJson(const glm::vec3& p) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << "[" << p.x << "," << p.y << "," << p.z
      << "]";
  return out.str();
}

bool IsPlanning(const GameLogic& game) {
  return game.Mode() != InputMode::Executing && game.Mode() != InputMode::GameOver;
}

bool CandidateApplied(const GameLogic& before, Team team, const JevCandidate& candidate,
                      GameLogic* out) {
  *out = before;
  if (!IsPlanning(*out) || out->Mode() != InputMode::AwaitingSelection) return false;
  const Unit* original = out->FindUnit(candidate.actorId);
  if (!original || !original->alive || original->team != team ||
      original->plan.type != PlannedActionType::None) {
    return false;
  }

  out->ClickUnit(candidate.actorId, team);
  if (out->Mode() != InputMode::ActionMenu) return false;
  if (candidate.kind == JevActionKind::Wait) {
    out->ChoosePass();
  } else if (candidate.kind == JevActionKind::Shoot) {
    out->ChooseShoot();
    out->ClickUnit(candidate.targetId, team);
  } else {
    out->ChooseMove();
    out->ClickGround(candidate.destination, team);
    out->FinishMovePlan();
  }

  const Unit* planned = out->FindUnit(candidate.actorId);
  if (!planned || out->Mode() != InputMode::AwaitingSelection) return false;
  if (candidate.kind == JevActionKind::Wait) {
    return planned->plan.type == PlannedActionType::Pass;
  }
  if (candidate.kind == JevActionKind::Shoot) {
    return planned->plan.type == PlannedActionType::Shoot &&
           planned->plan.shootTargetId == candidate.targetId;
  }
  return planned->plan.type == PlannedActionType::Move && !planned->plan.movePath.empty();
}

// Unvalidated candidate specs; ValidateCandidate() checks each one through a
// GameLogic copy (move pathing is the expensive part, so callers can spread
// validation across frames).
std::vector<JevCandidate> GenerateCandidateSpecs(const GameLogic& game, Team team, int actorId) {
  const Unit* actor = game.FindUnit(actorId);
  if (!actor) return {};
  std::vector<JevCandidate> candidates;

  const TeamVisibility visibility = game.ComputeVisibility(team);
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.team == team || !visibility.UnitVisible(unit.id)) continue;
    JevCandidate candidate;
    candidate.id = "shoot_" + std::to_string(unit.id);
    candidate.description = "Shoot visible enemy " + std::to_string(unit.id) + " at " +
                            PointJson(unit.position) + ".";
    candidate.kind = JevActionKind::Shoot;
    candidate.actorId = actorId;
    candidate.targetId = unit.id;
    candidates.push_back(std::move(candidate));
  }

  // Fixed bearings and two radii keep the option set bounded while giving
  // the navmesh enough alternatives around walls and over ramps.
  constexpr std::array<float, 2> kDistanceFractions = {0.8f, 0.45f};
  constexpr int kBearings = 6;
  int moveIndex = 0;
  for (float fraction : kDistanceFractions) {
    for (int bearing = 0; bearing < kBearings; ++bearing) {
      const float angle = 2.0f * kPi * static_cast<float>(bearing) / kBearings;
      const float distance = actor->MoveBudget() * fraction;
      JevCandidate candidate;
      candidate.id = "move_" + std::to_string(moveIndex++);
      candidate.kind = JevActionKind::Move;
      candidate.actorId = actorId;
      candidate.destination = actor->position +
                              glm::vec3(std::cos(angle) * distance, 0.0f,
                                        std::sin(angle) * distance);
      candidates.push_back(std::move(candidate));
    }
  }

  JevCandidate wait;
  wait.id = "wait";
  wait.description = "Wait in place for this round; use only when movement or a visible shot is worse.";
  wait.kind = JevActionKind::Wait;
  wait.actorId = actorId;
  candidates.push_back(std::move(wait));
  return candidates;
}

bool ValidateCandidate(const GameLogic& game, Team team, JevCandidate* candidate) {
  GameLogic trial;
  if (!CandidateApplied(game, team, *candidate, &trial)) return false;
  if (candidate->kind != JevActionKind::Move) return true;
  const Unit* actor = game.FindUnit(candidate->actorId);
  const Unit* planned = trial.FindUnit(candidate->actorId);
  candidate->destination = planned->plan.movePath.back();
  const float forward = team == Team::Blue ? candidate->destination.x - actor->position.x
                                           : actor->position.x - candidate->destination.x;
  std::ostringstream description;
  description << "Move legally to " << PointJson(candidate->destination) << "; "
              << (forward > 0.25f ? "advances toward the opposing deployment edge"
                                  : forward < -0.25f ? "retreats from the opposing edge"
                                                    : "moves laterally")
              << ".";
  candidate->description = description.str();
  return true;
}

std::string TeamName(Team team) { return team == Team::Blue ? "blue" : "red"; }

std::string BuildVisibleState(const GameLogic& game, Team team, int actorId) {
  const TeamVisibility visibility = game.ComputeVisibility(team);
  std::ostringstream out;
  out << "{\"round\":" << game.RoundNumber() << ",\"team\":" << JsonString(TeamName(team))
      << ",\"objective\":"
      << JsonString("Eliminate the opposing squad. Blue generally advances toward +X; Red toward -X. "
                    "Prefer a useful visible shot, otherwise advance while keeping options open.")
      << ",\"actor_id\":" << actorId << ",\"map_half_extent\":" << std::fixed
      << std::setprecision(2) << game.GetScene().mapHalfExtent << ",\"allies\":[";

  bool first = true;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team != team) continue;
    if (!first) out << ',';
    first = false;
    out << "{\"id\":" << unit.id << ",\"alive\":" << (unit.alive ? "true" : "false")
        << ",\"position\":" << PointJson(unit.position) << ",\"facing\":"
        << std::setprecision(3) << unit.facingYaw << ",\"planned\":"
        << (unit.plan.type == PlannedActionType::None ? "false" : "true") << "}";
  }
  out << "],\"visible_enemies\":[";
  first = true;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team == team || !unit.alive || !visibility.UnitVisible(unit.id)) continue;
    if (!first) out << ',';
    first = false;
    out << "{\"id\":" << unit.id << ",\"position\":" << PointJson(unit.position)
        << ",\"facing\":" << std::setprecision(3) << unit.facingYaw << "}";
  }
  out << "]}";
  return out.str();
}

}  // namespace

JevRequestBuilder::JevRequestBuilder(const GameLogic& game, Team team, unsigned requestNonce)
    : game_(game), team_(team) {
  if (!IsPlanning(game) || game.Mode() != InputMode::AwaitingSelection) return;
  const Unit* actor = nullptr;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team == team && unit.alive && unit.plan.type == PlannedActionType::None &&
        (!actor || unit.id < actor->id)) {
      actor = &unit;
    }
  }
  if (!actor) return;

  request_.team = team;
  request_.round = game.RoundNumber();
  request_.actorId = actor->id;
  request_.id = "r" + std::to_string(request_.round) + "-t" +
                std::to_string(static_cast<int>(team)) + "-u" + std::to_string(actor->id) + "-n" +
                std::to_string(requestNonce);
  pending_ = GenerateCandidateSpecs(game, team, actor->id);
  valid_ = true;
}

bool JevRequestBuilder::Step(double budgetMs) {
  if (!valid_) return true;
  const auto start = std::chrono::steady_clock::now();
  while (next_ < pending_.size()) {
    JevCandidate candidate = std::move(pending_[next_++]);
    if (ValidateCandidate(game_, team_, &candidate)) request_.candidates.push_back(std::move(candidate));
    const std::chrono::duration<double, std::milli> elapsed =
        std::chrono::steady_clock::now() - start;
    if (elapsed.count() >= budgetMs) break;
  }
  return next_ >= pending_.size();
}

std::optional<JevRequest> JevRequestBuilder::Finish() {
  if (!valid_ || next_ < pending_.size() || request_.candidates.empty()) return std::nullopt;
  std::ostringstream out;
  out << "{\"requestId\":" << JsonString(request_.id) << ",\"state\":"
      << BuildVisibleState(game_, team_, request_.actorId) << ",\"candidates\":[";
  for (size_t i = 0; i < request_.candidates.size(); ++i) {
    if (i) out << ',';
    out << "{\"id\":" << JsonString(request_.candidates[i].id) << ",\"description\":"
        << JsonString(request_.candidates[i].description) << "}";
  }
  out << "]}";
  request_.json = out.str();
  valid_ = false;
  return std::move(request_);
}

std::optional<JevRequest> BuildJevRequest(const GameLogic& game, Team team,
                                          unsigned requestNonce) {
  JevRequestBuilder builder(game, team, requestNonce);
  while (!builder.Step(1e9)) {
  }
  return builder.Finish();
}

bool ApplyJevChoice(GameLogic* game, const JevRequest& request, const std::string& choice) {
  if (!game || game->RoundNumber() != request.round || !IsPlanning(*game) ||
      game->Mode() != InputMode::AwaitingSelection) {
    return false;
  }
  const Unit* actor = game->FindUnit(request.actorId);
  if (!actor || !actor->alive || actor->team != request.team ||
      actor->plan.type != PlannedActionType::None) {
    return false;
  }
  const auto candidate = std::find_if(request.candidates.begin(), request.candidates.end(),
                                      [&](const JevCandidate& c) { return c.id == choice; });
  if (candidate == request.candidates.end()) return false;

  GameLogic validated;
  if (!CandidateApplied(*game, request.team, *candidate, &validated)) return false;
  *game = std::move(validated);
  return true;
}

std::string DeterministicFallbackChoice(const JevRequest& request) {
  for (const JevCandidate& candidate : request.candidates) {
    if (candidate.kind == JevActionKind::Shoot) return candidate.id;
  }
  const float sign = request.team == Team::Blue ? 1.0f : -1.0f;
  const JevCandidate* best = nullptr;
  for (const JevCandidate& candidate : request.candidates) {
    if (candidate.kind != JevActionKind::Move) continue;
    if (!best || sign * candidate.destination.x > sign * best->destination.x) best = &candidate;
  }
  if (best) return best->id;
  for (const JevCandidate& candidate : request.candidates) {
    if (candidate.kind == JevActionKind::Wait) return candidate.id;
  }
  return {};
}

}  // namespace tactics::ai
