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
constexpr float kGridCellSize = 0.75f;  // World units per ASCII cell.
constexpr int kMinGridCells = 32;
constexpr int kMaxGridCells = 96;
constexpr size_t kMaxListedObstacles = 48;

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

const char* WeaponName(WeaponType type) {
  switch (type) {
    case WeaponType::DesertEagle: return "Desert Eagle";
    case WeaponType::SniperRifle: return "sniper rifle";
    default: return "assault rifle";
  }
}

std::string HitPercent(const GameLogic& game, const Unit& shooter, const Unit& target) {
  return std::to_string(static_cast<int>(game.ShotHitChance(shooter, target) * 100.0f + 0.5f)) +
         "%";
}

const char* ReactionName(ReactionAction action) {
  switch (action) {
    case ReactionAction::Shoot: return "shoot";
    case ReactionAction::Stop: return "stop";
    case ReactionAction::Continue: return "continue";
    case ReactionAction::ShootStop: return "shoot_and_stop";
    case ReactionAction::ShootContinue: return "shoot_while_moving";
    default: return "do_nothing";
  }
}

float XZDistance(const glm::vec3& a, const glm::vec3& b) {
  return std::hypot(a.x - b.x, a.z - b.z);
}

// Latest remembered position of an enemy that is not currently visible.
struct Ghost {
  int id = -1;
  const GameLogic::EnemySighting* sighting = nullptr;
};

std::vector<Ghost> HiddenEnemyGhosts(const GameLogic& game, Team team,
                                     const TeamVisibility& visibility) {
  std::vector<Ghost> ghosts;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team == team || visibility.UnitVisible(unit.id)) continue;
    const auto& samples = game.Sightings(team, unit.id);
    if (samples.empty()) continue;
    ghosts.push_back({unit.id, &samples.back()});
  }
  return ghosts;
}

std::string ExposureNote(const GameLogic& game, Team team, const Unit& actor,
                         const glm::vec3& destination) {
  const TeamVisibility visibility = game.ComputeVisibility(team);
  Unit probe = actor;
  probe.position = destination;
  int seenBy = 0;
  int visibleCount = 0;
  const Scene& scene = game.GetScene();
  for (const Unit& enemy : scene.units) {
    if (enemy.team == team || !enemy.alive || !visibility.UnitVisible(enemy.id)) continue;
    ++visibleCount;
    if (CanUnitSee(enemy, probe, scene.obstacles, scene.walkSurfaces, scene.ground)) ++seenBy;
  }
  if (visibleCount == 0) return "no enemy is currently visible, so exposure there is unknown";
  if (seenBy == 0) return "hidden from all " + std::to_string(visibleCount) + " visible enemies "
                          "if they hold position (cover)";
  return "in line of sight of " + std::to_string(seenBy) + " of " +
         std::to_string(visibleCount) + " visible enemies if they hold position (exposed)";
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
    out->SetPlannedShotCount(candidate.shots, team);
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
           planned->plan.shootTargetId == candidate.targetId &&
           planned->plan.shots == candidate.shots;
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
    candidate.shots = MaxShotsPerAction(actor->weapon, constants::kRoundDuration);
    candidate.description = "Shoot visible enemy " + std::to_string(unit.id) + " at " +
                            PointJson(unit.position) + " with the " + WeaponName(actor->weapon) +
                            ", a burst of " + std::to_string(candidate.shots) +
                            " scattered shots fired over the round while the target stays in "
                            "view; current hit chance per shot " + HitPercent(game, *actor, unit) +
                            ".";
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

  // Tactical moves: duck behind nearby obstacles away from the threat, and
  // close on the last-known position of an enemy that slipped out of view.
  std::vector<glm::vec3> threats;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.alive && unit.team != team && visibility.UnitVisible(unit.id)) {
      threats.push_back(unit.position);
    }
  }
  const std::vector<Ghost> ghosts = HiddenEnemyGhosts(game, team, visibility);
  if (threats.empty()) {
    for (const Ghost& ghost : ghosts) threats.push_back(ghost.sighting->position);
  }
  if (!threats.empty()) {
    glm::vec3 nearestThreat = threats.front();
    for (const glm::vec3& t : threats) {
      if (XZDistance(t, actor->position) < XZDistance(nearestThreat, actor->position)) {
        nearestThreat = t;
      }
    }
    const auto& obstacles = game.GetScene().obstacles;
    std::vector<size_t> order;
    for (size_t i = 0; i < obstacles.size(); ++i) {
      const glm::vec3 c = obstacles[i].bounds.Center();
      if (obstacles[i].bounds.max.y - obstacles[i].bounds.min.y < 1.0f) continue;
      if (XZDistance(c, actor->position) <= actor->MoveBudget()) order.push_back(i);
    }
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return XZDistance(obstacles[a].bounds.Center(), actor->position) <
             XZDistance(obstacles[b].bounds.Center(), actor->position);
    });
    for (size_t k = 0; k < order.size() && k < 3; ++k) {
      const Obstacle& obstacle = obstacles[order[k]];
      const glm::vec3 c = obstacle.bounds.Center();
      glm::vec3 away = c - nearestThreat;
      away.y = 0.0f;
      if (glm::length(away) < 0.01f) continue;
      away = glm::normalize(away);
      const glm::vec3 half = obstacle.bounds.HalfExtents();
      JevCandidate candidate;
      candidate.id = "cover_" + std::to_string(order[k]);
      candidate.kind = JevActionKind::Move;
      candidate.actorId = actorId;
      candidate.destination = c + away * (std::hypot(half.x, half.z) + 1.0f);
      candidate.description = "Take cover on the far side of obstacle " +
                              std::to_string(order[k]) + " from the nearest " +
                              "known enemy.";
      candidates.push_back(std::move(candidate));
    }
  }
  int hunted = 0;
  for (const Ghost& ghost : ghosts) {
    if (hunted >= 2) break;
    glm::vec3 dir = ghost.sighting->position - actor->position;
    dir.y = 0.0f;
    const float dist = glm::length(dir);
    if (dist < 1.0f) continue;
    JevCandidate candidate;
    candidate.id = "hunt_" + std::to_string(ghost.id);
    candidate.kind = JevActionKind::Move;
    candidate.actorId = actorId;
    candidate.destination = actor->position + dir / dist * std::min(dist, actor->MoveBudget() * 0.8f);
    candidate.description = "Move toward ghost of enemy " + std::to_string(ghost.id) +
                            " (last seen " + std::to_string(ghost.sighting->ageRounds) +
                            " round(s) ago at " + PointJson(ghost.sighting->position) + ").";
    candidates.push_back(std::move(candidate));
    ++hunted;
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
  if (!candidate->description.empty()) description << candidate->description << " ";
  description << "Move legally to " << PointJson(candidate->destination) << "; "
              << (forward > 0.25f ? "advances toward the opposing deployment edge"
                                  : forward < -0.25f ? "retreats from the opposing edge"
                                                    : "moves laterally")
              << "; at the destination " << ExposureNote(game, team, *actor, candidate->destination)
              << ".";
  candidate->description = description.str();
  return true;
}

std::string TeamName(Team team) { return team == Team::Blue ? "blue" : "red"; }

int GridCells(const Scene& scene) {
  return std::clamp(static_cast<int>(std::ceil(2.0f * scene.mapHalfExtent / kGridCellSize)),
                    kMinGridCells, kMaxGridCells);
}

// Orientation-agnostic point-in-convex-polygon test on the XZ plane.
bool InsideFootprint(const std::vector<glm::vec2>& poly, float x, float z) {
  bool positive = false;
  bool negative = false;
  for (size_t i = 0; i < poly.size(); ++i) {
    const glm::vec2& a = poly[i];
    const glm::vec2& b = poly[(i + 1) % poly.size()];
    const float cross = (b.x - a.x) * (z - a.y) - (b.y - a.y) * (x - a.x);
    positive |= cross > 0.0f;
    negative |= cross < 0.0f;
  }
  return !(positive && negative);
}

// Top-down overview: row 0 is the most negative Z, column 0 the most negative
// X. Each cell is sampled at its centre against the exact obstacle footprint
// (obstacles smaller than a cell still mark the cell holding their centre).
// '#' obstacle, '+' climbable obstacle, '.' open; allies are their id digit,
// visible enemies lowercase 'e', remembered (ghost) enemies '?'.
std::vector<std::string> BuildGrid(const GameLogic& game, Team team,
                                   const TeamVisibility& visibility,
                                   const std::vector<Ghost>& ghosts, int cells) {
  const Scene& scene = game.GetScene();
  const float half = scene.mapHalfExtent;
  const float cell = 2.0f * half / static_cast<float>(cells);
  std::vector<std::string> rows(cells, std::string(cells, '.'));
  const auto toCell = [&](float v) {
    return std::clamp(static_cast<int>((v + half) / cell), 0, cells - 1);
  };
  for (const Obstacle& obstacle : scene.obstacles) {
    if (obstacle.bounds.max.y - obstacle.bounds.min.y < 1.0f) continue;
    const char mark = obstacle.climbable ? '+' : '#';
    const std::vector<glm::vec2> poly = ObstacleFootprint(obstacle);
    for (int z = toCell(obstacle.bounds.min.z); z <= toCell(obstacle.bounds.max.z); ++z) {
      for (int x = toCell(obstacle.bounds.min.x); x <= toCell(obstacle.bounds.max.x); ++x) {
        if (InsideFootprint(poly, -half + (x + 0.5f) * cell, -half + (z + 0.5f) * cell)) {
          rows[z][x] = mark;
        }
      }
    }
    const glm::vec3 c = obstacle.bounds.Center();
    rows[toCell(c.z)][toCell(c.x)] = mark;
  }
  for (const Ghost& ghost : ghosts) {
    rows[toCell(ghost.sighting->position.z)][toCell(ghost.sighting->position.x)] = '?';
  }
  for (const Unit& unit : scene.units) {
    if (!unit.alive) continue;
    if (unit.team == team) {
      rows[toCell(unit.position.z)][toCell(unit.position.x)] =
          static_cast<char>('0' + unit.id % 10);
    } else if (visibility.UnitVisible(unit.id)) {
      rows[toCell(unit.position.z)][toCell(unit.position.x)] = 'e';
    }
  }
  return rows;
}

std::string BuildVisibleState(const GameLogic& game, Team team,
                              const std::vector<int>& actorIds) {
  const TeamVisibility visibility = game.ComputeVisibility(team);
  const std::vector<Ghost> ghosts = HiddenEnemyGhosts(game, team, visibility);
  const int cells = GridCells(game.GetScene());
  std::ostringstream out;
  out << "{\"round\":" << game.RoundNumber() << ",\"team\":" << JsonString(TeamName(team))
      << ",\"objective\":"
      << JsonString("Eliminate the opposing squad. Blue generally advances toward +X; Red toward -X. "
                    "You plan every listed acting figure at once, one option each, as a single "
                    "coordinated squad plan: all plans execute simultaneously. Cover each other, "
                    "focus fire, stagger exposure instead of exposing everyone, and keep "
                    "allies out of each other's lines of fire. "
                    "Prefer a useful visible shot, otherwise advance while keeping options open. "
                    "Shooting: a Shoot action fires a burst of up to the weapon's magazine "
                    "(capped by shots fitting the 5 s round at its fire interval) at one "
                    "visible enemy. Each bullet flies its own line scattered inside a cone "
                    "(scatter_half_angle_deg). Hit chance per shot falls with distance and with "
                    "how far the target is off the shooter's facing, and is 0 outside the "
                    "shooter's shot cone or without line of sight; each shot option states "
                    "its current chance. A shot only connects while the target is in view "
                    "when it fires, so a target that moves away or behind cover can be "
                    "missed. "
                    "Bullets can hit any figure in their path, including allies (friendly "
                    "fire). A figure that sights an enemy while idle or moving may also "
                    "react and fire on its own. There is no ammo pool across rounds.")
      << ",\"acting_figures\":[";
  for (size_t i = 0; i < actorIds.size(); ++i) out << (i ? "," : "") << actorIds[i];
  out << "],\"map_half_extent\":" << std::fixed
      << std::setprecision(2) << game.GetScene().mapHalfExtent
      << ",\"fov_half_angle_deg\":" << constants::kShootHalfFovDegrees
      << ",\"shoot_range\":" << constants::kShootRange << ",\"playbook\":{";
  const SquadPlaybook& playbook = game.Playbook(team);
  out << "\"stationary_unseen\":" << JsonString(ReactionName(playbook.At(false, false)))
      << ",\"stationary_seen\":" << JsonString(ReactionName(playbook.At(false, true)))
      << ",\"moving_unseen\":" << JsonString(ReactionName(playbook.At(true, false)))
      << ",\"moving_seen\":" << JsonString(ReactionName(playbook.At(true, true)))
      << "},\"map\":{\"half_extent\":" << game.GetScene().mapHalfExtent
      << ",\"grid_cells\":" << cells << ",\"cell_size\":"
      << 2.0f * game.GetScene().mapHalfExtent / static_cast<float>(cells)
      << ",\"grid_legend\":"
      << JsonString("cell (row r, column c) covers x from -half_extent + c*cell_size and z from "
                    "-half_extent + r*cell_size; rows run -Z to +Z, columns -X to +X; # obstacle, "
                    "+ climbable obstacle, . open, digit = your figure id, e = visible enemy, "
                    "? = ghost (last known enemy position)")
      << ",\"grid\":[";
  const std::vector<std::string> grid = BuildGrid(game, team, visibility, ghosts, cells);
  for (size_t i = 0; i < grid.size(); ++i) out << (i ? "," : "") << JsonString(grid[i]);
  out << "],\"obstacles\":[";
  {
    const Scene& scene = game.GetScene();
    glm::vec3 from{0.0f};
    for (const int id : actorIds) from += game.FindUnit(id)->position;
    if (!actorIds.empty()) from /= static_cast<float>(actorIds.size());
    std::vector<size_t> order;
    for (size_t i = 0; i < scene.obstacles.size(); ++i) order.push_back(i);
    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
      return XZDistance(scene.obstacles[a].bounds.Center(), from) <
             XZDistance(scene.obstacles[b].bounds.Center(), from);
    });
    if (order.size() > kMaxListedObstacles) order.resize(kMaxListedObstacles);
    for (size_t i = 0; i < order.size(); ++i) {
      const Obstacle& o = scene.obstacles[order[i]];
      out << (i ? "," : "") << "{\"id\":" << order[i] << ",\"min_xz\":["
          << o.bounds.min.x << "," << o.bounds.min.z << "],\"max_xz\":[" << o.bounds.max.x
          << "," << o.bounds.max.z << "],\"height\":" << o.bounds.max.y - o.bounds.min.y
          << ",\"climbable\":" << (o.climbable ? "true" : "false") << "}";
    }
  }
  out << "]},\"allies\":[";

  bool first = true;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team != team) continue;
    if (!first) out << ',';
    first = false;
    out << "{\"id\":" << unit.id << ",\"alive\":" << (unit.alive ? "true" : "false")
        << ",\"position\":" << PointJson(unit.position) << ",\"facing\":"
        << std::setprecision(3) << unit.facingYaw << ",\"weapon\":"
        << JsonString(WeaponName(unit.weapon)) << ",\"magazine_size\":"
        << StatsOf(unit.weapon).magazineSize << ",\"shot_interval_s\":"
        << std::setprecision(2) << StatsOf(unit.weapon).shotIntervalSeconds
        << ",\"scatter_half_angle_deg\":" << StatsOf(unit.weapon).scatterHalfAngleDegrees
        << ",\"max_burst\":" << MaxShotsPerAction(unit.weapon, constants::kRoundDuration)
        << ",\"move_budget\":" << unit.MoveBudget()
        << ",\"planned\":"
        << (unit.plan.type == PlannedActionType::None ? "false" : "true") << "}";
  }
  out << "],\"visible_enemies\":[";
  first = true;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team == team || !unit.alive || !visibility.UnitVisible(unit.id)) continue;
    if (!first) out << ',';
    first = false;
    out << "{\"id\":" << unit.id << ",\"position\":" << PointJson(unit.position)
        << ",\"facing\":" << std::setprecision(3) << unit.facingYaw << ",\"weapon\":"
        << JsonString(WeaponName(unit.weapon)) << "}";
  }
  out << "],\"ghosts\":[";
  first = true;
  for (const Ghost& ghost : ghosts) {
    if (!first) out << ',';
    first = false;
    const auto& g = *ghost.sighting;
    out << "{\"id\":" << ghost.id << ",\"last_known_position\":" << PointJson(g.position)
        << ",\"rounds_ago\":" << g.ageRounds << ",\"was_moving\":"
        << (glm::length(g.moveDirection) > 0.01f ? "true" : "false")
        << ",\"move_direction_xz\":[" << std::setprecision(2) << g.moveDirection.x << ","
        << g.moveDirection.z << "]}";
  }
  out << "]}";
  return out.str();
}

}  // namespace

JevRequestBuilder::JevRequestBuilder(const GameLogic& game, Team team, unsigned requestNonce)
    : game_(game), team_(team) {
  if (!IsPlanning(game) || game.Mode() != InputMode::AwaitingSelection) return;
  std::vector<int> actors;
  for (const Unit& unit : game.GetScene().units) {
    if (unit.team == team && unit.alive && unit.plan.type == PlannedActionType::None) {
      actors.push_back(unit.id);
    }
  }
  if (actors.empty()) return;
  std::sort(actors.begin(), actors.end());

  request_.team = team;
  request_.round = game.RoundNumber();
  request_.id = "r" + std::to_string(request_.round) + "-t" +
                std::to_string(static_cast<int>(team)) + "-n" + std::to_string(requestNonce);
  for (const int actorId : actors) {
    for (JevCandidate& candidate : GenerateCandidateSpecs(game, team, actorId)) {
      candidate.id = "f" + std::to_string(actorId) + "_" + candidate.id;
      pending_.push_back(std::move(candidate));
    }
  }
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
  // Only figures that kept at least one legal option take part in the plan.
  for (const JevCandidate& candidate : request_.candidates) {
    if (std::find(request_.actorIds.begin(), request_.actorIds.end(), candidate.actorId) ==
        request_.actorIds.end()) {
      request_.actorIds.push_back(candidate.actorId);
    }
  }
  std::ostringstream out;
  out << "{\"requestId\":" << JsonString(request_.id) << ",\"state\":"
      << BuildVisibleState(game_, team_, request_.actorIds) << ",\"candidates\":[";
  for (size_t i = 0; i < request_.candidates.size(); ++i) {
    if (i) out << ',';
    out << "{\"id\":" << JsonString(request_.candidates[i].id) << ",\"figure\":"
        << request_.candidates[i].actorId << ",\"description\":"
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

bool ApplyJevChoice(GameLogic* game, const JevRequest& request,
                    const std::vector<std::string>& choices) {
  if (!game || game->RoundNumber() != request.round || !IsPlanning(*game) ||
      game->Mode() != InputMode::AwaitingSelection ||
      choices.size() != request.actorIds.size()) {
    return false;
  }
  // Exactly one known option per acting figure.
  std::vector<const JevCandidate*> picked;
  for (const std::string& choice : choices) {
    const auto candidate = std::find_if(request.candidates.begin(), request.candidates.end(),
                                        [&](const JevCandidate& c) { return c.id == choice; });
    if (candidate == request.candidates.end()) return false;
    for (const JevCandidate* other : picked) {
      if (other->actorId == candidate->actorId) return false;
    }
    picked.push_back(&*candidate);
  }

  // Plans are applied in turn to a copy, so the whole squad plan lands or
  // none of it does.
  GameLogic validated = *game;
  for (const JevCandidate* candidate : picked) {
    GameLogic next;
    if (!CandidateApplied(validated, request.team, *candidate, &next)) return false;
    validated = std::move(next);
  }
  *game = std::move(validated);
  return true;
}

std::vector<std::string> DeterministicFallbackChoice(const JevRequest& request) {
  std::vector<std::string> choices;
  const float sign = request.team == Team::Blue ? 1.0f : -1.0f;
  for (const int actorId : request.actorIds) {
    const JevCandidate* shoot = nullptr;
    const JevCandidate* best = nullptr;
    const JevCandidate* wait = nullptr;
    for (const JevCandidate& candidate : request.candidates) {
      if (candidate.actorId != actorId) continue;
      if (candidate.kind == JevActionKind::Shoot && !shoot) shoot = &candidate;
      if (candidate.kind == JevActionKind::Wait && !wait) wait = &candidate;
      if (candidate.kind == JevActionKind::Move &&
          (!best || sign * candidate.destination.x > sign * best->destination.x)) {
        best = &candidate;
      }
    }
    const JevCandidate* pick = shoot ? shoot : best ? best : wait;
    if (pick) choices.push_back(pick->id);
  }
  return choices;
}

}  // namespace tactics::ai
