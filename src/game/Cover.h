#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "game/NavMesh.h"
#include "game/Types.h"

namespace tactics {

// Cover (issue #290, phase 1). Purely geometric and static: it depends only
// on the obstacles around a tile, never on where enemies stand, and uses no
// randomness or floating accumulation, so it is replay-safe (cf. #284).
//
// A tile is a cover tile when a wall/blocker sits right beside it.
//  - Flat: every adjacent blocker keeps running along both sides of the tile
//    (standing along a wall) -- nothing to peek around.
//  - Tactical: some adjacent blocker ends within kCoverEdgeReach of the tile
//    (building corner, end/side of a car) -- a spot to peek from.
enum class CoverKind : uint8_t { None = 0, Flat = 1, Tactical = 2 };

constexpr float kCoverProbeDistance = 0.7f;  // Tile centre to blocker face probe.
constexpr float kCoverEdgeReach = 0.75f;     // How far along the wall an open edge counts.
// A crouched figure is ~1.15 tall; a blocker must hide at least 70% of that
// (more than 30% visible = no protection, so it is not cover at all).
constexpr float kCrouchedHeight = 1.15f;
constexpr float kCoverMinHeight = 0.7f * kCrouchedHeight;  // Blocker must rise this far above the feet.
constexpr float kCoverStandHeight = 1.4f;  // At/above this the figure stands with its back to it.

struct CoverTile {
  CoverKind kind = CoverKind::None;
  // Bit k set = a blocker lies in compass direction k, the direction at
  // angle k * 45 degrees from +X toward +Z (matches Unit::facingYaw).
  uint8_t dirMask = 0;
  // Tallest adjacent blocker rises less than kCoverStandHeight above the feet:
  // the figure crouches behind it instead of standing against it.
  bool crouch = false;
};

// Classifies a feet position. `yaw` (if given) receives the world yaw
// pointing at the blockers (mean of the blocked directions), for poses.
CoverTile ClassifyCover(const std::vector<Obstacle>& obstacles, const glm::vec3& feet,
                        float* yaw = nullptr);

// Per-node cover for every node of `field` (same grid and indexing). Nodes
// the field does not reach stay None.
struct CoverMap {
  int nx = 0, nz = 0;
  std::vector<CoverTile> tiles;
  const CoverTile& At(int ix, int iz) const { return tiles[iz * nx + ix]; }
};

CoverMap ComputeCoverMap(const std::vector<Obstacle>& obstacles, const ReachField& field);

}  // namespace tactics
