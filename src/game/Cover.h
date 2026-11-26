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
// A tile is a cover tile when it stands beside a wall face -- in front of
// the face, not past its span (diagonally off a corner is no cover).
//  - Flat: the wall keeps running (standing along a wall) -- nothing to
//    peek around.
//  - Tactical: the stretch of wall leading up to an open end (building
//    corner, end of a car), cut at 45 degrees: the tile is within
//    kCoverEdgeReach of the corner along the wall and no farther from the
//    wall than from the corner, so the wedge tapers to a point at the
//    corner itself.
enum class CoverKind : uint8_t { None = 0, Flat = 1, Tactical = 2 };

constexpr float kCoverProbeDistance = 0.7f;  // Tile centre to blocker face probe.
constexpr float kCoverEdgeReach = 0.75f;     // How far along the wall an open edge counts.
// A crouched figure is ~1.15 tall; a blocker must hide at least 70% of that
// (more than 30% visible = no protection, so it is not cover at all).
// The heights themselves live in constants:: (Types.h) because the crouched
// pose also shrinks the sim's hit box and sight/aim probes (issue #292).
constexpr float kCrouchedHeight = constants::kCrouchedHeight;
constexpr float kCoverMinHeight = 0.7f * kCrouchedHeight;  // Blocker must rise this far above the feet.
constexpr float kCoverStandHeight = 1.4f;  // At/above this the figure stands with its back to it.
// Chest height of a figure holding the crouch pose, for judging whether a
// blocker actually lies on an enemy's shot line at it (it sits below
// kCoverMinHeight, so any qualifying blocker crossing that line stops it).
constexpr float kCrouchChestHeight = constants::kCrouchChestHeight;

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
// pointing at the blockers (mean of the wall directions), for poses.
CoverTile ClassifyCover(const std::vector<Obstacle>& obstacles, const glm::vec3& feet,
                        float* yaw = nullptr);

// One cover-zone rectangle for overlays (issue #292): a band-deep stretch
// along a wall face. A face with an open end contributes a tactical stretch
// (the kCoverEdgeReach of wall leading up to the corner) and the remainder
// is flat; rects of one face never overlap. The overlay draws tactical
// rects as solid fills and flat rects as angled stripes -- schematic marks,
// not gradients.
struct CoverZoneRect {
  glm::vec2 origin{0.0f};  // On the face line, at the stretch's start.
  glm::vec2 along{0.0f};   // Unit direction from origin toward the stretch's end.
  glm::vec2 out{0.0f};     // Unit outward normal of the face.
  float length = 0.0f;     // Stretch length along the face.
  float depth = 0.0f;      // Band depth, outward from the face.
  bool tactical = false;
};

// All cover-zone rectangles for feet standing at height `feetY`, `band` deep
// off each qualifying wall face. Same face/height/open-end rules as
// ClassifyCover, so the drawn zones follow the walls exactly instead of the
// nav grid.
std::vector<CoverZoneRect> CoverZoneRects(const std::vector<Obstacle>& obstacles, float feetY,
                                          float band);

// Per-node cover for every node of `field` (same grid and indexing). Nodes
// the field does not reach stay None.
struct CoverMap {
  int nx = 0, nz = 0;
  std::vector<CoverTile> tiles;
  const CoverTile& At(int ix, int iz) const { return tiles[iz * nx + ix]; }
};

CoverMap ComputeCoverMap(const std::vector<Obstacle>& obstacles, const ReachField& field);

}  // namespace tactics
