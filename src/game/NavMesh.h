#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

// A convex, axis-aligned rectangular navmesh cell in the XZ plane.
struct NavCell {
  float xMin = 0.0f, xMax = 0.0f;
  float zMin = 0.0f, zMax = 0.0f;

  bool Contains(float x, float z) const {
    return x >= xMin && x <= xMax && z >= zMin && z <= zMax;
  }
  glm::vec3 Center() const { return glm::vec3((xMin + xMax) * 0.5f, 0.0f, (zMin + zMax) * 0.5f); }
};

// Free-space navmesh for a flat ground plane with axis-aligned box obstacles,
// built via vertical-slab decomposition. Cells form a graph searched with
// A*; the resulting cell-to-cell corridor is then pulled taut with a
// visibility-shortcut pass (greedily skipping to the furthest waypoint with
// a clear line of sight) to produce a smooth, continuous-space path.
class NavMesh {
 public:
  // `agentRadius` inflates obstacle footprints so the resulting mesh
  // represents the free space for the *center* of a circular agent.
  void Build(const std::vector<AABB>& obstacles, float mapHalfExtent, float agentRadius);

  // Returns true and fills `outPath` with a smoothed path from `start` to
  // `goal` (both XZ, Y ignored/zeroed) if a path exists. `outPath` always
  // starts with `start` and ends with `goal` on success.
  bool FindPath(glm::vec3 start, glm::vec3 goal, std::vector<glm::vec3>* outPath) const;

  // True if the given XZ point lies inside the walkable navmesh area.
  bool IsWalkable(float x, float z) const;

  const std::vector<NavCell>& Cells() const { return cells_; }

 private:
  int FindCellContaining(float x, float z) const;

  std::vector<NavCell> cells_;
  std::vector<std::vector<int>> neighbors_;      // neighbors_[cellIndex] = adjacent cell indices.
  std::vector<AABB> paddedFootprints_;           // Agent-radius-inflated obstacle footprints.
};

}  // namespace tactics
