#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

// A convex, axis-aligned rectangular navmesh cell in the XZ plane, resting
// at a fixed `elevation` (world-space Y). Ground cells have elevation 0;
// "climb-top" cells (see NavMesh::Build) sit atop a climbable obstacle at
// that obstacle's height.
struct NavCell {
  float xMin = 0.0f, xMax = 0.0f;
  float zMin = 0.0f, zMax = 0.0f;
  float elevation = 0.0f;

  bool Contains(float x, float z) const {
    return x >= xMin && x <= xMax && z >= zMin && z <= zMax;
  }
  glm::vec3 Center() const {
    return glm::vec3((xMin + xMax) * 0.5f, elevation, (zMin + zMax) * 0.5f);
  }
};

// Free-space navmesh for a flat ground plane with axis-aligned box obstacles,
// built via vertical-slab decomposition. Cells form a graph searched with
// A*; the resulting cell-to-cell corridor is then pulled taut with the
// funnel algorithm (bending only at obstacle corners) to produce a
// shortest continuous-space path within the corridor.
//
// Climbable obstacles additionally get a "climb-top" cell placed at their
// summit, wired into the graph with edges to every ground cell bordering
// their footprint. A* naturally routes across these edges when climbing
// onto/over an obstacle is part of the shortest path.
class NavMesh {
 public:
  // `agentRadius` inflates obstacle footprints so the resulting mesh
  // represents the free space for the *center* of a circular agent.
  //
  // Stage-A form: every obstacle is a plain impassable box (no climbing).
  void Build(const std::vector<AABB>& obstacles, float mapHalfExtent, float agentRadius);

  // Stage-C form: obstacles marked `climbable` get a navmesh connection from
  // ground level up to their top surface.
  void Build(const std::vector<Obstacle>& obstacles, float mapHalfExtent, float agentRadius);

  // Returns true and fills `outPath` with a smoothed path from `start` to
  // `goal` (XZ plus an elevation hint used to disambiguate overlapping
  // ground/climb-top cells) if a path exists. `outPath` always starts at
  // `start`'s XZ and ends at `goal`'s XZ (with Y snapped to the resolved
  // cell's elevation) on success.
  bool FindPath(glm::vec3 start, glm::vec3 goal, std::vector<glm::vec3>* outPath) const;

  // True if the given XZ point lies inside the walkable navmesh area at
  // ground level (elevation 0); climb-top surfaces are not considered.
  bool IsWalkable(float x, float z) const;

  const std::vector<NavCell>& Cells() const { return cells_; }

 private:
  void BuildGroundMesh(const std::vector<AABB>& obstacles, float mapHalfExtent,
                        float agentRadius);
  void AddClimbConnections(const std::vector<Obstacle>& obstacles, float agentRadius);

  int FindCellContaining(float x, float z) const;
  int FindCellContaining(float x, float z, float yHint) const;

  std::vector<NavCell> cells_;
  std::vector<std::vector<int>> neighbors_;      // neighbors_[cellIndex] = adjacent cell indices.
  std::vector<AABB> paddedFootprints_;           // Agent-radius-inflated obstacle footprints.
  float mapHalfExtent_ = 0.0f;
};

}  // namespace tactics
