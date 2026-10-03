#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

// Axis-aligned XZ rectangle the navmesh covers.
struct NavRegion {
  float xMin = 0.0f, xMax = 0.0f;
  float zMin = 0.0f, zMax = 0.0f;
};

// A convex, axis-aligned rectangular navmesh cell in the XZ plane. Ground
// cells (`climbTop == false`) have elevation 0 in the mesh's internal flat
// space and follow the scene's HeightField (if any) in world space;
// "climb-top" cells (see NavMesh::Build) sit atop a climbable obstacle at
// that obstacle's height.
struct NavCell {
  float xMin = 0.0f, xMax = 0.0f;
  float zMin = 0.0f, zMax = 0.0f;
  float elevation = 0.0f;
  bool climbTop = false;

  bool Contains(float x, float z) const {
    return x >= xMin && x <= xMax && z >= zMin && z <= zMax;
  }
  glm::vec3 Center() const {
    return glm::vec3((xMin + xMax) * 0.5f, elevation, (zMin + zMax) * 0.5f);
  }
};

// Path-distance field from a single source, sampled on a regular XZ grid
// (node (ix, iz) sits at (minX + ix*step, minZ + iz*step)). `dist` holds the
// routed (around-obstacles) ground distance from the source -- walked over
// the terrain HeightField when the mesh has one -- or infinity for
// unreachable / out-of-budget / non-walkable nodes. Node() always reports
// y = 0; consumers that need terrain-following geometry sample the scene's
// HeightField themselves.
struct ReachField {
  float minX = 0.0f, minZ = 0.0f, step = 1.0f;
  int nx = 0, nz = 0;
  float budget = 0.0f;
  std::vector<float> dist;
  // World-space surface height selected for each reached node. This keeps a
  // ramp/deck frontier on its own layer instead of snapping it to terrain.
  std::vector<float> surfaceY;

  bool Reached(int ix, int iz) const {
    return ix >= 0 && iz >= 0 && ix < nx && iz < nz && dist[iz * nx + ix] <= budget;
  }
  float Dist(int ix, int iz) const { return dist[iz * nx + ix]; }
  glm::vec3 Node(int ix, int iz) const {
    const int index = iz * nx + ix;
    const float y = index >= 0 && static_cast<size_t>(index) < surfaceY.size()
                        ? surfaceY[index]
                        : 0.0f;
    return glm::vec3(minX + ix * step, y, minZ + iz * step);
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
  // ground level up to their top surface. `ground` (optional) is the scene's
  // terrain heightfield: paths and elevation queries then follow the sampled
  // terrain instead of a flat y = 0 plane. The field is copied; the caller's
  // instance need not outlive the mesh.
  void Build(const std::vector<Obstacle>& obstacles, float mapHalfExtent, float agentRadius,
             const HeightField* ground = nullptr,
             const std::vector<WalkSurface>* walkSurfaces = nullptr);

  // Windowed form: meshes only the XZ rectangle `region` (obstacles are
  // clipped to it). Used by GameLogic to mesh just the area a figure can
  // reach in one round instead of the whole map.
  void Build(const std::vector<Obstacle>& obstacles, const NavRegion& region, float agentRadius,
             const HeightField* ground = nullptr,
             const std::vector<WalkSurface>* walkSurfaces = nullptr);

  // Returns true and fills `outPath` with a smoothed path from `start` to
  // `goal` (XZ plus an elevation hint used to disambiguate overlapping
  // ground/climb-top cells) if a path exists. `outPath` always starts at
  // `start`'s XZ and ends at `goal`'s XZ (with Y snapped to the resolved
  // cell's surface height) on success. With a terrain heightfield, ground
  // segments are densified (one waypoint per ~kTerrainPathStep) and each
  // waypoint's Y sampled from the terrain, so linear interpolation between
  // waypoints tracks the ground.
  bool FindPath(glm::vec3 start, glm::vec3 goal, std::vector<glm::vec3>* outPath) const;

  // World-space Y of the walkable surface of `cell` at (x, z): the obstacle
  // top for climb-top cells, else the terrain height (0 when flat).
  float SurfaceY(const NavCell& cell, float x, float z) const {
    return cell.climbTop ? cell.elevation : ground_.HeightAt(x, z);
  }

  // Highest/nearest explicit surface at XZ for a supplied elevation hint;
  // returns terrain height when no walk-surface is selected.
  float ResolveSurfaceY(float x, float z, float yHint) const;

  // True if the given XZ point lies inside the walkable navmesh area at
  // ground level (elevation 0); climb-top surfaces are not considered.
  bool IsWalkable(float x, float z) const;

  // Dijkstra flood fill over ground-level free space from `start`, covering
  // everything within `budget` path distance. Climb-top surfaces are not
  // included. Returns an empty field if `start` isn't on walkable ground.
  ReachField ComputeReachField(const glm::vec3& start, float budget, float step = 0.25f) const;

  const std::vector<NavCell>& Cells() const { return cells_; }
  const std::vector<WalkSurface>& WalkSurfaces() const { return walkSurfaces_; }

 private:
  void BuildGroundMesh(const std::vector<AABB>& obstacles, const NavRegion& region,
                        float agentRadius);
  void AddClimbConnections(const std::vector<Obstacle>& obstacles, float agentRadius);

  int FindCellContaining(float x, float z) const;
  int FindCellContaining(float x, float z, float yHint) const;
  int FindWalkSurfaceContaining(float x, float z, float yHint) const;
  bool FindGroundPath(glm::vec3 start, glm::vec3 goal,
                      std::vector<glm::vec3>* outPath) const;
  void LiftGroundPathOntoTerrain(std::vector<glm::vec3>* outPath) const;
  bool FindPolygonGroundPath(glm::vec3 start, glm::vec3 goal,
                             std::vector<glm::vec3>* outPath) const;
  bool FindSurfacePath(int startSurface, glm::vec3 start, int goalSurface,
                       glm::vec3 goal, std::vector<glm::vec3>* outPath) const;
  glm::vec3 GroundConnectionPoint(int surface) const;

  std::vector<NavCell> cells_;
  std::vector<std::vector<int>> neighbors_;      // neighbors_[cellIndex] = adjacent cell indices.
  std::vector<AABB> paddedFootprints_;           // Agent-radius-inflated obstacle footprints.
  NavRegion region_;
  HeightField ground_;                            // Empty = flat ground at y 0.
  std::vector<WalkSurface> walkSurfaces_;
  std::vector<std::vector<glm::vec2>> paddedPolygons_;
  bool polygonMode_ = false;
};

}  // namespace tactics
