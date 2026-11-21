#pragma once

// Destruction I (issue #350): deterministic explosion -> fracture planning.
//
// An explosion is described by a compact record (position, radius, seed); the
// debug bomb emits it today and grenades will emit the same record later.
// PlanExplosion turns one such record plus the current scene into an ordered
// list of WorldChanges (see game/WorldChange.h), the way pre-fractured
// destruction usually works (Voronoi cells, damage by radius): every concrete
// element the blast reaches is pre-cracked into seeded Voronoi cells in its
// own plane -- walls, the jamb / sill / head strips around windows and corner
// posts in the wall plane; ceiling / roof slabs, floor surfaces and the roof
// massing (parapets) in XZ. The crack pattern is world-anchored, so coplanar
// elements crack along the same lines. Cells near the blast are knocked out;
// the others stay standing as convex pieces (wall pieces carry the cell as
// their wall-plane profile), so every torn edge sits at the fracture's own
// angle. Each knocked-out cell becomes a debris chunk of exactly that shape:
// the Debris change simulates its flight here against the post-cut scene
// until it lands flat -- the largest chunks as rubble obstacles (blocking
// movement and sight), smaller ones as litter -- with per-chunk flight
// parameters so clients replay the same fall out of the same hole. The plan
// is a pure function of (scene, spec): the server applies it through
// GameLogic::ApplyWorldChange and ships the resulting log entries, so every
// client rebuilds exactly the same holes and rubble.

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "game/Scene.h"
#include "game/WorldChange.h"

namespace tactics {

struct BombSpec {
  glm::vec3 position{0.0f};
  float radius = 3.5f;
  uint32_t seed = 0;
};

constexpr float kDefaultBombRadius = 3.5f;

// Explosion seed = hash(match seed, round, bomb id): stable across replays
// and reconnects, never derived from a clock.
uint32_t ExplosionSeed(uint32_t matchSeed, int round, int bombId);

// Deterministic fracture plan for one blast against the current scene. The
// plan must be applied in order (GameLogic::ApplyWorldChange does; ids/time
// stamps are its job). Bounded by a piece budget; empty when nothing
// destructible is in range.
std::vector<WorldChange> PlanExplosion(const Scene& scene, const BombSpec& spec);

// Where debris chunk `piece` (a Debris change's rest-pose prism) is `t`
// seconds after launch, as the transform to apply to its rest geometry: the
// closed-form ballistic between the recorded launch point and the rest pose,
// yawing and tumbling so that at t = 0 the chunk sits exactly in the cell it
// was knocked out of and from `duration` on it is the identity.
glm::mat4 DebrisPose(const Obstacle& piece, const DebrisFlight& flight, float t);

}  // namespace tactics
