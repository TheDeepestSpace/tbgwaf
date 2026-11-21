#pragma once

// Destruction I (issue #350): deterministic explosion -> fracture planning.
//
// An explosion is described by a compact record (position, radius, seed); the
// debug bomb emits it today and grenades will emit the same record later.
// PlanExplosion turns one such record plus the current scene into an ordered
// list of WorldChanges (see game/WorldChange.h): wall holes cut from the wall
// prisms in the blast (seeded graded Voronoi fracture, rect-merged so every
// piece stays an upright prism), ceiling/roof slab cuts, floor holes in
// building floor surfaces, and street fixtures removed. The plan is a pure
// function of (scene, spec): the server applies it through
// GameLogic::ApplyWorldChange and ships the resulting log entries, so every
// client rebuilds exactly the same holes; cosmetic-only debris can be
// regenerated client-side from the same seed.

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

// Deterministic fracture plan for one blast against the current scene.
// Change `target` indices are chained: later changes may target pieces an
// earlier change of the same plan appends, so the plan must be applied in
// order (GameLogic::ApplyWorldChange does; ids/time stamps are its job).
// Bounded by a piece budget; empty when nothing destructible is in range.
std::vector<WorldChange> PlanExplosion(const Scene& scene, const BombSpec& spec);

}  // namespace tactics
