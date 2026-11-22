#pragma once

// Destruction I (issue #350): deterministic explosion -> fracture planning.
//
// An explosion is described by a compact record (position, radius, seed); the
// debug bomb emits it today and grenades will emit the same record later.
// PlanExplosion turns one such record plus the current scene into an ordered
// list of WorldChanges (see game/WorldChange.h), the way pre-fractured
// destruction usually works (Voronoi cells, damage by radius): every concrete
// element the blast reaches is pre-cracked into seeded Voronoi cells --
// walls, the jamb / sill / head strips around windows and corner posts in
// the wall plane; ceiling / roof slabs, floors, overpass decks and ramps and
// the massing (parapets, deck rails, pier caps) in XZ; thick solids
// (overpass piers, solid building blocks) in XZ per layer of their height.
// The crack pattern is world-anchored, so coplanar elements crack along the
// same lines. Cells near the blast are knocked out; the others stay standing
// as convex pieces (wall pieces carry the cell as their wall-plane profile),
// so every torn edge sits at the fracture's own angle. A stair flight the
// blast reaches comes down whole; window frames in the blast are blown off,
// the others stay on what is left standing. Terrain is not cut.
//
// Everything knocked out is accounted for: each cell becomes a debris chunk
// of exactly that shape, a rigid body thrown by the blast and simulated here
// (game/DebrisPhysics.h) against the post-cut scene: it tumbles, bounces off
// standing geometry, the ground, floors, figures and the other chunks, and
// comes to rest in whatever pose the contacts leave it in -- flat, tilted,
// leaning on a neighbour, heaped in the breach if that is where it lands.
// Chunks of 0.1 m^2 and up that settle are rubble obstacles (blocking
// movement and sight through their vertical envelope), smaller ones litter;
// the Debris changes carry each chunk's sampled pose track so clients replay
// the very same collisions. The plan is a pure function of (scene, spec): the
// server applies it through GameLogic::ApplyWorldChange and ships the
// resulting log entries, so every client rebuilds exactly the same holes and
// rubble.

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

#include "game/DebrisPhysics.h"
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
// stamps are its job). Bounded by a cut budget (elements past it are left
// intact, never cut without their chunks); empty when nothing destructible
// is in range.
std::vector<WorldChange> PlanExplosion(const Scene& scene, const BombSpec& spec);

// The prism of a Debris change's `piece` at launch (in the wall or slab it
// was cut from), or of a rubble obstacle where it lies. False when `piece`
// has no usable outline.
bool DebrisPieceShape(const Obstacle& piece, DebrisShape* shape);

// The transform to apply to a chunk's launch geometry `t` seconds after
// launch: its sampled track, linearly interpolated (identity at t = 0; held
// at the rest pose from the end of the track on).
glm::mat4 DebrisPose(const DebrisFlight& flight, float t);

// How long a Debris change's flights have been running in a game showing
// `round`, `elapsed` seconds into its execution (`executing`: the round is
// playing, live or replayed). A blast inside a round (`at` > 0) runs on the
// round clock alone: 0 on the frame it goes off, the same age for the same
// clock (so a paused replay holds the chunks still), frozen at the turn's end
// and carried on through the turns that follow. Negative for a change made
// outside a round (`at` is exactly 0 only then): no round clock covers it.
float DebrisRoundAge(const WorldChange& change, int round, bool executing, float elapsed);

// The rubble obstacle a settled chunk becomes: its body at rest, with the
// footprint / bounds of its vertical envelope. Used by both the planner and
// every client applying the change, so they agree to the bit.
Obstacle DebrisRestObstacle(const Obstacle& piece, const DebrisFlight& flight);

}  // namespace tactics
