#pragma once

// Builder scenes: a player-authored skirmish -- climbable boxes and placed
// figures on a flat square map -- described by a small data struct that is
// turned into a game Scene, serialized to one line of JSON, and embedded in
// repro logs (header field "builder_scene", see docs/repro-log.md) so a
// session on a hand-built map replays headlessly without any registry entry.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "game/GameLogic.h"
#include "game/ReproLog.h"
#include "game/Scene.h"

namespace tactics {

// A climbable box: centered at (cx, cz) with half extents (hx, hz) in XZ,
// `height` tall, its underside at `y` (0 = on the ground; above 0 = stacked on
// other boxes). Its top is BoxTop().
struct BuilderBox {
  float cx = 0.0f, cz = 0.0f;
  float hx = 1.5f, hz = 1.5f;
  float height = 1.2f;
  float y = 0.0f;
};
inline float BoxTop(const BuilderBox& box) { return box.y + box.height; }

// A figure standing at (x, z) on whatever surface is highest there.
struct BuilderFigure {
  Team team = Team::Blue;
  WeaponType weapon = WeaponType::AssaultRifle;
  float x = 0.0f, z = 0.0f;
  // Facing in radians from +X (as Unit::facingYaw), used when `hasYaw`;
  // otherwise the figure faces its enemy side (Blue +X, Red -X).
  float yaw = 0.0f;
  bool hasYaw = false;
};

struct BuilderScene {
  std::string name = "Untitled";
  float mapHalfExtent = 20.0f;  // "Medium" flat surface.
  int roundLimit = 0;           // Same default as Scene: unlimited.
  std::vector<BuilderBox> boxes;  // All climbable.
  std::vector<BuilderFigure> figures;
};

// Game Scene for `bs`: flat ground, `mapHalfExtent` as the playable area
// (the map edge is enforced by the simulation, as in the default scene), one
// climbable Obstacle per box, one Unit per figure with id == index, the
// figure's explicit weapon, facing its explicit yaw (default: its enemy side,
// Blue +X, Red -X) and standing at SurfaceHeightAt(x, z). No flag.
Scene BuildScene(const BuilderScene& bs);

// Top of the highest box whose XZ extent contains (x, z) (edges inclusive),
// else 0.
float SurfaceHeightAt(const BuilderScene& bs, float x, float z);

// Single-line JSON (no trailing newline); floats round-trip exactly.
std::string SerializeBuilderScene(const BuilderScene& bs);
// Parses SerializeBuilderScene text. Returns false (and sets *error, if
// given) on malformed or out-of-range input; *out is only written on
// success.
bool ParseBuilderScene(const std::string& text, BuilderScene* out, std::string* error = nullptr);

// Per-session setup of a Builder game, mirroring InitDemoGame: resets the
// config a GameLogic carries across Reset() (recorder, shot-roll source,
// playbooks, friendly fire), seeds the shot RNG with `seed`, then
// Reset(BuildScene(bs)).
void InitBuilderGame(GameLogic* game, const BuilderScene& bs, uint32_t seed);

// Scene id recorded in a builder log's header: "builder:" + bs.name.
std::string BuilderSceneLogId(const BuilderScene& bs);

// Parses `text`, requires a header "builder_scene", rebuilds the game with
// InitBuilderGame(header seed) and replays the log into `*game`. On a
// parse failure or a log without builder_scene the report's `error` is set
// and nothing is replayed. `parsed`/`afterEntry` as in ReplayDemoLog.
ReplayReport ReplayBuilderLog(
    const std::string& text, GameLogic* game, ReproLog* parsed = nullptr,
    const std::function<void(const ReproEntry&, float dt)>& afterEntry = {});

// Replays any repro log: builder logs through ReplayBuilderLog, everything
// else through ReplayDemoLog (registry lookup). This is what
// tactics_repro_replay uses.
ReplayReport ReplayAnyLog(
    const std::string& text, GameLogic* game, ReproLog* parsed = nullptr,
    const std::function<void(const ReproEntry&, float dt)>& afterEntry = {});

}  // namespace tactics
