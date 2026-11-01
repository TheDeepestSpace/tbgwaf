#pragma once

// Demo scenes (issue #240): small fixed setups -- a few figures on a minimal
// map, normal game controls -- for reproducing odd behaviour, recorded as
// repro logs (see game/ReproLog.h). The registry is data-driven: a scene is
// an id, a name/description, a shot-RNG seed and a function that builds its
// Scene. The app no longer lists them (the user-facing repro flow is the
// Builder, see game/BuilderScene.h); the registry stays for the scenario
// tests (`map: {demo_scene: <id>}`), registry logs (ReplayDemoLog,
// tactics_repro_replay, TBGWAF_REPRO_LOG) and repro_log_tests. To add one,
// append a DemoScene to kDemoScenes in DemoScenes.cpp (and, per repo
// convention, a tests/scenarios/demo_<id>.yaml that pins it).

#include <cstdint>
#include <string>
#include <vector>

#include "game/ReproLog.h"
#include "game/Scene.h"

namespace tactics {

struct DemoScene {
  const char* id;           // Stable; recorded in repro logs.
  const char* name;         // Display name.
  const char* description;  // What to do to reproduce.
  uint32_t seed;            // Shot-RNG seed for the session.
  Scene (*build)();
  // Where a preview camera looks (XZ) and its zoom delta from the
  // map-fitted view (positive = further out), like a scenario's `camera:`.
  float cameraTargetX, cameraTargetZ, cameraZoom;
};

const std::vector<DemoScene>& DemoScenes();
// nullptr if no scene has that id.
const DemoScene* FindDemoScene(const std::string& id);

// Builds the scene and a game on it with the seed applied: the state every
// recording of that scene starts from. Also resets the per-session config a
// GameLogic carries across Reset() (playbooks, friendly fire) to defaults so
// the log's header fully describes its starting point.
void InitDemoGame(const DemoScene& scene, GameLogic* game);

// Parses `text`, looks its scene up, builds a fresh game for it and replays
// the log into `*game` (see ReplayReproLog). On a parse/lookup failure the
// report's `error` is set and nothing is replayed. `afterEntry` as in
// ReplayReproLog.
ReplayReport ReplayDemoLog(const std::string& text, GameLogic* game, ReproLog* parsed = nullptr,
                           const std::function<void(const ReproEntry&, float dt)>& afterEntry = {});

}  // namespace tactics
