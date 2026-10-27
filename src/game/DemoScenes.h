#pragma once

// Demo scenes (issue #240): small fixed setups -- a few figures on a minimal
// map, normal game controls -- for reproducing odd behaviour and handing the
// recorded session (see game/ReproLog.h) to developers/agents. The registry
// is data-driven: a scene is an id, a name/description for its card on the
// Demo Scenes screen, a shot-RNG seed and a function that builds its Scene.
// To add one, append a DemoScene to kDemoScenes in DemoScenes.cpp (and,
// per repo convention, a tests/scenarios/demo_<id>.yaml that pins it with
// `map: {demo_scene: <id>}`).
//
// Thumbnails are rendered by the app itself the first time the Demo Scenes
// screen opens (one offscreen RenderPane per card), so a new scene needs no
// asset.

#include <cstdint>
#include <string>
#include <vector>

#include "game/ReproLog.h"
#include "game/Scene.h"

namespace tactics {

struct DemoScene {
  const char* id;           // Stable; recorded in repro logs.
  const char* name;         // Card button label.
  const char* description;  // What to do to reproduce.
  uint32_t seed;            // Shot-RNG seed for the session.
  Scene (*build)();
  // Where the thumbnail camera looks (XZ) and its zoom delta from the
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
