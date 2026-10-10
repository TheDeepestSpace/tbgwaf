#include <SDL.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl2.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <functional>
#endif

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "game/GameLogic.h"
#include "game/Geometry.h"
#include "game/MapGenerator.h"
#include "game/Raycast.h"
#include "game/TurnTimeline.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"
#include "net/RemoteClient.h"
#include "AppFlow.h"
#include "ui/Hud.h"
#include "ui/Menu.h"

using tactics::DeserializeSnapshot;
using tactics::GameLogic;
using tactics::GameSnapshot;
using tactics::InputMode;
using tactics::Obstacle;
using tactics::SerializeSnapshot;
using tactics::Team;
using tactics::TeamVisibility;
using tactics::Unit;

namespace {

constexpr int kInitialWindowWidth = 1280;
constexpr int kInitialWindowHeight = 720;

// Panes: a native build shows both teams side by side in one window (left =
// Blue, right = Red) off a single GameLogic. The web page
// (web/index.html) instead shows them as two separate <canvas>es by
// instantiating this module twice -- one client per team, each with its own
// canvas, camera, input, and UI, filling that canvas with a single pane.
// Both teams plan at once (WEGO), so the two instances share one match by
// exchanging state over a page-level message bus: each instance ships its
// own team's plans while planning, and the Blue instance alone commits and
// simulates the round (Red asks it to), broadcasting snapshots that the Red
// instance mirrors rather than re-simulating.
constexpr int kMaxPanes = 2;
constexpr Uint32 kPeerSyncTimeoutMs = 500;

#ifdef __EMSCRIPTEN__
// Emscripten's SDL2 port reaches the canvas through the "#canvas" CSS
// selector (canvas sizing, mouse-event registration), which cannot address
// per-instance canvases in a page that instantiates this module twice. Bind
// the selector to this instance's own canvas via the module-scoped
// specialHTMLTargets table instead; must run before SDL_Init.
EM_JS(void, tbgwaf_bind_canvas, (), {
  specialHTMLTargets["#canvas"] = Module.canvas;
});

// 0 = blue, 1 = red, -1 = not specified. The page assigns each client
// instance its team via the Module.tbgwafPlayer override.
EM_JS(int, tbgwaf_requested_player, (), {
  if (Module.tbgwafPlayer === "blue") return 0;
  if (Module.tbgwafPlayer === "red") return 1;
  return -1;
});

// The bus (Module.tbgwafBus, one object shared by both instances on the
// page) is just a list of per-instance inboxes: posting appends the message
// to every inbox but our own, and each instance drains its inbox at the top
// of its frame.
EM_JS(void, tbgwaf_channel_open, (), {
  if (Module.tbgwafInbox) return;
  Module.tbgwafInbox = [];
  if (Module.tbgwafBus) Module.tbgwafBus.inboxes.push(Module.tbgwafInbox);
});

EM_JS(void, tbgwaf_channel_post, (const char* msg), {
  if (!Module.tbgwafBus) return;
  const text = UTF8ToString(msg);
  for (const inbox of Module.tbgwafBus.inboxes) {
    if (inbox !== Module.tbgwafInbox) inbox.push(text);
  }
});

// Returns a malloc'd message (caller frees) or null if the inbox is empty.
EM_JS(char*, tbgwaf_channel_next, (), {
  if (!Module.tbgwafInbox || Module.tbgwafInbox.length === 0) return 0;
  const msg = Module.tbgwafInbox.shift();
  const size = lengthBytesUTF8(msg) + 1;
  const ptr = _malloc(size);
  stringToUTF8(msg, ptr, size);
  return ptr;
});

// Server mode (real networking, as opposed to the in-page bus above): the
// page passes the WebSocket URL as Module.tbgwafServer (and optionally a
// room name as Module.tbgwafRoom). Returns a malloc'd string (caller frees),
// empty when unset.
EM_JS(char*, tbgwaf_server_url, (), {
  const text = Module.tbgwafServer || "";
  const size = lengthBytesUTF8(text) + 1;
  const ptr = _malloc(size);
  stringToUTF8(text, ptr, size);
  return ptr;
});
EM_JS(char*, tbgwaf_server_room, (), {
  const text = Module.tbgwafRoom || "";
  const size = lengthBytesUTF8(text) + 1;
  const ptr = _malloc(size);
  stringToUTF8(text, ptr, size);
  return ptr;
});

// WebSocket plumbing: events are queued as "O" (open), "C" (closed) and
// "M<text frame>" and drained by the frame loop.
EM_JS(void, tbgwaf_ws_open, (const char* url), {
  const queue = [];
  Module.tbgwafWsQueue = queue;
  const ws = new WebSocket(UTF8ToString(url));
  Module.tbgwafWs = ws;
  ws.onopen = () => queue.push("O");
  ws.onclose = () => queue.push("C");
  ws.onerror = () => {};
  ws.onmessage = (e) => { if (typeof e.data === "string") queue.push("M" + e.data); };
});

EM_JS(void, tbgwaf_ws_send, (const char* msg), {
  const ws = Module.tbgwafWs;
  if (ws && ws.readyState === 1) ws.send(UTF8ToString(msg));
});

// Returns a malloc'd event (caller frees) or null if none are queued.
EM_JS(char*, tbgwaf_ws_next, (), {
  const queue = Module.tbgwafWsQueue;
  if (!queue || queue.length === 0) return 0;
  const msg = queue.shift();
  const size = lengthBytesUTF8(msg) + 1;
  const ptr = _malloc(size);
  stringToUTF8(msg, ptr, size);
  return ptr;
});

// CSS size of this instance's own canvas (each client renders into its own
// <canvas>, so a global "#canvas" selector would not do).
EM_JS(double, tbgwaf_canvas_css_width, (), {
  return Module.canvas.getBoundingClientRect().width;
});
EM_JS(double, tbgwaf_canvas_css_height, (), {
  return Module.canvas.getBoundingClientRect().height;
});
#endif

// Finds the alive unit whose bounding box the ray hits nearest, or -1.
int PickUnit(const gfx::Ray& ray, const std::vector<Unit>& units) {
  int bestId = -1;
  float bestT = std::numeric_limits<float>::infinity();
  for (const auto& unit : units) {
    if (!unit.alive) continue;
    float t = 0.0f;
    if (tactics::RayIntersectsAABB(ray.origin, ray.direction, unit.Bounds(), &t)) {
      if (t < bestT) {
        bestT = t;
        bestId = unit.id;
      }
    }
  }
  return bestId;
}


// Terrain picking: coarse ray-march against the heightfield, refined by
// bisection once the ray first dips below the ground. Replaces the flat
// y=0 plane intersection on scenes with sampled terrain, so a click on a
// hillside lands on the slope the player actually pointed at (a flat-plane
// hit would land several units past it at shallow camera angles).
bool IntersectTerrain(const gfx::Ray& ray, const tactics::HeightField& ground, glm::vec3* outPoint,
                      float* outT) {
  constexpr float kCoarseStep = 0.5f;
  constexpr float kMaxDistance = 2000.0f;
  float prevT = 0.0f;
  if (ray.origin.y - ground.HeightAt(ray.origin.x, ray.origin.z) <= 0.0f) return false;
  for (float t = kCoarseStep; t <= kMaxDistance; t += kCoarseStep) {
    const glm::vec3 p = ray.origin + ray.direction * t;
    if (p.y - ground.HeightAt(p.x, p.z) <= 0.0f) {
      float lo = prevT, hi = t;
      for (int i = 0; i < 24; ++i) {
        const float mid = 0.5f * (lo + hi);
        const glm::vec3 q = ray.origin + ray.direction * mid;
        (q.y - ground.HeightAt(q.x, q.z) <= 0.0f ? hi : lo) = mid;
      }
      *outT = 0.5f * (lo + hi);
      *outPoint = ray.origin + ray.direction * *outT;
      return true;
    }
    prevT = t;
  }
  return false;
}

// Stage-C climbing: a ground/move click can land either on the ground
// surface (the y=0 plane, or the sampled terrain when the scene has one) or
// on top of a climbable obstacle (a crate's top face). Pick whichever the
// ray actually hits nearer the camera -- matches how the scene is rendered
// (the ground is drawn everywhere but obstacles occlude it visually).
//
// Any walkable surface counts, not just crates: climbable obstacles and the
// scene's flat walkable slabs (sidewalks) are all picked by their top face.
bool IntersectGroundOrClimbTop(const gfx::Ray& ray, const tactics::Scene& scene, glm::vec3* outPoint) {
  bool found = false;
  float bestT = std::numeric_limits<float>::infinity();
  glm::vec3 bestPoint(0.0f);

  glm::vec3 groundPoint;
  float terrainT = 0.0f;
  if (!scene.ground.Empty()) {
    if (IntersectTerrain(ray, scene.ground, &groundPoint, &terrainT)) {
      found = true;
      bestT = terrainT;
      bestPoint = groundPoint;
    }
  } else if (gfx::OrbitCamera::IntersectGroundPlane(ray, &groundPoint)) {
    const float t = glm::dot(groundPoint - ray.origin, ray.direction);
    found = true;
    bestT = t;
    bestPoint = groundPoint;
  }

  constexpr float kTopFaceEpsilon = 1e-2f;
  auto considerSurface = [&](const tactics::AABB& bounds) {
    float t = 0.0f;
    if (!tactics::RayIntersectsAABB(ray.origin, ray.direction, bounds, &t)) return;
    const glm::vec3 hit = ray.origin + ray.direction * t;
    if (hit.y < bounds.max.y - kTopFaceEpsilon) return;  // Hit a side, not the top.
    if (t < bestT) {
      found = true;
      bestT = t;
      bestPoint = hit;
    }
  };
  for (const auto& obstacle : scene.obstacles) {
    if (obstacle.climbable) considerSurface(obstacle.bounds);
  }
  for (const tactics::AABB& slab : scene.sidewalks) considerSurface(slab);
  for (const tactics::WalkSurface& surface : scene.walkSurfaces) {
    if (surface.vertices.size() < 3) continue;
    const glm::vec3 normal = glm::cross(surface.vertices[1] - surface.vertices[0],
                                        surface.vertices[2] - surface.vertices[0]);
    const float denominator = glm::dot(normal, ray.direction);
    if (std::fabs(denominator) < 1e-7f) continue;
    const float t = glm::dot(normal, surface.vertices[0] - ray.origin) / denominator;
    if (t < 0.0f || t >= bestT) continue;
    const glm::vec3 hit = ray.origin + ray.direction * t;
    if (!tactics::SurfaceContainsXZ(surface, hit.x, hit.z)) continue;
    found = true;
    bestT = t;
    bestPoint = hit;
  }

  if (found && outPoint) *outPoint = bestPoint;
  return found;
}

using ui::PaneRect;

// One pane fills the window; two split it down the middle (pane 0 left).
PaneRect ComputePaneRect(int pane, int paneCount, int windowWidth) {
  if (paneCount == 1) return PaneRect{0, windowWidth};
  const int leftWidth = windowWidth / 2;
  if (pane == 0) return PaneRect{0, leftWidth};
  return PaneRect{leftWidth, windowWidth - leftWidth};
}

int PaneForX(int x, int paneCount, int windowWidth) {
  return paneCount == 1 || x < windowWidth / 2 ? 0 : 1;
}

}  // namespace

int main() {
#ifdef __EMSCRIPTEN__
  tbgwaf_bind_canvas();
#endif
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }

  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_Window* window = SDL_CreateWindow(
      "tbgwaf", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, kInitialWindowWidth,
      kInitialWindowHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (!window) {
    std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }

  SDL_GLContext glContext = SDL_GL_CreateContext(window);
  if (!glContext) {
    std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  SDL_GL_SetSwapInterval(1);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  // Our loop drains all pending SDL events before each NewFrame(), so we
  // want queued input applied immediately rather than spread across
  // multiple frames (the default trickling is meant for low-framerate apps
  // that call NewFrame() between individual OS events).
  ImGui::GetIO().ConfigInputTrickleEventQueue = false;
  ImGui_ImplSDL2_InitForOpenGL(window, glContext);
  ImGui_ImplOpenGL3_Init("#version 300 es");

  glEnable(GL_DEPTH_TEST);

  // All scene-pass GL resources (shaders, meshes, shadow map, light) live
  // in the shared SceneRenderer, so the headless visual scenario runner
  // (issue #14) draws through exactly the same code path as the app.
  gfx::SceneRenderer renderer;
  if (!renderer.Init()) return 1;
  // Issue #110 prototype: the per-unit shadow-map FOV mask (walls/roofs/deck
  // sides too) is the app default for now so deployments show it;
  // TBGWAF_FOV_SHADOW_MAP=0 restores the analytic CPU overlay.
  // TBGWAF_FOV_PROBE_HEIGHT=<units> tests ground visibility that high above
  // the surface instead of at the surface (see SceneRenderer).
  bool fovShadowMap = true;
  if (const char* fovEnv = std::getenv("TBGWAF_FOV_SHADOW_MAP")) {
    fovShadowMap = fovEnv[0] != '\0' && fovEnv[0] != '0';
  }
  if (fovShadowMap) renderer.SetFovOverlayMode(gfx::FovOverlayMode::ShadowMap);
  if (const char* probeEnv = std::getenv("TBGWAF_FOV_PROBE_HEIGHT")) {
    renderer.SetFovProbeHeight(static_cast<float>(std::atof(probeEnv)));
  }

  // One independent orbit camera per pane, so each side can freely
  // rotate/zoom its own view without affecting the other's.
  std::array<gfx::OrbitCamera, kMaxPanes> cameras;
  // Both web clients must build the same city, so the web seed stays fixed.
  // Native runs pick a random seed per launch (shown and editable on the Map
  // Select screen); TBGWAF_MAP_SEED overrides it.
  uint32_t mapSeed = 7;
#ifndef __EMSCRIPTEN__
  mapSeed = std::random_device{}();
#endif
  if (const char* seedEnv = std::getenv("TBGWAF_MAP_SEED")) {
    mapSeed = static_cast<uint32_t>(std::strtoul(seedEnv, nullptr, 10));
  }
  // TBGWAF_MAP picks the generator/urban variant; both web clients must
  // agree the same way they must agree on the seed. The urban-auto default
  // lets the seed decide whether the city gets an overpass (and which layout,
  // issue #130); TBGWAF_MAP=urban forces it off, urban-elevated forces it on;
  // urban-full forces every optional feature layer on.
  std::string mapType = "urban-auto";
  if (const char* mapEnv = std::getenv("TBGWAF_MAP")) mapType = mapEnv;
  auto makeBaseMap = [](uint32_t mapSeed, const std::string& mapType) {
    if (mapType == "hilly") return tactics::GenerateHillyMap(mapSeed);
    tactics::MapGeneratorConfig config;
    if (mapType == "urban-merge" || mapType == "urban-elevated") config.arteryCount = 2;
    if (mapType == "urban" || mapType == "urban-merge") {
      config.elevatedHighway = tactics::OverpassMode::Off;
    }
    if (mapType == "urban-elevated") config.elevatedHighway = tactics::OverpassMode::On;
    if (mapType == "urban-full") {
      // Every optional detail layer forced on (issue #146).
      for (const char* name : {"lanes", "curbs", "furniture", "terrain", "interiors", "rail"}) {
        tactics::UrbanFeatureByName(&config.features, name)->mode = tactics::FeatureMode::On;
      }
    }
    return tactics::GenerateUrbanMap(mapSeed, config);
  };
  // Chosen on the Game Mode screen; CTF turns on the neutral flag objective.
  bool ctfMode = false;
  auto makeMap = [&mapSeed, &mapType, &ctfMode, &makeBaseMap]() {
    tactics::Scene scene = makeBaseMap(mapSeed, mapType);
    scene.flag.enabled = ctfMode;
    return scene;
  };
  GameLogic game(makeMap());
  // Start zoomed out far enough that the whole map is in view.
  for (auto& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);

  // Turn timeline (issue #143): this client's locally recorded match history
  // plus the view-only replay controller. One shared pair serves both panes
  // of a native window (they share the one GameLogic, so they share the one
  // history; either pane's strip drives the same replay), while each web
  // client instance records and replays its own -- replay never posts to the
  // bus, so the Blue-authority/Red-mirror protocol is untouched.
  tactics::TurnTimeline timeline;
  timeline.Reset(game);
  tactics::TimelinePlayback timelinePlayback;

  // Navmesh boundary debug overlay ('N' toggles it): a full-map navmesh
  // built on first use (GameLogic's own is windowed per move, so the debug
  // view meshes the whole scene itself).
  bool showNavMeshDebug = false;
  bool navMeshDebugBuilt = false;
  tactics::NavMesh navMeshDebug;

  // Which team this client instance plays (web two-canvas mode); nullopt =
  // one window showing both teams.
  std::optional<Team> fixedTeam;
  // Until the simulator answers our hello (or the timeout passes), our fresh
  // local state may be stale relative to a match already in progress, so a
  // follower doesn't act on it.
  bool awaitingPeerSync = false;
  Uint32 peerSyncDeadline = 0;
#ifdef __EMSCRIPTEN__
  if (const int requested = tbgwaf_requested_player(); requested >= 0) {
    fixedTeam = requested == 0 ? Team::Blue : Team::Red;
    tbgwaf_channel_open();
    tbgwaf_channel_post("H");
    awaitingPeerSync = *fixedTeam != Team::Blue;
    peerSyncDeadline = SDL_GetTicks() + kPeerSyncTimeoutMs;
  }
#endif
  // Server mode: this client plays against a remote opponent through the
  // authoritative server (src/server). The local GameLogic is only a
  // presentation mirror driven by net::RemoteClient; our team is assigned by
  // the server once an opponent has joined.
  std::unique_ptr<tactics::net::RemoteClient> remote;
#ifdef __EMSCRIPTEN__
  {
    char* urlRaw = tbgwaf_server_url();
    const std::string serverUrl(urlRaw);
    std::free(urlRaw);
    if (!serverUrl.empty()) {
      char* roomRaw = tbgwaf_server_room();
      remote = std::make_unique<tactics::net::RemoteClient>(&game, roomRaw);
      std::free(roomRaw);
      fixedTeam = Team::Blue;  // Placeholder until the server's "start".
      tbgwaf_ws_open(serverUrl.c_str());
    }
  }
#endif
  const bool remoteMode = remote != nullptr;
  const bool networked = fixedTeam.has_value();
  // The Blue instance (or the lone native window) simulates rounds; the Red
  // instance mirrors it during execution. A server-mode client never
  // simulates: the server is the only authority.
  const bool isSimulator = !remoteMode && (!networked || *fixedTeam == Team::Blue);
  const int paneCount = networked ? 1 : kMaxPanes;
  auto paneTeam = [&](int pane) {
    if (networked) return *fixedTeam;
    return pane == 0 ? Team::Blue : Team::Red;
  };
  std::string lastSentState;
  bool forceBroadcast = false;

  bool quit = false;
  constexpr float kClickDragThresholdPx = 5.0f;
  constexpr Uint32 kDoubleClickMs = 400;
  int lastClickUnitId = -1;  // Figure hit by the previous click, for double-click detection.
  int lastClickPane = -1;
  Uint32 lastClickTicks = 0;
  int leftDragPane = -1;  // -1 = not dragging; else the pane a left-drag (pan) started in.
  int aimedUnitId = -1;  // Unit whose planned-move ghost is being dragged, or -1.
  int aimedPane = -1;    // Pane the ghost grab started in.
  float leftDragDistance = 0.0f;  // Accumulated pixels moved during the current left-drag.
  int rightDragPane = -1;  // -1 = not dragging; else the pane a right-drag started in.
  glm::vec3 hoveredGroundPoint(0.0f);
  bool hasHoveredGroundPoint = false;

  // Headless CI (xvfb) has no real user input; bound the run so the smoke
  // test still exits on its own after exercising a few frames.
  const bool isSmokeTest = std::getenv("TBGWAF_SMOKE_TEST") != nullptr;
  int frameCount = 0;
  constexpr int kSmokeTestMaxFrames = 60;
  Uint32 lastFrameTicks = SDL_GetTicks();
  // App-level screen (Splash -> Map Select -> gameplay), generated from
  // flow/app_flow.yaml.
  tbgwaf_flow::State screen = tbgwaf_flow::kInitialState;
  // Local-only debug panel state (defaults: FOV and shadows on, FPS shown).
  gfx::RenderDebugOptions debugOptions;
  bool showFps = true;
  float smoothedFps = 0.0f;
  float frameMs = 0.0f;
  float fovMs = 0.0f;
  gfx::RenderFrameStats frameStats;

  auto runFrame = [&]() {
#ifdef __EMSCRIPTEN__
    // Track our own canvas element's CSS size (half the page per canvas).
    if (networked) {
      const double cssW = tbgwaf_canvas_css_width();
      const double cssH = tbgwaf_canvas_css_height();
      int curW = 0, curH = 0;
      SDL_GetWindowSize(window, &curW, &curH);
      if (cssW >= 1.0 && cssH >= 1.0 &&
          (curW != static_cast<int>(cssW) || curH != static_cast<int>(cssH))) {
        SDL_SetWindowSize(window, static_cast<int>(cssW), static_cast<int>(cssH));
      }
    }
#endif
    int windowWidth = kInitialWindowWidth, windowHeight = kInitialWindowHeight;
    SDL_GetWindowSize(window, &windowWidth, &windowHeight);

    // --- Sync: apply the peer's state / answer its requests. ---
    auto isPlanningMode = [](InputMode mode) {
      return mode != InputMode::Executing && mode != InputMode::GameOver;
    };
#ifdef __EMSCRIPTEN__
    if (remoteMode) {
      while (char* raw = tbgwaf_ws_next()) {
        const std::string event(raw);
        std::free(raw);
        if (event == "O") remote->OnOpen();
        else if (event == "C") remote->OnClose();
        else if (event.size() > 1 && event[0] == 'M') remote->OnMessage(event.substr(1));
      }
      if (remote->team()) fixedTeam = remote->team();
      if (remote->ConsumeMatchStarted()) {
        timeline.Reset(game);
        timelinePlayback.Reset();
        for (auto& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);
      }
    } else if (networked) {
      while (char* raw = tbgwaf_channel_next()) {
        const std::string msg(raw);
        std::free(raw);
        if (msg == "H") {
          forceBroadcast = true;  // A peer just opened: hand it our state.
        } else if (msg == "C") {
          if (isSimulator && game.CanCommitRound()) game.CommitRound();
        } else if (msg == "N") {
          if (isSimulator && game.Mode() == InputMode::GameOver) {
            game.Reset(makeMap());
            timeline.Reset(game);
            timelinePlayback.Reset();
            navMeshDebugBuilt = false;
            showNavMeshDebug = false;
          }
        } else if (msg.size() > 2 && (msg[0] == 'S' || msg[0] == 'P') && msg[1] == ' ') {
          GameSnapshot snap;
          if (!DeserializeSnapshot(msg.substr(2), &snap)) continue;
          if (msg[0] == 'S' && !isSimulator) {
            // Simulator state. Mid-planning of the same round we only learn
            // the simulator's team's plans (keeping our own plans and
            // selection); otherwise (execution, game over, a new round) we
            // take its whole state.
            const bool samePlanning = isPlanningMode(snap.mode) &&
                                      isPlanningMode(game.Mode()) &&
                                      snap.roundNumber == game.RoundNumber();
            const bool ok = samePlanning ? game.ImportTeamPlans(snap, Team::Blue)
                                         : game.ImportState(snap);
            if (ok) awaitingPeerSync = false;
          } else if (msg[0] == 'P' && isSimulator) {
            // The follower's plans for its own team, only meaningful while
            // we're planning the same round.
            if (isPlanningMode(game.Mode()) && snap.roundNumber == game.RoundNumber()) {
              game.ImportTeamPlans(snap, Team::Red);
            }
          }
        }
      }
      if (awaitingPeerSync && SDL_GetTicks() >= peerSyncDeadline) awaitingPeerSync = false;
    }
#endif

    std::array<PaneRect, kMaxPanes> paneRects;
    for (int pane = 0; pane < paneCount; ++pane) {
      paneRects[pane] = ComputePaneRect(pane, paneCount, windowWidth);
    }

    // Mouse-driven game input (world picking) must not be dispatched until
    // io.WantCaptureMouse reflects the UI actually built *this* frame:
    // checking it during event polling (before ImGui::NewFrame()/the UI
    // pass) reads a stale value from the previous frame, letting clicks
    // meant for an ImGui button leak into world-space picking (or vice
    // versa). So we just record raw input here and dispatch it below, after
    // the UI for this frame has been built.
    bool leftClickPending = false;
    int leftClickX = 0, leftClickY = 0;
    bool escapePending = false;
    bool enterPending = false;
    int mouseX = 0, mouseY = 0;
    SDL_GetMouseState(&mouseX, &mouseY);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);

      if (event.type == SDL_QUIT) {
        quit = true;
      } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_RIGHT) {
        rightDragPane = PaneForX(event.button.x, paneCount, windowWidth);
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_RIGHT) {
        rightDragPane = -1;
      } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
        leftDragPane = PaneForX(event.button.x, paneCount, windowWidth);
        leftDragDistance = 0.0f;
        // Grabbing a planned move's wireframe ghost re-aims its final facing
        // (instead of panning) until the button is released.
        aimedUnitId = -1;
        aimedPane = -1;
        if (isPlanningMode(game.Mode()) && !awaitingPeerSync && !timelinePlayback.Active() &&
            !ImGui::GetIO().WantCaptureMouse) {
          const PaneRect& grabRect = paneRects[leftDragPane];
          const Team grabTeam = paneTeam(leftDragPane);
          const gfx::Ray grabRay = cameras[leftDragPane].ScreenPointToRay(
              static_cast<float>(event.button.x - grabRect.x), static_cast<float>(event.button.y),
              static_cast<float>(grabRect.width), static_cast<float>(windowHeight));
          if (std::fabs(grabRay.direction.y) > 1e-4f) {
            constexpr float kGrabRadius = 0.7f;
            for (const Unit& unit : game.GetScene().units) {
              if (!unit.alive || unit.team != grabTeam ||
                  unit.plan.type != tactics::PlannedActionType::Move || unit.plan.movePath.empty()) {
                continue;
              }
              const glm::vec3 dest = unit.plan.movePath.back();
              const float t = (dest.y + 0.9f - grabRay.origin.y) / grabRay.direction.y;
              if (t <= 0.0f) continue;
              const glm::vec3 hit = grabRay.origin + grabRay.direction * t;
              if (glm::length(glm::vec2(hit.x - dest.x, hit.z - dest.z)) <= kGrabRadius) {
                aimedUnitId = unit.id;
                aimedPane = leftDragPane;
                break;
              }
            }
          }
        }
      } else if (event.type == SDL_MOUSEMOTION) {
        mouseX = event.motion.x;
        mouseY = event.motion.y;
        // While a ghost is grabbed, left-drag aims it (handled after the UI
        // pass) instead of panning the camera.
        if (leftDragPane >= 0 && !ImGui::GetIO().WantCaptureMouse) {
          if (aimedUnitId < 0) {
            constexpr float kPanSpeed = 0.0015f;
            // Drag the world under the cursor: target moves opposite to the drag.
            cameras[leftDragPane].Pan(-event.motion.xrel * kPanSpeed, event.motion.yrel * kPanSpeed);
          }
          leftDragDistance += std::hypot(static_cast<float>(event.motion.xrel),
                                          static_cast<float>(event.motion.yrel));
        }
        if (rightDragPane >= 0 && !ImGui::GetIO().WantCaptureMouse) {
          constexpr float kRotateSpeed = 0.005f;
          cameras[rightDragPane].Rotate(-event.motion.xrel * kRotateSpeed,
                                         event.motion.yrel * kRotateSpeed);
        }
      } else if (event.type == SDL_MOUSEWHEEL) {
        if (!ImGui::GetIO().WantCaptureMouse) {
          constexpr float kZoomSpeed = 0.625f;  // Fraction of current distance per tick.
          // preciseY carries fractional trackpad deltas; wheel.y is rounded to
          // whole ticks, which makes trackpad zoom steppy.
#if SDL_VERSION_ATLEAST(2, 0, 18)
          const float wheelY = event.wheel.preciseY;
#else
          const float wheelY = static_cast<float>(event.wheel.y);
#endif
          gfx::OrbitCamera& camera = cameras[PaneForX(mouseX, paneCount, windowWidth)];
          camera.Zoom(-wheelY * kZoomSpeed * camera.TargetDistance());
        }
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
        // A drag that panned the camera must not also fire a click.
        if (leftDragDistance < kClickDragThresholdPx) leftClickPending = true;
        leftDragPane = -1;
        leftDragDistance = 0.0f;
        aimedUnitId = -1;
        aimedPane = -1;
        leftClickX = event.button.x;
        leftClickY = event.button.y;
      } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
        escapePending = true;
      } else if (event.type == SDL_KEYDOWN && (event.key.keysym.sym == SDLK_RETURN ||
                                               event.key.keysym.sym == SDLK_KP_ENTER)) {
        enterPending = true;
      } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_n &&
                 !ImGui::GetIO().WantCaptureKeyboard) {
        showNavMeshDebug = !showNavMeshDebug;
        if (showNavMeshDebug && !navMeshDebugBuilt) {
          navMeshDebug.Build(game.GetScene().obstacles, game.GetScene().mapHalfExtent,
                             tactics::constants::kAgentRadius, &game.GetScene().ground,
                             &game.GetScene().walkSurfaces);
          navMeshDebugBuilt = true;
        }
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    if (screen != tbgwaf_flow::State::Gameplay) {
      std::optional<tbgwaf_flow::Event> flowEvent =
          tactics::ui::DrawMenu(screen, windowWidth, windowHeight, &mapSeed);
      // Headless smoke run: walk the menu automatically so gameplay is exercised.
      if (isSmokeTest && !flowEvent) {
        switch (screen) {
          case tbgwaf_flow::State::Splash: flowEvent = tbgwaf_flow::Event::NewGame; break;
          case tbgwaf_flow::State::GameMode: flowEvent = tbgwaf_flow::Event::SelectRegular; break;
          default: flowEvent = tbgwaf_flow::Event::SelectUrban; break;
        }
      }
      if (flowEvent) {
        if (*flowEvent == tbgwaf_flow::Event::SelectRegular ||
            *flowEvent == tbgwaf_flow::Event::SelectCtf) {
          ctfMode = *flowEvent == tbgwaf_flow::Event::SelectCtf;
        }
        if (*flowEvent == tbgwaf_flow::Event::SelectUrban ||
            *flowEvent == tbgwaf_flow::Event::SelectHills) {
          // The menu's Urban entry uses the seed-driven random overpass mode.
          mapType = *flowEvent == tbgwaf_flow::Event::SelectHills ? "hilly" : "urban-auto";
          game.Reset(makeMap());
          timeline.Reset(game);
          timelinePlayback.Reset();
          for (auto& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);
        }
        screen = tbgwaf_flow::Next(screen, *flowEvent);
        lastFrameTicks = SDL_GetTicks();
      }
      glViewport(0, 0, windowWidth, windowHeight);
      glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      SDL_GL_SwapWindow(window);
      return;
    }

    const Uint32 nowTicks = SDL_GetTicks();
    const float dt = static_cast<float>(nowTicks - lastFrameTicks) / 1000.0f;
    lastFrameTicks = nowTicks;
    frameMs = dt * 1000.0f;
    if (dt > 0.0f) {
      const float instantFps = 1.0f / dt;
      smoothedFps = smoothedFps == 0.0f ? instantFps : smoothedFps + (instantFps - smoothedFps) * 0.05f;
    }
    // A follower mirrors execution from the simulator's snapshots; running
    // its own (empty) round would end it immediately.
    if (isSimulator || game.Mode() != InputMode::Executing) game.Update(dt);
    // Sighting memory is per-page derived state: tick it unconditionally so
    // a follower (which skips Update() while Executing) still builds it.
    if (remoteMode) remote->Update(dt);
    game.UpdateSightingMemory(dt);
    for (auto& camera : cameras) camera.Update(dt);

    // Turn timeline (issue #143): record the local game's history and
    // advance any replay in progress. While a replay is viewed, the panes
    // render `displayGame` (the replay instance) and game-action input is
    // inert; the live game keeps simulating/syncing untouched underneath.
    if (timeline.Observe(game, dt)) timelinePlayback.Reset();
    timelinePlayback.Update(timeline, dt);
    const bool replayActive = timelinePlayback.Active();
    const GameLogic& displayGame = replayActive ? timelinePlayback.Game() : game;

    // WEGO rounds: both teams plan simultaneously, so during the planning
    // phase *both* panes accept game-action input (unit selection,
    // move/shoot targeting), each acting only for its own team --
    // GameLogic's team-tagged input calls enforce that a pane can never
    // plan the other side's figures. While a committed round executes (and
    // after game over) neither pane takes action input. Camera
    // orbit/zoom/pan is never gated -- either player can look around their
    // own pane at any time.
    const bool remotePlaying =
        !remoteMode || remote->status() == tactics::net::RemoteClient::Status::Playing;
    const bool planning =
        !replayActive && isPlanningMode(game.Mode()) && !awaitingPeerSync && remotePlaying;
    // The HUD reflects whatever state is on screen (replay or live); its
    // button presses are only applied to the live game when no replay is
    // active (replay is strictly view-only).
    const bool hudPlanning = isPlanningMode(displayGame.Mode()) && !awaitingPeerSync && remotePlaying;
    const bool fogActive = displayGame.Mode() != InputMode::GameOver;
    // The team whose plan the shared selection/preview overlays currently
    // belong to (only one figure is ever mid-selection at a time).
    std::optional<Team> selectedTeam;
    if (const auto selectedId = game.SelectedUnitId()) {
      if (const Unit* selected = game.FindUnit(*selectedId)) selectedTeam = selected->team;
    }

    std::array<TeamVisibility, kMaxPanes> paneVisibility;
    const Uint64 fovStart = SDL_GetPerformanceCounter();
    for (int pane = 0; pane < paneCount; ++pane) {
      if (fogActive) paneVisibility[pane] = displayGame.ComputeVisibility(paneTeam(pane));
    }
    fovMs = static_cast<float>(SDL_GetPerformanceCounter() - fovStart) * 1000.0f /
            static_cast<float>(SDL_GetPerformanceFrequency());

    // --- UI ---
    float roundPanelBottom = 2.0f;
    for (int pane = 0; pane < paneCount; ++pane) {
      const ui::HudActions hud = ui::DrawHud(displayGame, paneTeam(pane), hudPlanning,
                                             paneRects[pane], windowHeight, cameras[pane],
                                             nullptr, pane == 0 ? &roundPanelBottom : nullptr);
      // Replay is view-only: while it is showing, HUD presses must not leak
      // into the live match (return to Live first).
      if (!replayActive) {
        if (hud.newMatch) {
          if (remoteMode) {
            remote->RequestNewMatch();
          } else if (isSimulator) {
            game.Reset(makeMap());
            timeline.Reset(game);
            timelinePlayback.Reset();
            navMeshDebugBuilt = false;
            showNavMeshDebug = false;
          } else {
#ifdef __EMSCRIPTEN__
            tbgwaf_channel_post("N");
#endif
          }
        }
        if (hud.commit) {
          if (remoteMode) {
            remote->RequestCommit();
          } else if (isSimulator) {
            game.CommitRound();
          } else {
#ifdef __EMSCRIPTEN__
            // Send our latest plans first so the simulator commits with them.
            lastSentState = SerializeSnapshot(game.ExportState());
            tbgwaf_channel_post(("P " + lastSentState).c_str());
            tbgwaf_channel_post("C");
#endif
          }
        }
        if (hud.move) game.ChooseMove();
        if (hud.shoot) game.ChooseShoot();
        if (hud.pass) game.ChoosePass();
        if (hud.cancel) game.CancelAction();
        if (hud.playbook) game.SetPlaybook(paneTeam(pane), *hud.playbook);
        if (hud.done) game.FinishMovePlan();
        if (hud.fire) game.ConfirmAim(paneTeam(pane));
        if (hud.shots) game.SetPlannedShotCount(*hud.shots, paneTeam(pane));
      }

      // Turn timeline strip (issue #143). Both panes drive the same shared
      // playback, so scrubbing in either shows the replay in both (each
      // through its own team's fog).
      const ui::TimelineActions timelineUi = ui::DrawTimeline(
          timeline, timelinePlayback, paneTeam(pane), paneRects[pane], windowHeight);
      if (timelineUi.seekTick) timelinePlayback.SeekTick(timeline, game, *timelineUi.seekTick);
      if (timelineUi.seekFrame) timelinePlayback.SeekFrame(timeline, game, *timelineUi.seekFrame);
      if (timelineUi.togglePlay) timelinePlayback.TogglePlay(timeline, game);
      if (timelineUi.live) timelinePlayback.Deactivate();
    }

    // Pane divider. Both teams plan at once, so there's no "inactive side"
    // to dim -- each pane is always its own player's live view.
    if (paneCount == 2) {
      ImGui::GetForegroundDrawList()->AddLine(
          ImVec2(static_cast<float>(paneRects[1].x), 0.0f),
          ImVec2(static_cast<float>(paneRects[1].x), static_cast<float>(windowHeight)),
          IM_COL32(255, 255, 255, 60), 2.0f);
    }

    if (remoteMode) {
      const std::string statusText = remote->StatusText();
      if (!statusText.empty()) {
        ImGui::SetNextWindowPos(
            ImVec2(static_cast<float>(paneRects[0].x + paneRects[0].width / 2),
                   static_cast<float>(windowHeight) - 12.0f),
            ImGuiCond_Always, ImVec2(0.5f, 1.0f));
        ImGui::SetNextWindowBgAlpha(0.7f);
        ImGui::Begin("##netstatus", nullptr,
                     ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
                         ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoSavedSettings);
        ImGui::TextUnformatted(statusText.c_str());
        ImGui::End();
      }
    }

    // --- Dispatch deferred input, now that WantCaptureMouse reflects the UI
    // actually built this frame. ---
    if (escapePending && !replayActive) game.CancelAction();
    if (enterPending && !replayActive) game.FinishMovePlan();

    const bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse;
    if (!uiWantsMouse && planning) {
      const int hoverPane = PaneForX(mouseX, paneCount, windowWidth);
      if (game.Mode() == InputMode::AwaitingShootTarget) {
        // The preview cone follows the cursor until an aim point is placed.
        const PaneRect& rect = paneRects[hoverPane];
        const gfx::Ray hoverRay = cameras[hoverPane].ScreenPointToRay(
            static_cast<float>(mouseX - rect.x), static_cast<float>(mouseY),
            static_cast<float>(rect.width), static_cast<float>(windowHeight));
        glm::vec3 hoverPoint;
        if (IntersectGroundOrClimbTop(hoverRay, game.GetScene(), &hoverPoint)) {
          hoveredGroundPoint = hoverPoint;
          hasHoveredGroundPoint = true;
        }
      } else if (game.Mode() == InputMode::AwaitingMoveDestination) {
        const PaneRect& rect = paneRects[hoverPane];
        const gfx::Ray hoverRay = cameras[hoverPane].ScreenPointToRay(
            static_cast<float>(mouseX - rect.x), static_cast<float>(mouseY),
            static_cast<float>(rect.width), static_cast<float>(windowHeight));
        glm::vec3 hoverPoint;
        if (IntersectGroundOrClimbTop(hoverRay, game.GetScene(), &hoverPoint)) {
          hoveredGroundPoint = hoverPoint;
          hasHoveredGroundPoint = true;
          // Team-tagged: hovering over the *other* player's pane just clears
          // the preview instead of steering this pane's selected mover.
          game.HoverGround(hoverPoint, paneTeam(hoverPane));
        }
      }
      if (aimedUnitId >= 0 && leftDragPane >= 0 && leftDragDistance >= kClickDragThresholdPx) {
        // Aim the ghost at the cursor's point on the destination's plane.
        const Unit* aimed = game.FindUnit(aimedUnitId);
        const gfx::Ray aimRay =
            cameras[aimedPane].ScreenPointToRay(
                static_cast<float>(mouseX - paneRects[aimedPane].x), static_cast<float>(mouseY),
                static_cast<float>(paneRects[aimedPane].width), static_cast<float>(windowHeight));
        if (aimed && !aimed->plan.movePath.empty() && std::fabs(aimRay.direction.y) > 1e-4f) {
          const glm::vec3 dest = aimed->plan.movePath.back();
          const float t = (dest.y - aimRay.origin.y) / aimRay.direction.y;
          if (t > 0.0f) {
            const glm::vec3 hit = aimRay.origin + aimRay.direction * t;
            if (glm::length(glm::vec2(hit.x - dest.x, hit.z - dest.z)) > 0.1f) {
              game.SetPlannedMoveFacing(aimedUnitId, std::atan2(hit.z - dest.z, hit.x - dest.x),
                                       paneTeam(aimedPane));
            }
          }
        }
      }
      if (leftClickPending) {
        const int clickPane = PaneForX(leftClickX, paneCount, windowWidth);
        const PaneRect& rect = paneRects[clickPane];
        const gfx::Ray clickRay = cameras[clickPane].ScreenPointToRay(
            static_cast<float>(leftClickX - rect.x), static_cast<float>(leftClickY),
            static_cast<float>(rect.width), static_cast<float>(windowHeight));
        // Only figures actually rendered on this pane this frame (own
        // team, or enemies currently inside this team's FOV) are pickable
        // -- a hidden enemy's collision box must not be clickable just
        // because it happens to sit behind something that is drawn.
        const Team clickTeam = paneTeam(clickPane);
        std::vector<Unit> pickableUnits;
        for (const Unit& unit : game.GetScene().units) {
          if (unit.alive && gfx::IsUnitVisibleForRender(unit, clickTeam, fogActive,
                                                          paneVisibility[clickPane])) {
            pickableUnits.push_back(unit);
          }
        }
        const int hitUnit = PickUnit(clickRay, pickableUnits);
        const Uint32 clickTicks = SDL_GetTicks();
        const bool doubleClick = hitUnit >= 0 && hitUnit == lastClickUnitId &&
                                 clickPane == lastClickPane &&
                                 clickTicks - lastClickTicks <= kDoubleClickMs;
        lastClickUnitId = doubleClick ? -1 : hitUnit;
        lastClickPane = clickPane;
        lastClickTicks = clickTicks;
        if (game.Mode() == InputMode::AwaitingShootTarget) {
          // Aiming: every click goes through the aim ray, which picks a
          // figure first (staging the lock-on; the shot-level bar stays up
          // and Fire commits) and otherwise places/moves the free-aim "+"
          // selector on a surface in the shooter's 360-degree LOS.
          game.ClickAimRay(clickRay.origin, clickRay.direction, clickTeam);
        } else if (hitUnit >= 0) {
          game.ClickUnit(hitUnit, clickTeam);
          if (doubleClick) {
            // Drone-style focus: frame the unit's whole movement frontier
            // (same reach as the reach-field query window).
            for (const Unit& unit : game.GetScene().units) {
              if (unit.id != hitUnit) continue;
              cameras[clickPane].FocusOn(
                  unit.position, unit.MoveBudget() + 2.0f * tactics::constants::kAgentRadius);
              break;
            }
          }
        } else if (game.Mode() == InputMode::AwaitingMoveDestination) {
          glm::vec3 point;
          if (IntersectGroundOrClimbTop(clickRay, game.GetScene(), &point)) {
            game.ClickGround(point, clickTeam);
          }
        }
      }
    }
    // Anchored under the Round panel so it follows collapse/expand.
    ui::DrawDebugPanel(debugOptions.disableFov, debugOptions.disableShadows,
                       debugOptions.disableOcclusionFade, showFps,
                       smoothedFps, frameMs, fovMs, frameStats,
                       roundPanelBottom + 6.0f);

    // --- Render: one shadow pass + one color pass per pane, both inside
    // SceneRenderer::RenderPane. Selection/move-preview overlays belong to
    // whichever team the currently selected figure is on, so only that
    // player's pane shows them (the other side must not see enemy plans). ---
    // The panel above shows last frame's render counters (drawn before this
    // frame's render pass).
    frameStats = {};
    debugOptions.stats = &frameStats;
    for (int pane = 0; pane < paneCount; ++pane) {
      const PaneRect& rect = paneRects[pane];
      std::optional<glm::vec3> hover;
      // No hover marker over a replay: it belongs to live move planning.
      if (hasHoveredGroundPoint && !replayActive) hover = hoveredGroundPoint;
      gfx::PaneOverlays overlays = gfx::BuildPaneOverlays(displayGame, paneTeam(pane), hover);
      if (showNavMeshDebug) overlays.navMeshDebug = &navMeshDebug;
      renderer.RenderPane(displayGame, paneTeam(pane), fogActive, paneVisibility[pane],
                          cameras[pane], rect.x, 0, rect.width, windowHeight, overlays, 0,
                          debugOptions);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(window);

#ifdef __EMSCRIPTEN__
    // Publish state changes: the simulator ships the whole match, the
    // follower just its plans (only while planning).
    if (remoteMode) {
      for (const std::string& frame : remote->TakeOutgoing()) tbgwaf_ws_send(frame.c_str());
    } else if (networked && !awaitingPeerSync && (isSimulator || isPlanningMode(game.Mode()))) {
      std::string state = SerializeSnapshot(game.ExportState());
      if (forceBroadcast || state != lastSentState) {
        tbgwaf_channel_post(((isSimulator ? "S " : "P ") + state).c_str());
        lastSentState = std::move(state);
        forceBroadcast = false;
      }
    }
#endif

    if (isSmokeTest && ++frameCount >= kSmokeTestMaxFrames) {
      quit = true;
    }

#ifdef __EMSCRIPTEN__
    if (quit) emscripten_cancel_main_loop();
#endif
  };

#ifdef __EMSCRIPTEN__
  std::function<void()> frameFn = runFrame;
  emscripten_set_main_loop_arg(
      [](void* arg) { (*static_cast<std::function<void()>*>(arg))(); }, &frameFn, 0, 1);
#else
  while (!quit) runFrame();

  renderer.Destroy();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();
#endif
  return 0;
}
