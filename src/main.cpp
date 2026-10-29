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

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "ai/JevPlanner.h"
#include "game/BuilderScene.h"
#include "game/GameLogic.h"
#include "game/Geometry.h"
#include "game/MapGenerator.h"
#include "game/Raycast.h"
#include "game/ReproLog.h"
#include "game/TurnTimeline.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"
#include "AppFlow.h"
#include "ui/BuilderPanel.h"
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

// CSS size of this instance's own canvas (each client renders into its own
// <canvas>, so a global "#canvas" selector would not do).
EM_JS(double, tbgwaf_canvas_css_width, (), {
  return Module.canvas.getBoundingClientRect().width;
});
EM_JS(double, tbgwaf_canvas_css_height, (), {
  return Module.canvas.getBoundingClientRect().height;
});

EM_JS(int, tbgwaf_ai_enabled, (), { return Module.tbgwafAI ? 1 : 0; });
EM_JS(int, tbgwaf_ai_paused, (), { return Module.tbgwafAIPaused ? 1 : 0; });
EM_JS(int, tbgwaf_ai_fallback, (), { return Module.tbgwafAIFallback ? 1 : 0; });
EM_JS(int, tbgwaf_auto_commit, (), { return Module.tbgwafAutoCommit ? 1 : 0; });
EM_JS(int, tbgwaf_spectator, (), { return Module.tbgwafSpectator ? 1 : 0; });
EM_JS(int, tbgwaf_default_ctf, (), { return Module.tbgwafDefaultCtf ? 1 : 0; });
EM_JS(int, tbgwaf_ai_generation, (), { return Module.tbgwafAIGeneration || 0; });
EM_JS(int, tbgwaf_take_restart, (), {
  const value = Module.tbgwafRestart ? 1 : 0;
  Module.tbgwafRestart = false;
  return value;
});
EM_JS(int, tbgwaf_take_ai_retry, (), {
  const value = Module.tbgwafRetryAI ? 1 : 0;
  Module.tbgwafRetryAI = false;
  return value;
});
EM_JS(void, tbgwaf_ai_request, (const char* payload), {
  if (Module.tbgwafRequestDecision) Module.tbgwafRequestDecision(UTF8ToString(payload));
});
EM_JS(char*, tbgwaf_ai_next, (), {
  if (!Module.tbgwafAiInbox || Module.tbgwafAiInbox.length === 0) return 0;
  const msg = Module.tbgwafAiInbox.shift();
  const size = lengthBytesUTF8(msg) + 1;
  const ptr = _malloc(size);
  stringToUTF8(msg, ptr, size);
  return ptr;
});
EM_JS(void, tbgwaf_report_match, (const char* phase, int round, int winner), {
  if (Module.tbgwafReportMatch) Module.tbgwafReportMatch(UTF8ToString(phase), round, winner);
});
EM_JS(void, tbgwaf_ai_local_error, (const char* message), {
  if (Module.tbgwafReportAiError) Module.tbgwafReportAiError(UTF8ToString(message));
});
// Builder repro log download: hands the text to the browser as a file
// download.
EM_JS(void, tbgwaf_download_text, (const char* filename, const char* text), {
  const blob = new Blob([UTF8ToString(text)], {type: "application/x-ndjson"});
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  a.href = url;
  a.download = UTF8ToString(filename);
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  setTimeout(() => URL.revokeObjectURL(url), 1000);
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

// Frames `camera` on what `bs` actually contains (cubes and figures, plus a
// margin), centered on the map: a near-empty scene still shows some ground,
// a crowded one its whole map.
void FitCameraToBuilderScene(gfx::OrbitCamera* camera, const tactics::BuilderScene& bs) {
  float extent = 6.0f;
  for (const tactics::BuilderBox& box : bs.boxes) {
    extent = std::max({extent, std::fabs(box.cx) + box.hx + 2.0f, std::fabs(box.cz) + box.hz + 2.0f});
  }
  for (const tactics::BuilderFigure& figure : bs.figures) {
    extent = std::max({extent, std::fabs(figure.x) + 2.0f, std::fabs(figure.z) + 2.0f});
  }
  camera->target = glm::vec3(0.0f);
  camera->FitToExtent(std::min(extent, bs.mapHalfExtent));
  camera->Update(1.0e3f);
}

// Builder card thumbnails: each scene is rendered offscreen through the same
// SceneRenderer pass as gameplay (no fog, both teams' FOV) into a texture
// the Builder screen shows. `*texture` is created on first use and reused
// (re-rendered in place) after the scene is edited; the caller deletes it
// with the card.
constexpr int kThumbnailWidth = 320;
constexpr int kThumbnailHeight = 180;

void RenderBuilderThumbnail(gfx::SceneRenderer& renderer, const tactics::BuilderScene& bs,
                            GLuint* texture, int index) {
  if (*texture == 0) {
    glGenTextures(1, texture);
    glBindTexture(GL_TEXTURE_2D, *texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, kThumbnailWidth, kThumbnailHeight, 0, GL_RGBA,
                 GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_2D, 0);
  }
  GLuint fbo = 0, depthStencil = 0;
  // The scene pass uses depth and stencil.
  glGenRenderbuffers(1, &depthStencil);
  glBindRenderbuffer(GL_RENDERBUFFER, depthStencil);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, kThumbnailWidth, kThumbnailHeight);
  glBindRenderbuffer(GL_RENDERBUFFER, 0);
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, *texture, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER,
                            depthStencil);
  const bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  if (complete) {
    GameLogic preview(tactics::BuildScene(bs));
    preview.UpdateSightingMemory(0.0f);
    gfx::OrbitCamera camera;
    FitCameraToBuilderScene(&camera, bs);
    gfx::PaneOverlays overlays = gfx::BuildPaneOverlays(preview, Team::Blue, std::nullopt);
    overlays.spectatorFov = true;
    renderer.RenderPane(preview, Team::Blue, /*fogActive=*/false, TeamVisibility{}, camera, 0, 0,
                        kThumbnailWidth, kThumbnailHeight, overlays, fbo);
#ifndef __EMSCRIPTEN__
    // TBGWAF_THUMBNAIL_DUMP=<dir>: write each card's render as
    // <dir>/builder-<index>.ppm (debug aid for the offscreen path, which no
    // golden covers).
    if (const char* dumpDir = std::getenv("TBGWAF_THUMBNAIL_DUMP")) {
      std::vector<unsigned char> pixels(static_cast<size_t>(kThumbnailWidth) * kThumbnailHeight * 4);
      glBindFramebuffer(GL_FRAMEBUFFER, fbo);
      glReadPixels(0, 0, kThumbnailWidth, kThumbnailHeight, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      std::ofstream out(std::string(dumpDir) + "/builder-" + std::to_string(index) + ".ppm",
                        std::ios::binary);
      out << "P6\n" << kThumbnailWidth << ' ' << kThumbnailHeight << "\n255\n";
      for (int y = kThumbnailHeight - 1; y >= 0; --y) {  // GL rows are bottom-up.
        for (int x = 0; x < kThumbnailWidth; ++x) {
          const unsigned char* p = &pixels[(static_cast<size_t>(y) * kThumbnailWidth + x) * 4];
          out.write(reinterpret_cast<const char*>(p), 3);
        }
      }
    }
#else
    (void)index;
#endif
  } else {
    std::fprintf(stderr, "builder thumbnail framebuffer incomplete\n");
  }
  glDeleteFramebuffers(1, &fbo);
  glDeleteRenderbuffers(1, &depthStencil);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

// The Builder's first card, and the scene TBGWAF_SMOKE_BUILDER plays: two
// cubes (a Blue sniper perched on one), 2v2.
tactics::BuilderScene MakeSampleBuilderScene() {
  tactics::BuilderScene bs;
  bs.name = "Rooftop standoff";
  bs.boxes.push_back({-6.0f, 0.0f, 1.5f, 1.5f, 1.2f});
  bs.boxes.push_back({2.0f, -4.0f, 1.0f, 2.0f, 2.0f});
  bs.figures.push_back({Team::Blue, tactics::WeaponType::SniperRifle, -6.0f, 0.0f});
  bs.figures.push_back({Team::Blue, tactics::WeaponType::AssaultRifle, -8.0f, 5.0f});
  bs.figures.push_back({Team::Red, tactics::WeaponType::DesertEagle, 8.0f, 2.0f});
  bs.figures.push_back({Team::Red, tactics::WeaponType::AssaultRifle, 8.0f, -5.0f});
  return bs;
}

// Editor picking: the figure whose standing box (feet at its surface, ~1.8
// tall) the ray hits nearest, or -1; likewise for cubes.
int PickBuilderFigure(const gfx::Ray& ray, const tactics::BuilderScene& bs) {
  int best = -1;
  float bestT = std::numeric_limits<float>::infinity();
  for (int i = 0; i < static_cast<int>(bs.figures.size()); ++i) {
    const tactics::BuilderFigure& figure = bs.figures[i];
    const float y = tactics::SurfaceHeightAt(bs, figure.x, figure.z);
    const tactics::AABB bounds{glm::vec3(figure.x - 0.5f, y, figure.z - 0.5f),
                               glm::vec3(figure.x + 0.5f, y + 1.8f, figure.z + 0.5f)};
    float t = 0.0f;
    if (tactics::RayIntersectsAABB(ray.origin, ray.direction, bounds, &t) && t < bestT) {
      bestT = t;
      best = i;
    }
  }
  return best;
}

int PickBuilderBox(const gfx::Ray& ray, const tactics::BuilderScene& bs) {
  int best = -1;
  float bestT = std::numeric_limits<float>::infinity();
  for (int i = 0; i < static_cast<int>(bs.boxes.size()); ++i) {
    const tactics::BuilderBox& box = bs.boxes[i];
    const tactics::AABB bounds{glm::vec3(box.cx - box.hx, 0.0f, box.cz - box.hz),
                               glm::vec3(box.cx + box.hx, box.height, box.cz + box.hz)};
    float t = 0.0f;
    if (tactics::RayIntersectsAABB(ray.origin, ray.direction, bounds, &t) && t < bestT) {
      bestT = t;
      best = i;
    }
  }
  return best;
}

std::string UtcTimestamp(bool forFilename) {
  const std::time_t now = std::time(nullptr);
  char buf[32];
  std::tm tm{};
#ifdef _WIN32
  gmtime_s(&tm, &now);
#else
  gmtime_r(&now, &tm);
#endif
  std::strftime(buf, sizeof buf, forFilename ? "%Y%m%d-%H%M%S" : "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

// Hands the session's repro log to the user: a browser download on the web,
// a file (TBGWAF_REPRO_DIR, else the working directory) natively. Returns
// where it went, for the panel.
std::string DownloadReproLog(tactics::ReproRecorder& recorder, const std::string& sceneId) {
  const std::string text = recorder.Text();
  // Scene ids may carry a Builder scene's free-form name ("builder:My yard"):
  // keep the filename portable.
  std::string safeId = sceneId;
  for (char& c : safeId) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_') c = '-';
  }
  const std::string filename = "repro-" + safeId + "-" + UtcTimestamp(true) + ".jsonl";
#ifdef __EMSCRIPTEN__
  tbgwaf_download_text(filename.c_str(), text.c_str());
  return filename;
#else
  std::string path = filename;
  if (const char* dir = std::getenv("TBGWAF_REPRO_DIR")) path = std::string(dir) + "/" + filename;
  std::ofstream out(path, std::ios::binary);
  out << text;
  if (!out) {
    std::fprintf(stderr, "could not write repro log %s\n", path.c_str());
    return "";
  }
  std::fprintf(stderr, "repro log written to %s\n", path.c_str());
  return path;
#endif
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
      for (const char* name :
           {"lanes", "curbs", "furniture", "terrain", "interiors", "rail", "facades"}) {
        tactics::UrbanFeatureByName(&config.features, name)->mode = tactics::FeatureMode::On;
      }
    }
    if (const char* terrain = std::getenv("TBGWAF_TERRAIN")) {
      tactics::ParseFeatureMode(terrain, &config.features.terrain.mode);
    }
    // TBGWAF_BLOCK_CHARACTER=downtown|residential|industrial|mixed forces
    // every block's neighbourhood character (issue #151); unset/auto keeps
    // the per-block draw. Unknown values are ignored.
    if (const char* characterEnv = std::getenv("TBGWAF_BLOCK_CHARACTER")) {
      tactics::ParseBlockCharacter(characterEnv, &config.character);
    }
    return tactics::GenerateUrbanMap(mapSeed, config);
  };
  // Chosen on the Game Mode screen; CTF turns on the neutral flag objective.
  bool ctfMode = false;
#ifdef __EMSCRIPTEN__
  bool ctfFromPreview = false;
#endif
  auto makeMap = [&mapSeed, &mapType, &ctfMode, &makeBaseMap]() {
    tactics::Scene scene = makeBaseMap(mapSeed, mapType);
    tactics::ApplySpawnHints(&scene);
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

  // Builder: the player-built scenes (in memory for the session; the first
  // card is a sample), each card's thumbnail texture and whether it needs a
  // re-render, the scene open in the editor / in play, and the session
  // recorder. A builder game is played hotseat on one screen; the recorder
  // keeps its log downloadable until the next builder game starts.
  std::vector<tactics::BuilderScene> builderScenes = {MakeSampleBuilderScene()};
  std::vector<GLuint> builderThumbnails(builderScenes.size(), 0);
  std::vector<bool> builderThumbnailDirty(builderScenes.size(), true);
  int builderIndex = -1;          // Scene open in the editor / played.
  bool builderGameActive = false;  // Gameplay was entered via start_builder_game.
  std::string builderLogSceneId;   // Scene id of the recorder's current log.
  std::string builderStatus;
  tactics::ReproRecorder reproRecorder([] { return static_cast<uint64_t>(SDL_GetTicks()); });
  // The editor's own view of the scene being built: a preview game rebuilt
  // whenever the scene changes (keyed on its serialization), its camera and
  // selection, and the figure being dragged in the viewport (-1 = none).
  GameLogic builderPreview;
  std::string builderPreviewKey;
  gfx::OrbitCamera builderCamera;
  tactics::ui::BuilderEditorState builderEditor;
  int builderDragFigure = -1;
  auto addBuilderScene = [&](tactics::BuilderScene bs) {
    builderScenes.push_back(std::move(bs));
    builderThumbnails.push_back(0);
    builderThumbnailDirty.push_back(true);
    return static_cast<int>(builderScenes.size()) - 1;
  };
  auto deleteBuilderScene = [&](int index) {
    if (builderThumbnails[index] != 0) glDeleteTextures(1, &builderThumbnails[index]);
    builderScenes.erase(builderScenes.begin() + index);
    builderThumbnails.erase(builderThumbnails.begin() + index);
    builderThumbnailDirty.erase(builderThumbnailDirty.begin() + index);
    if (builderIndex == index) builderIndex = -1;
    else if (builderIndex > index) --builderIndex;
  };
  // Opens scene `index` in the editor: fresh selection, camera fit to the map.
  auto openBuilderEditor = [&](int index) {
    builderIndex = index;
    builderEditor = {};
    builderDragFigure = -1;
    builderPreviewKey.clear();  // Rebuilt on the first editor frame.
    builderCamera = gfx::OrbitCamera();
    builderCamera.FitToExtent(builderScenes[index].mapHalfExtent);
    builderCamera.Update(1.0e3f);
  };

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
  const bool networked = fixedTeam.has_value();
  // The Blue instance (or the lone native window) simulates rounds; the Red
  // instance mirrors it during execution.
  const bool isSimulator = !networked || *fixedTeam == Team::Blue;
  // Panes outside a builder game; a builder game is one hotseat pane.
  const int defaultPaneCount = networked ? 1 : kMaxPanes;
  // Builder hotseat: one screen plans for both teams, so the "pane team"
  // (whose HUD, overlays and input tags are used) follows the selected
  // figure, Blue when nothing is selected.
  auto hotseatTeam = [&]() {
    if (const auto selectedId = game.SelectedUnitId()) {
      if (const Unit* selected = game.FindUnit(*selectedId)) return selected->team;
    }
    return Team::Blue;
  };
  auto paneTeam = [&](int pane) {
    if (builderGameActive) return hotseatTeam();
    if (networked) return *fixedTeam;
    return pane == 0 ? Team::Blue : Team::Red;
  };
  std::string lastSentState;
  bool forceBroadcast = false;
#ifdef __EMSCRIPTEN__
  std::optional<tactics::ai::JevRequest> pendingJevRequest;
  // Candidates are validated a few ms per frame so planning never freezes the
  // page (and the other pane) while Jev's request is being prepared.
  std::optional<tactics::ai::JevRequestBuilder> jevBuilder;
  unsigned jevRequestNonce = 0;
  int observedAiGeneration = tbgwaf_ai_generation();
  bool jevErrorHold = false;
  int lastAutoCommittedRound = -1;
  // Seconds every figure has had a plan; the auto-commit waits a beat so both
  // teams' plan overlays are rendered before execution starts.
  float autoCommitHold = 0.0f;
  constexpr float kAutoCommitDelay = 0.75f;
  int lastReportedRound = -1;
  int lastReportedWinner = -2;
  InputMode lastReportedMode = InputMode::Executing;
#endif

  // Local-only debug panel state (defaults: FOV and shadows on, FPS shown).
  gfx::RenderDebugOptions debugOptions;
  // AO view is on by default, including in the smoke test (software GL needs
  // the longer smoke_test timeout).
  debugOptions.thermal = true;
  bool showFps = true;
  float smoothedFps = 0.0f;
  float frameMs = 0.0f;
  float fovMs = 0.0f;
  gfx::RenderFrameStats frameStats;
  // The thermal setting a builder game overrode, restored when it stops.
  std::optional<bool> thermalBeforeBuilder;

  // Starts a hotseat game on builder scene `index`: per-session reset and a
  // fresh recording (the log embeds the scene), cameras framed on it.
  auto startBuilderGame = [&](int index) {
    builderIndex = index;
    builderGameActive = true;
    const tactics::BuilderScene& bs = builderScenes[index];
    uint32_t seed = SDL_GetTicks() * 2654435761u ^ static_cast<uint32_t>(std::time(nullptr));
    if (const char* seedEnv = std::getenv("TBGWAF_MAP_SEED")) {
      seed = static_cast<uint32_t>(std::strtoul(seedEnv, nullptr, 10));
    }
    tactics::InitBuilderGame(&game, bs, seed);
    builderLogSceneId = tactics::BuilderSceneLogId(bs);
    reproRecorder.Start(game, builderLogSceneId, seed, UtcTimestamp(false),
                        tactics::SerializeBuilderScene(bs));
    game.SetInputRecorder(&reproRecorder);
    timeline.Reset(game);
    timelinePlayback.Reset();
    navMeshDebugBuilt = false;
    showNavMeshDebug = false;
    for (auto& camera : cameras) FitCameraToBuilderScene(&camera, bs);
    builderStatus.clear();
    // Builder play is local to this instance: no peer sync to wait for.
    awaitingPeerSync = false;
    // Hotseat shows both teams at once; the thermal view's monochrome would
    // make them indistinguishable, so builder games start in normal colors
    // (the debug panel can still turn it back on). Restored on Stop.
    if (!thermalBeforeBuilder) thermalBeforeBuilder = debugOptions.thermal;
    debugOptions.thermal = false;
  };
  // Ends a builder game (Stop): the recorder stops listening but keeps the
  // log for Download.
  auto stopBuilderGame = [&]() {
    game.SetInputRecorder(nullptr);
    builderGameActive = false;
    timelinePlayback.Reset();
    if (thermalBeforeBuilder) debugOptions.thermal = *thermalBeforeBuilder;
    thermalBeforeBuilder.reset();
  };
  // Leaving the Builder for the Game Mode menu: drop the builder game so it
  // is never published, and (web) rejoin the peer's match -- ask it for its
  // state; the simulator re-broadcasts ours.
  auto leaveBuilderFlow = [&]() {
    stopBuilderGame();
    game.Reset(makeMap());
    timeline.Reset(game);
    timelinePlayback.Reset();
    navMeshDebugBuilt = false;
    showNavMeshDebug = false;
    lastSentState.clear();
    forceBroadcast = isSimulator;
#ifdef __EMSCRIPTEN__
    if (networked) {
      tbgwaf_channel_post("H");
      awaitingPeerSync = !isSimulator;
      peerSyncDeadline = SDL_GetTicks() + kPeerSyncTimeoutMs;
    }
#endif
  };
#ifdef __EMSCRIPTEN__
  // Builder screens and builder games are single-instance: a networked
  // instance in them neither sends nor applies bus messages.
  auto inBuilderFlow = [&](tbgwaf_flow::State state) {
    return state == tbgwaf_flow::State::Builder || state == tbgwaf_flow::State::BuilderEdit ||
           (state == tbgwaf_flow::State::Gameplay && builderGameActive);
  };
#endif

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
  // TBGWAF_SMOKE_BUILDER=1 walks into the Builder instead (thumbnails, the
  // editor) and plays the sample scene hotseat, with a scripted round, and
  // TBGWAF_REPRO_OUT=<path> writes the recorded log on exit (ctest
  // builder_smoke_test replays it).
  const bool isSmokeBuilder = isSmokeTest && std::getenv("TBGWAF_SMOKE_BUILDER") != nullptr;
  int frameCount = 0;
  const int kSmokeTestMaxFrames = isSmokeBuilder ? 240 : 60;
  Uint32 lastFrameTicks = SDL_GetTicks();
  // App-level screen (Splash -> Map Select -> gameplay), generated from
  // flow/app_flow.yaml.
  tbgwaf_flow::State screen = tbgwaf_flow::kInitialState;
#ifndef __EMSCRIPTEN__
  // TBGWAF_REPRO_LOG=<file>: replay a recorded session (a Builder log, or a
  // registry demo-scene log) into the live game (the turn timeline records
  // it, so it can be scrubbed) and open straight on the result. View-only:
  // the loaded session is not re-recorded. A Builder log's scene is added to
  // the Builder's cards and shown hotseat, as it was played.
  if (const char* logPath = std::getenv("TBGWAF_REPRO_LOG")) {
    std::ifstream in(logPath, std::ios::binary);
    std::stringstream buffer;
    buffer << in.rdbuf();
    if (!in) {
      std::fprintf(stderr, "TBGWAF_REPRO_LOG: cannot read %s\n", logPath);
      return 1;
    }
    tactics::ReproLog parsed;
    std::string parseError;
    if (!tactics::ParseReproLog(buffer.str(), &parsed, &parseError)) {
      std::fprintf(stderr, "TBGWAF_REPRO_LOG: %s\n", parseError.c_str());
      return 1;
    }
    // Restart the timeline on the session's initial state (so its Start tick
    // is the session's): replaying the header alone sets the game up exactly
    // as the full replay below will, without applying any entry.
    tactics::ReproLog headerOnly;
    headerOnly.header = parsed.header;
    const tactics::ReplayReport setup =
        tactics::ReplayAnyLog(tactics::SerializeReproLog(headerOnly), &game);
    if (!setup.error.empty()) {
      std::fprintf(stderr, "TBGWAF_REPRO_LOG: %s\n", setup.error.c_str());
      return 1;
    }
    timeline.Reset(game);
    const tactics::ReplayReport report = tactics::ReplayAnyLog(
        buffer.str(), &game, nullptr, [&](const tactics::ReproEntry&, float dt) {
          game.UpdateSightingMemory(dt);
          timeline.Observe(game, dt);
        });
    if (!report.error.empty()) {
      std::fprintf(stderr, "TBGWAF_REPRO_LOG: %s\n", report.error.c_str());
      return 1;
    }
    timeline.Observe(game, 0.0f);
    const std::string loaded = std::string("Loaded ") + logPath + ": " +
                               std::to_string(report.mismatches.size()) + " hash mismatch(es)";
    std::fprintf(stderr, "%s\n", loaded.c_str());
    tactics::BuilderScene loadedScene;
    if (!parsed.header.builderScene.empty() &&
        tactics::ParseBuilderScene(parsed.header.builderScene, &loadedScene)) {
      builderIndex = addBuilderScene(loadedScene);
      builderGameActive = true;
      builderStatus = loaded;
      thermalBeforeBuilder = debugOptions.thermal;
      debugOptions.thermal = false;
      for (auto& camera : cameras) FitCameraToBuilderScene(&camera, loadedScene);
    } else {
      for (auto& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);
    }
    screen = tbgwaf_flow::State::Gameplay;
  }
#endif
  auto runFrame = [&]() {
#ifdef __EMSCRIPTEN__
    // A restart from the page (e.g. switching to Jev vs Jev) pulls this
    // instance out of the Builder back into the shared match.
    const bool pageRestart = networked && tbgwaf_take_restart();
    if (pageRestart && inBuilderFlow(screen)) {
      leaveBuilderFlow();
      screen = tbgwaf_flow::State::GameMode;
    }
    // Jev never plays a builder game.
    const bool aiEnabled = networked && tbgwaf_ai_enabled() && !inBuilderFlow(screen);
    const bool aiPaused = tbgwaf_ai_paused();
    const int aiGeneration = tbgwaf_ai_generation();
    if (aiGeneration != observedAiGeneration) {
      observedAiGeneration = aiGeneration;
      pendingJevRequest.reset();
      jevBuilder.reset();
      jevErrorHold = false;
    }
    if (tbgwaf_take_ai_retry()) jevErrorHold = false;
    if (pageRestart) {
      // Jev vs Jev defaults to CTF; leaving it restores the menu's choice.
      const bool previewCtf = tbgwaf_default_ctf();
      if (previewCtf) ctfMode = true;
      else if (ctfFromPreview) ctfMode = false;
      ctfFromPreview = previewCtf;
      game.Reset(makeMap());
      pendingJevRequest.reset();
      jevBuilder.reset();
      jevErrorHold = false;
      lastAutoCommittedRound = -1;
      autoCommitHold = 0.0f;
      lastSentState.clear();
      forceBroadcast = isSimulator;
      awaitingPeerSync = false;
      navMeshDebugBuilt = false;
      showNavMeshDebug = false;
    }
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
    if (networked) {
      // Builder screens/games are local: the peer's messages are drained
      // and dropped (leaving the Builder resyncs, see leaveBuilderFlow).
      const bool dropMessages = inBuilderFlow(screen);
      while (char* raw = tbgwaf_channel_next()) {
        const std::string msg(raw);
        std::free(raw);
        if (dropMessages) continue;
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

    const bool builderHotseat = screen == tbgwaf_flow::State::Gameplay && builderGameActive;
    const bool builderEditing = screen == tbgwaf_flow::State::BuilderEdit && builderIndex >= 0 &&
                                builderIndex < static_cast<int>(builderScenes.size());
    const int paneCount = builderHotseat ? 1 : defaultPaneCount;
    // The editor's 3D viewport: the window left of its side panel.
    const PaneRect editorRect{
        0, std::max(1, windowWidth - static_cast<int>(tactics::ui::BuilderPanelWidth(windowWidth)))};
    auto cameraForPane = [&](int pane) -> gfx::OrbitCamera& {
      return builderEditing ? builderCamera : cameras[pane];
    };
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
        builderDragFigure = -1;
        if (builderEditing && !ImGui::GetIO().WantCaptureMouse &&
            event.button.x < editorRect.x + editorRect.width) {
          // Editor: pressing on a figure drags it (instead of panning) until
          // the button is released.
          const gfx::Ray grabRay = builderCamera.ScreenPointToRay(
              static_cast<float>(event.button.x - editorRect.x), static_cast<float>(event.button.y),
              static_cast<float>(editorRect.width), static_cast<float>(windowHeight));
          builderDragFigure = PickBuilderFigure(grabRay, builderScenes[builderIndex]);
          if (builderDragFigure >= 0) {
            builderEditor.selectedFigure = builderDragFigure;
            builderEditor.selectedBox = -1;
          }
        }
        if (screen == tbgwaf_flow::State::Gameplay && isPlanningMode(game.Mode()) &&
            !awaitingPeerSync && !timelinePlayback.Active() &&
            !ImGui::GetIO().WantCaptureMouse) {
          const PaneRect& grabRect = paneRects[leftDragPane];
          const Team grabTeam = paneTeam(leftDragPane);
          const gfx::Ray grabRay = cameras[leftDragPane].ScreenPointToRay(
              static_cast<float>(event.button.x - grabRect.x), static_cast<float>(event.button.y),
              static_cast<float>(grabRect.width), static_cast<float>(windowHeight));
          if (std::fabs(grabRay.direction.y) > 1e-4f) {
            constexpr float kGrabRadius = 0.7f;
            for (const Unit& unit : game.GetScene().units) {
              // Hotseat: either team's ghosts can be grabbed.
              if (!unit.alive || (!builderHotseat && unit.team != grabTeam) ||
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
          if (aimedUnitId < 0 && builderDragFigure < 0) {
            constexpr float kPanSpeed = 0.0015f;
            // Drag the world under the cursor: target moves opposite to the drag.
            cameraForPane(leftDragPane)
                .Pan(-event.motion.xrel * kPanSpeed, event.motion.yrel * kPanSpeed);
          }
          leftDragDistance += std::hypot(static_cast<float>(event.motion.xrel),
                                          static_cast<float>(event.motion.yrel));
        }
        if (rightDragPane >= 0 && !ImGui::GetIO().WantCaptureMouse) {
          constexpr float kRotateSpeed = 0.005f;
          cameraForPane(rightDragPane)
              .Rotate(-event.motion.xrel * kRotateSpeed, event.motion.yrel * kRotateSpeed);
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
          gfx::OrbitCamera& camera = cameraForPane(PaneForX(mouseX, paneCount, windowWidth));
          camera.Zoom(-wheelY * kZoomSpeed * camera.TargetDistance());
        }
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
        // A drag that panned the camera must not also fire a click.
        if (leftDragDistance < kClickDragThresholdPx) leftClickPending = true;
        leftDragPane = -1;
        leftDragDistance = 0.0f;
        aimedUnitId = -1;
        aimedPane = -1;
        builderDragFigure = -1;
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

    // --- Builder editor (builder_edit): the scene being built, rendered
    // full-height left of the side panel, with its own camera. ---
    if (screen == tbgwaf_flow::State::BuilderEdit && !builderEditing) {
      screen = tbgwaf_flow::Next(screen, tbgwaf_flow::Event::BackToBuilder);  // Scene gone.
    }
    if (builderEditing) {
      tactics::BuilderScene& bs = builderScenes[builderIndex];
      const Uint32 nowTicks = SDL_GetTicks();
      builderCamera.Update(static_cast<float>(nowTicks - lastFrameTicks) / 1000.0f);
      lastFrameTicks = nowTicks;

      const tactics::ui::BuilderEditorActions editor =
          tactics::ui::DrawBuilderEditor(&bs, &builderEditor, windowWidth, windowHeight);

      // Deferred viewport input, now that WantCaptureMouse reflects this
      // frame's UI. Dragging a figure follows the walkable surface under the
      // cursor (ground or a cube top) and stays snapped onto it.
      auto editorRay = [&](int x, int y) {
        return builderCamera.ScreenPointToRay(
            static_cast<float>(x - editorRect.x), static_cast<float>(y),
            static_cast<float>(editorRect.width), static_cast<float>(windowHeight));
      };
      const bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse;
      if (builderDragFigure >= 0 && builderDragFigure < static_cast<int>(bs.figures.size()) &&
          leftDragDistance >= kClickDragThresholdPx) {
        glm::vec3 point;
        if (IntersectGroundOrClimbTop(editorRay(mouseX, mouseY), builderPreview.GetScene(),
                                      &point)) {
          tactics::ui::PlaceBuilderFigure(&bs, builderDragFigure, point.x, point.z);
        }
      }
      if (leftClickPending && !uiWantsMouse && leftClickX < editorRect.x + editorRect.width) {
        // Click: select the figure or cube under the cursor (nothing = clear).
        const gfx::Ray ray = editorRay(leftClickX, leftClickY);
        builderEditor.selectedFigure = PickBuilderFigure(ray, bs);
        builderEditor.selectedBox =
            builderEditor.selectedFigure >= 0 ? -1 : PickBuilderBox(ray, bs);
      }
      if (escapePending && !ImGui::GetIO().WantCaptureKeyboard) builderEditor = {};

      // Headless smoke run: start the scene as soon as the editor has shown.
      const bool start = editor.start || isSmokeBuilder;
      if (start || editor.back) {
        builderThumbnailDirty[builderIndex] = true;  // Its card shows the edits.
        builderDragFigure = -1;
      }
      if (start) {
        if (const std::vector<std::string> problems = tactics::ui::BuilderSceneProblems(bs);
            !problems.empty()) {
          std::fprintf(stderr, "builder scene not startable: %s\n", problems.front().c_str());
        }
        startBuilderGame(builderIndex);
        screen = tbgwaf_flow::Next(screen, tbgwaf_flow::Event::StartBuilderGame);
        lastFrameTicks = SDL_GetTicks();
      } else if (editor.back) {
        screen = tbgwaf_flow::Next(screen, tbgwaf_flow::Event::BackToBuilder);
      }

      // Rebuild the preview whenever the scene changed (cheap: a flat map).
      const std::string key = tactics::SerializeBuilderScene(bs);
      if (key != builderPreviewKey) {
        builderPreview.Reset(tactics::BuildScene(bs));
        builderPreview.UpdateSightingMemory(0.0f);
        builderPreviewKey = key;
      }
      tactics::ui::DrawBuilderViewportOverlay(bs, builderEditor, builderCamera, editorRect,
                                              windowHeight);
      gfx::PaneOverlays overlays;
      overlays.spectatorFov = true;  // Both teams' FOV, as the hotseat game shows them.
      if (const Unit* selected = builderPreview.FindUnit(builderEditor.selectedFigure)) {
        overlays.selectionHighlight = selected->position;
      }
      // Normal colors (not the thermal view) so the teams read apart.
      gfx::RenderDebugOptions editorOptions = debugOptions;
      editorOptions.thermal = false;
      editorOptions.stats = nullptr;
      glViewport(0, 0, windowWidth, windowHeight);
      glClearColor(0.08f, 0.09f, 0.11f, 1.0f);
      glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
      renderer.RenderPane(builderPreview, Team::Blue, /*fogActive=*/false, TeamVisibility{},
                          builderCamera, editorRect.x, 0, editorRect.width, windowHeight, overlays,
                          0, editorOptions);
      ImGui::Render();
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
      SDL_GL_SwapWindow(window);
      return;
    }

    if (screen != tbgwaf_flow::State::Gameplay) {
      // Builder cards: (re-)render the thumbnails of new or edited scenes.
      tactics::ui::BuilderMenu builderMenu;
      if (screen == tbgwaf_flow::State::Builder) {
        for (int i = 0; i < static_cast<int>(builderScenes.size()); ++i) {
          if (!builderThumbnailDirty[i]) continue;
          RenderBuilderThumbnail(renderer, builderScenes[i], &builderThumbnails[i], i);
          builderThumbnailDirty[i] = false;
        }
        for (int i = 0; i < static_cast<int>(builderScenes.size()); ++i) {
          builderMenu.cards.push_back({builderScenes[i].name, builderThumbnails[i]});
        }
        builderMenu.lastLogAvailable = reproRecorder.Started();
        builderMenu.status = builderStatus;
      }
      std::optional<tbgwaf_flow::Event> flowEvent = tactics::ui::DrawMenu(
          screen, windowWidth, windowHeight, &mapSeed, &builderMenu);
      if (builderMenu.downloadLastLog) {
        const std::string where = DownloadReproLog(reproRecorder, builderLogSceneId);
        builderStatus = where.empty() ? "Download failed" : "Saved " + where;
      }
      if (builderMenu.deleteIndex >= 0) deleteBuilderScene(builderMenu.deleteIndex);
      // Headless smoke run: walk the menu automatically so gameplay is
      // exercised (the builder smoke walks into the sample scene's editor).
      bool autoEnterGameplay = isSmokeTest;
      bool autoCtf = false;
#ifdef __EMSCRIPTEN__
      autoEnterGameplay = autoEnterGameplay || aiEnabled;
      autoCtf = tbgwaf_default_ctf();
      ctfFromPreview = autoCtf;
#endif
      if (autoEnterGameplay && !flowEvent) {
        switch (screen) {
          case tbgwaf_flow::State::Splash: flowEvent = tbgwaf_flow::Event::NewGame; break;
          case tbgwaf_flow::State::GameMode:
            flowEvent = isSmokeBuilder ? tbgwaf_flow::Event::SelectBuilder
                        : autoCtf      ? tbgwaf_flow::Event::SelectCtf
                                       : tbgwaf_flow::Event::SelectRegular;
            break;
          case tbgwaf_flow::State::MapSelect: flowEvent = tbgwaf_flow::Event::SelectUrban; break;
          case tbgwaf_flow::State::Builder:
            if (isSmokeBuilder && !builderScenes.empty()) {
              builderMenu.openIndex = 0;
              flowEvent = tbgwaf_flow::Event::OpenBuilderScene;
            }
            break;
          default: break;
        }
      }
      if (flowEvent) {
        if (*flowEvent == tbgwaf_flow::Event::SelectRegular ||
            *flowEvent == tbgwaf_flow::Event::SelectCtf) {
          ctfMode = *flowEvent == tbgwaf_flow::Event::SelectCtf;
        }
        if (*flowEvent == tbgwaf_flow::Event::ExitBuilder) leaveBuilderFlow();
        if (*flowEvent == tbgwaf_flow::Event::OpenBuilderScene) {
          if (builderMenu.newScene) {
            tactics::BuilderScene fresh;  // Empty medium flat ground.
            fresh.name = "Scene " + std::to_string(builderScenes.size() + 1);
            builderMenu.openIndex = addBuilderScene(std::move(fresh));
          }
          if (builderMenu.openIndex >= 0 &&
              builderMenu.openIndex < static_cast<int>(builderScenes.size())) {
            openBuilderEditor(builderMenu.openIndex);
          } else {
            flowEvent.reset();
          }
        }
        if (flowEvent && (*flowEvent == tbgwaf_flow::Event::SelectUrban ||
                          *flowEvent == tbgwaf_flow::Event::SelectHills)) {
          // The menu's Urban entry uses the seed-driven random overpass mode.
          mapType = *flowEvent == tbgwaf_flow::Event::SelectHills ? "hilly" : "urban-auto";
          game.Reset(makeMap());
          timeline.Reset(game);
          timelinePlayback.Reset();
          for (auto& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);
        }
        if (flowEvent) screen = tbgwaf_flow::Next(screen, *flowEvent);
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
    bool simulationPaused = false;
#ifdef __EMSCRIPTEN__
    simulationPaused = networked && aiPaused && tbgwaf_auto_commit() && !builderHotseat;
#endif
    // A builder game is local to this instance, so even a web follower
    // simulates it itself.
    const bool simulatesLocally = isSimulator || builderHotseat;
    if (!simulationPaused && (simulatesLocally || game.Mode() != InputMode::Executing)) {
      game.Update(dt);
    }
    // Sighting memory is per-page derived state: tick it unconditionally so
    // a follower (which skips Update() while Executing) still builds it.
    game.UpdateSightingMemory(dt);
    for (auto& camera : cameras) camera.Update(dt);

#ifdef __EMSCRIPTEN__
    // Apply completed typed choices only if their request still matches this
    // round. Mode changes/restarts increment the JS generation and clear the
    // pending request before a stale result can reach GameLogic.
    while (char* raw = tbgwaf_ai_next()) {
      const std::string message(raw);
      std::free(raw);
      if (!pendingJevRequest || message.size() < 3 || message[1] != ' ') continue;
      const size_t split = message.find(' ', 2);
      const std::string requestId = message.substr(2, split == std::string::npos
                                                         ? std::string::npos
                                                         : split - 2);
      if (requestId != pendingJevRequest->id) continue;
      if (message[0] == 'D' && split != std::string::npos) {
        std::vector<std::string> choices;
        std::stringstream list(message.substr(split + 1));
        for (std::string item; std::getline(list, item, ',');) choices.push_back(item);
        if (!tactics::ai::ApplyJevChoice(&game, *pendingJevRequest, choices)) {
          jevErrorHold = true;
          tbgwaf_ai_local_error("selected action became invalid; retry required");
        }
      } else if (message[0] == 'E') {
        jevErrorHold = true;
      }
      pendingJevRequest.reset();
      jevBuilder.reset();
    }

    if (aiEnabled && !aiPaused && !jevErrorHold && !awaitingPeerSync &&
        game.Mode() == InputMode::AwaitingSelection) {
      if (!pendingJevRequest) {
        if (!jevBuilder) {
          jevBuilder.emplace(game, *fixedTeam, ++jevRequestNonce);
        }
        if (jevBuilder->Step(6.0)) {
          pendingJevRequest = jevBuilder->Finish();
          jevBuilder.reset();
          if (pendingJevRequest) {
            if (tbgwaf_ai_fallback()) {
              const std::vector<std::string> choices =
                  tactics::ai::DeterministicFallbackChoice(*pendingJevRequest);
              if (!tactics::ai::ApplyJevChoice(&game, *pendingJevRequest, choices)) {
                jevErrorHold = true;
                tbgwaf_ai_local_error("deterministic fallback action became invalid; retry required");
              }
              pendingJevRequest.reset();
            } else {
              tbgwaf_ai_request(pendingJevRequest->json.c_str());
            }
          }
        }
      }
    } else if (!aiEnabled || aiPaused || game.Mode() == InputMode::GameOver) {
      pendingJevRequest.reset();
      jevBuilder.reset();
    }

    // Blue is the only simulator. With no human player (Jev vs Jev) it commits once as
    // soon as every living figure has a plan; human modes commit manually.
    if (networked && isSimulator && !builderHotseat && tbgwaf_auto_commit() && !aiPaused &&
        game.CanCommitRound() && lastAutoCommittedRound != game.RoundNumber()) {
      if (autoCommitHold >= kAutoCommitDelay) {
        autoCommitHold = 0.0f;
        lastAutoCommittedRound = game.RoundNumber();
        game.CommitRound();
      } else {
        autoCommitHold += dt;
      }
    } else {
      autoCommitHold = 0.0f;
    }
#endif
    // Builder smoke: plan the sample scene's scripted round through the same
    // input API a player's clicks reach (all recorded), then commit. Blue 0
    // is the sniper on the cube top.
    if (isSmokeBuilder && builderHotseat && frameCount == 10 &&
        game.Mode() == InputMode::AwaitingSelection) {
      game.ClickUnit(2, Team::Red);
      game.ChooseMove();
      game.ClickGround(glm::vec3(5.0f, 0.0f, 4.0f), Team::Red);
      game.FinishMovePlan();
      game.ClickUnit(1, Team::Blue);
      game.ChooseShoot();
      game.SetPlannedShotCount(3, Team::Blue);
      game.PlaceAimPoint(glm::vec3(8.0f, 1.0f, -5.0f), Team::Blue);
      game.ConfirmAim(Team::Blue);
      game.ClickUnit(0, Team::Blue);
      game.ChoosePass();
      game.ClickUnit(3, Team::Red);
      game.ChoosePass();
      if (!game.CanCommitRound()) std::fprintf(stderr, "builder smoke: round not fully planned\n");
      game.CommitRound();
    }

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
    const bool planning = !replayActive && isPlanningMode(game.Mode()) && !awaitingPeerSync;
    // The HUD reflects whatever state is on screen (replay or live); its
    // button presses are only applied to the live game when no replay is
    // active (replay is strictly view-only).
    const bool hudPlanning = isPlanningMode(displayGame.Mode()) && !awaitingPeerSync;
#ifdef __EMSCRIPTEN__
    const bool humanPlanning = planning && !aiEnabled;
    const bool hudHumanPlanning = hudPlanning && !aiEnabled;
#else
    const bool humanPlanning = planning;
    const bool hudHumanPlanning = hudPlanning;
#endif
    // Jev-vs-Jev spectator (web, Blue instance only): render-only omniscience.
    // Jev's request state is built elsewhere from team-visible data, untouched.
#ifdef __EMSCRIPTEN__
    const bool spectator = networked && tbgwaf_spectator();
#else
    const bool spectator = false;
#endif
    // Builder hotseat: one screen plays both teams, so nothing is fogged and
    // both teams' FOV cones are drawn (the spectator view's rendering).
    const bool fogActive =
        !spectator && !builderHotseat && displayGame.Mode() != InputMode::GameOver;
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
    // Thermal view restyles the whole HUD (both panes and the debug panel
    // below) to sensor-green symbology; popped after the debug panel.
    if (debugOptions.thermal) ui::PushThermalHudStyle();
    float roundPanelBottom = 2.0f;
    for (int pane = 0; pane < paneCount; ++pane) {
      const ui::HudActions hud = ui::DrawHud(displayGame, paneTeam(pane), hudHumanPlanning,
                                             paneRects[pane], windowHeight, cameras[pane],
                                             nullptr, pane == 0 ? &roundPanelBottom : nullptr);
      // Replay is view-only: while it is showing, HUD presses must not leak
      // into the live match (return to Live first).
      if (!replayActive) {
        if (hud.newMatch && builderHotseat) {
          // Same scene, fresh recording.
          if (builderIndex >= 0 && builderIndex < static_cast<int>(builderScenes.size())) {
            startBuilderGame(builderIndex);
          }
        } else if (hud.newMatch) {
          if (isSimulator) {
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
          if (simulatesLocally) {
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

    // --- Dispatch deferred input, now that WantCaptureMouse reflects the UI
    // actually built this frame. ---
    if (escapePending && !replayActive) game.CancelAction();
    if (enterPending && !replayActive) game.FinishMovePlan();

    const bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse;
    if (!uiWantsMouse && humanPlanning) {
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
                                        builderHotseat ? aimed->team : paneTeam(aimedPane));
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
                                                          paneVisibility[clickPane],
                                                          game.GetScene())) {
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
          // Hotseat: any figure is selectable -- the click acts for the
          // clicked figure's own team.
          const Unit* hit = game.FindUnit(hitUnit);
          game.ClickUnit(hitUnit, builderHotseat && hit ? hit->team : clickTeam);
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
    const bool hudWasThermal = debugOptions.thermal;
    ui::DrawDebugPanel(debugOptions.disableFov, debugOptions.disableShadows, debugOptions.thermal,
                       debugOptions.thermalBlackHot, debugOptions.thermalFovAlpha,
                       debugOptions.thermalAo.radius,
                       debugOptions.thermalAo.strength, debugOptions.thermalAo.quality,
                       debugOptions.thermalAo.softness,
                       debugOptions.disableOcclusionFade, showFps,
                       smoothedFps, frameMs, fovMs, frameStats,
                       roundPanelBottom + 6.0f);
    // Pop against the pre-panel state: the checkbox may have just toggled
    // `thermal`, but the push at the top of this frame is what must unwind.
    if (hudWasThermal) ui::PopThermalHudStyle();
    // Builder panel (builder games): which scene is in play, Download log
    // (the recording so far; works mid-game and after game over) and Stop,
    // back to the Builder's cards.
    if (builderHotseat) {
      // Top-right: the Round panel holds the top-left corner.
      ImGui::SetNextWindowPos(ImVec2(windowWidth - 10.0f, 10.0f), ImGuiCond_Always,
                              ImVec2(1.0f, 0.0f));
      ImGui::Begin("Builder game", nullptr,
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                       ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                       ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoTitleBar);
      const bool sceneKnown = builderIndex >= 0 && builderIndex < static_cast<int>(builderScenes.size());
      ImGui::Text("Builder: %s", sceneKnown ? builderScenes[builderIndex].name.c_str() : "?");
      ImGui::SameLine();
      if (reproRecorder.Started()) {
        if (ImGui::Button("Download log")) {
          const std::string where = DownloadReproLog(reproRecorder, builderLogSceneId);
          builderStatus = where.empty() ? "Download failed" : "Saved " + where;
        }
        ImGui::SameLine();
      }
      if (ImGui::Button("Stop")) {
        stopBuilderGame();
        screen = tbgwaf_flow::Next(screen, tbgwaf_flow::Event::BackToBuilder);
      }
      ImGui::TextUnformatted(
          "Hotseat: click any figure to plan it. Plan both teams, then Commit Round.");
      if (!builderStatus.empty()) ImGui::TextUnformatted(builderStatus.c_str());
      ImGui::End();
    }

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
      overlays.spectatorFov = spectator || builderHotseat;
      renderer.RenderPane(displayGame, paneTeam(pane), fogActive, paneVisibility[pane],
                          cameras[pane], rect.x, 0, rect.width, windowHeight, overlays, 0,
                          debugOptions);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(window);

#ifdef __EMSCRIPTEN__
    // A builder game is local: the page hears nothing of it, and nothing of
    // it goes on the bus.
    const int reportWinner = game.Winner() ? static_cast<int>(*game.Winner()) : -1;
    if (!builderHotseat &&
        (game.RoundNumber() != lastReportedRound || game.Mode() != lastReportedMode ||
         reportWinner != lastReportedWinner)) {
      const char* phase = game.Mode() == InputMode::GameOver
                              ? "game-over"
                              : game.Mode() == InputMode::Executing ? "executing" : "planning";
      tbgwaf_report_match(phase, game.RoundNumber(), reportWinner);
      lastReportedRound = game.RoundNumber();
      lastReportedMode = game.Mode();
      lastReportedWinner = reportWinner;
    }
    // Publish state changes: the simulator ships the whole match, the
    // follower just its plans (only while planning).
    if (networked && !builderHotseat && !awaitingPeerSync &&
        (isSimulator || isPlanningMode(game.Mode()))) {
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
#ifndef __EMSCRIPTEN__
    if (quit && reproRecorder.Started()) {
      if (const char* outPath = std::getenv("TBGWAF_REPRO_OUT")) {
        std::ofstream out(outPath, std::ios::binary);
        out << reproRecorder.Text();
        std::fprintf(stderr, "repro log written to %s\n", outPath);
      }
    }
#endif

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
