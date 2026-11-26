// Renders blasts inside generated buildings to PNG frames for the asset
// gallery's "Blasts in buildings" folder (tests/blast_gallery/).
//
//   xvfb-run tactics_blast_gallery <out dir>
//
// Each case builds one hollow building on an empty scene, places the debug
// bomb inside it, detonates, and captures the blast at a few moments of the
// debris flight (the renderer replays the recorded tracks on a wall clock
// when the blast goes off outside a round).
#include <SDL.h>
#include <GLES3/gl3.h>

#include <cstdio>
#include <filesystem>
#include <string>

#include "game/GameLogic.h"
#include "game/MapGenerator.h"
#include "gfx/SceneRenderer.h"
#include "visual/ImageUtil.h"

namespace {
struct Case {
  const char* name;
  uint32_t buildingSeed;
  glm::vec3 bomb;
  float radius;
  float yaw, pitch;
};
const Case kCases[] = {
    {"ground_floor_blast", 11, {0.0f, 1.2f, 3.0f}, 7.0f, 3.1f, -0.5f},
    {"corner_blast", 23, {2.5f, 1.2f, 2.5f}, 7.0f, 2.2f, -0.6f},
    {"upper_floor_blast", 5, {-2.5f, 4.5f, 2.5f}, 7.0f, 2.5f, -0.45f},
};
const float kShotTimes[] = {0.0f, 0.4f, 0.9f, 4.0f};
constexpr int kW = 960, kH = 600;

bool Run(gfx::SceneRenderer& renderer, const Case& c, const std::filesystem::path& out) {
  tactics::Scene scene;
  tactics::GeneratedBuildingSpec spec;
  spec.seed = c.buildingSeed;
  spec.hx = spec.hz = 5.5f;
  if (!tactics::AppendGeneratedBuilding(&scene, spec)) return false;
  tactics::GameLogic game(scene);
  game.SetMatchSeed(7u);
  gfx::OrbitCamera camera;
  camera.target = glm::vec3(0.0f, 2.0f, 0.0f);
  camera.Zoom(17.0f);
  camera.Rotate(c.yaw, c.pitch);
  camera.Update(1000.0f);
  gfx::RenderDebugOptions debug;
  debug.disableFov = true;
  const auto render = [&] {
    renderer.RenderPane(game, tactics::Team::Blue, false, {}, camera, 0, 0, kW, kH, {}, 0,
                        debug);
    return visual::CaptureFramebuffer(kW, kH);
  };
  render();
  game.DebugPlaceBomb(c.bomb, c.radius);
  if (!game.DebugDetonateBomb()) return false;
  const Uint32 start = SDL_GetTicks();
  for (size_t shot = 0; shot < sizeof kShotTimes / sizeof kShotTimes[0]; ++shot) {
    const Uint32 due = start + static_cast<Uint32>(kShotTimes[shot] * 1000.0f);
    while (SDL_GetTicks() < due) SDL_Delay(5);
    const auto image = render();
    char name[96];
    std::snprintf(name, sizeof name, "%s_%zu.png", c.name, shot + 1);
    if (!visual::SavePng(out / name, image)) return false;
  }
  return true;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <out dir>\n", argv[0]);
    return 2;
  }
  const std::filesystem::path out = argv[1];
  std::filesystem::create_directories(out);
  if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1;
  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_Window* window = SDL_CreateWindow("Blast gallery", 0, 0, kW, kH,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  SDL_GLContext context = window ? SDL_GL_CreateContext(window) : nullptr;
  if (!context) {
    std::fprintf(stderr, "GL context failed: %s\n", SDL_GetError());
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  int result = 0;
  {
    gfx::SceneRenderer renderer;
    if (!renderer.Init()) {
      std::fprintf(stderr, "Renderer initialization failed\n");
      result = 1;
    } else {
      for (const Case& c : kCases) {
        if (!Run(renderer, c, out)) {
          std::fprintf(stderr, "case %s failed\n", c.name);
          result = 1;
        }
      }
    }
    renderer.Destroy();
  }
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
