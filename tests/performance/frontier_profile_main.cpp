// Move-planning profiler (issue #256): frame cost while planning a move with
// the movement frontier open (static and orbiting camera, see-through fade on
// and off) plus the latency of the frontier-opening click itself. Manual-run
// only, like the performance suite:
//
//   xvfb-run --auto-servernum ./build/tactics_frontier_profile [seed] [frames]

#include <SDL.h>
#include <GLES3/gl3.h>

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>

#include "game/GameLogic.h"
#include "game/MapGenerator.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kPaneCount = 2;

double RenderFrames(gfx::SceneRenderer* renderer, const tactics::GameLogic& game,
                    gfx::OrbitCamera* cameras, int frames, bool disableFade,
                    bool orbit = false) {
  std::array<tactics::TeamVisibility, kPaneCount> visibility;
  visibility[0] = game.ComputeVisibility(tactics::Team::Blue);
  visibility[1] = game.ComputeVisibility(tactics::Team::Red);
  const tactics::Team teams[kPaneCount] = {tactics::Team::Blue, tactics::Team::Red};
  glFinish();
  const Uint64 start = SDL_GetPerformanceCounter();
  for (int f = 0; f < frames; ++f) {
    if (orbit) {
      for (int pane = 0; pane < kPaneCount; ++pane) cameras[pane].Rotate(0.01f, 0.0f);
    }
    gfx::RenderDebugOptions debug;
    debug.disableOcclusionFade = disableFade;
    for (int pane = 0; pane < kPaneCount; ++pane) {
      const gfx::PaneOverlays overlays = gfx::BuildPaneOverlays(game, teams[pane]);
      renderer->RenderPane(game, teams[pane], /*fogActive=*/true, visibility[pane],
                           cameras[pane], pane * (kWindowWidth / kPaneCount), 0,
                           kWindowWidth / kPaneCount, kWindowHeight, overlays, 0, debug);
    }
    SDL_GL_SwapWindow(SDL_GL_GetCurrentWindow());
    glFinish();
  }
  const Uint64 end = SDL_GetPerformanceCounter();
  return 1000.0 * static_cast<double>(end - start) /
         static_cast<double>(SDL_GetPerformanceFrequency()) / frames;
}

}  // namespace

int main(int argc, char** argv) {
  uint32_t seed = 2024;
  int frames = 60;
  if (argc > 1) seed = static_cast<uint32_t>(std::strtoul(argv[1], nullptr, 10));
  if (argc > 2) frames = std::atoi(argv[2]);

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
  SDL_Window* window =
      SDL_CreateWindow("frontier profile", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                       kWindowWidth, kWindowHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
  SDL_GLContext context = SDL_GL_CreateContext(window);
  SDL_GL_SetSwapInterval(0);

  gfx::SceneRenderer renderer;
  if (!renderer.Init()) return 1;
  renderer.SetFovOverlayMode(gfx::FovOverlayMode::ShadowMap);

  tactics::GameLogic game(tactics::GenerateUrbanMap(seed));
  gfx::OrbitCamera cameras[kPaneCount];
  for (gfx::OrbitCamera& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);

  std::printf("seed %u: obstacles=%zu walkSurfaces=%zu massing=%zu units=%zu ziplines=%zu\n",
              seed, game.GetScene().obstacles.size(), game.GetScene().walkSurfaces.size(),
              game.GetScene().massing.size(), game.GetScene().units.size(),
              game.GetScene().ziplines.size());

  // Warm up (shader compiles, mesh uploads).
  RenderFrames(&renderer, game, cameras, 10, false);

  const double closedFade = RenderFrames(&renderer, game, cameras, frames, false);
  const double closedNoFade = RenderFrames(&renderer, game, cameras, frames, true);

  // Open the frontier for the first living Blue unit.
  int unitId = -1;
  for (const tactics::Unit& u : game.GetScene().units) {
    if (u.alive && u.team == tactics::Team::Blue) { unitId = u.id; break; }
  }
  game.ClickUnit(unitId, tactics::Team::Blue);
  const Uint64 t0 = SDL_GetPerformanceCounter();
  game.ChooseMove();
  const Uint64 t1 = SDL_GetPerformanceCounter();
  const double openMs = 1000.0 * static_cast<double>(t1 - t0) /
                        static_cast<double>(SDL_GetPerformanceFrequency());
  const tactics::ReachField* f = game.MoveFrontier();
  int reached = 0;
  if (f) {
    for (int iz = 0; iz < f->nz; ++iz)
      for (int ix = 0; ix < f->nx; ++ix) reached += f->Reached(ix, iz) ? 1 : 0;
  }
  std::printf("ChooseMove (frontier open) latency: %.2f ms; field %dx%d, %d reached, "
              "%zu zipline frontiers\n",
              openMs, f ? f->nx : 0, f ? f->nz : 0, reached, game.ZiplineFrontiers().size());

  RenderFrames(&renderer, game, cameras, 10, false);  // Warm frontier meshes.
  const double openFade = RenderFrames(&renderer, game, cameras, frames, false);
  const double openNoFade = RenderFrames(&renderer, game, cameras, frames, true);
  const double orbitFade = RenderFrames(&renderer, game, cameras, frames, false, true);
  const double orbitNoFade = RenderFrames(&renderer, game, cameras, frames, true, true);

  std::printf("frame ms (%d frames, %d panes):\n", frames, kPaneCount);
  std::printf("  frontier closed, fade on : %8.2f\n", closedFade);
  std::printf("  frontier closed, fade off: %8.2f\n", closedNoFade);
  std::printf("  frontier open,   fade on : %8.2f\n", openFade);
  std::printf("  frontier open,   fade off: %8.2f\n", openNoFade);
  std::printf("  open + orbiting, fade on : %8.2f\n", orbitFade);
  std::printf("  open + orbiting, fade off: %8.2f\n", orbitNoFade);
  std::printf("  fade cost closed: %.2f ms, open: %.2f ms, open+orbit: %.2f ms\n",
              closedFade - closedNoFade, openFade - openNoFade, orbitFade - orbitNoFade);

  renderer.Destroy();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
