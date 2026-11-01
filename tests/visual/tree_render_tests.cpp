// Actual GLES instancing, cache invalidation and AO integration, independent
// of the historical city/gameplay goldens. Optional review screenshot.
#include <SDL.h>
#include <GLES3/gl3.h>

#include <cstdio>
#include <stdexcept>

#include "game/MapGenerator.h"
#include "gfx/SceneRenderer.h"
#include "visual/ImageUtil.h"

namespace {
void Require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

void CheckTrees(gfx::SceneRenderer& renderer) {
  tactics::Scene scene;
  for (int variant = 0; variant < 3; ++variant)
    scene.trees.push_back({{(variant - 1) * 5.0f, 0, 0}, 1.0f, 0.4f, 1.0f, variant});
  tactics::GameLogic game(scene);
  gfx::OrbitCamera camera;
  camera.FitToExtent(10.0f);
  gfx::RenderDebugOptions debug;
  debug.disableFov = true;
  gfx::RenderFrameStats stats;
  debug.stats = &stats;
  auto render = [&](const tactics::GameLogic& current) {
    stats = {};
    renderer.RenderPane(current, tactics::Team::Blue, false, {}, camera, 0, 0, 320, 200,
                         {}, 0, debug);
    auto image = visual::CaptureFramebuffer(320, 200);
    Require(glGetError() == GL_NO_ERROR, "Tree render produced a GL error");
    return image.rgb;
  };
  debug.disableTrees = true;
  const auto bare = render(game);
  const int bareDraws = stats.drawCalls;
  debug.disableTrees = false;
  const auto color = render(game);
  Require(color != bare, "Trees did not appear");
  Require(stats.drawCalls == bareDraws + 6, "Trees exceeded three draws per color/depth pass");
  Require(render(game) == color, "Cached trees changed between identical frames");

  // Many more instances must still cost the same number of submissions.
  tactics::Scene crowded = scene;
  for (int i = 0; i < 30; ++i)
    crowded.trees.push_back({{float(i % 6) * 4 - 10, 0, float(i / 6) * 4 - 10},
                             0.8f, 0.1f * i, 1.0f, i % 3});
  render(tactics::GameLogic(crowded));
  Require(stats.drawCalls == bareDraws + 6, "Draw count grew with tree count");
  Require(render(game) == color, "Switching scenes did not restore tree instances");
  scene.trees[0].position.z += 5.0f;
  Require(render(tactics::GameLogic(scene)) != color, "Same-size tree update reused stale instances");
  scene.trees.clear();
  Require(render(tactics::GameLogic(scene)) == bare, "Empty scene retained old trees");

  debug.thermal = true;
  const auto blackHot = render(game);
  debug.disableTrees = true;
  const auto bareThermal = render(game);
  const int bareThermalDraws = stats.drawCalls;
  debug.disableTrees = false;
  Require(render(game) != bareThermal, "Trees missing from AO view");
  Require(stats.drawCalls == bareThermalDraws + 3, "AO trees must use three draws, no shadow pass");
  debug.thermalBlackHot = false;
  const auto whiteHot = render(game);
  Require(blackHot != whiteHot, "Tree thermal polarity did not change");
  for (size_t i = 0; i < whiteHot.size(); i += 3) {
    Require(whiteHot[i] == whiteHot[i + 1] && whiteHot[i] == whiteHot[i + 2],
            "Foliage leaked color into AO view");
  }
  debug.thermal = false;
  Require(render(game) == color, "Trees changed after switching back from AO view");
}

void Screenshot(gfx::SceneRenderer& renderer, const char* path) {
  auto scene = tactics::GenerateUrbanMap(42);
  Require(!scene.trees.empty(), "Review seed has no trees");
  scene.units.clear();
  gfx::OrbitCamera camera;
  camera.target = scene.trees.front().position;
  camera.Zoom(22.0f);
  camera.Rotate(0.0f, -0.3f);
  camera.Update(1000.0f);
  tactics::GameLogic game(scene);
  renderer.RenderPane(game, tactics::Team::Blue, false, {}, camera, 0, 0, 1280, 720);
  Require(visual::SavePng(path, visual::CaptureFramebuffer(1280, 720)), "Screenshot write failed");
}
}  // namespace

int main(int argc, char** argv) {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1;
  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_Window* window = SDL_CreateWindow("Tree renderer tests", 0, 0, 1280, 720,
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
    try {
      Require(renderer.Init(), "Renderer initialization failed");
      CheckTrees(renderer);
      if (argc > 1) Screenshot(renderer, argv[1]);
      std::puts("PASS tree rendering: instancing, scene replacement, AO, polarity, mode switching");
    } catch (const std::exception& e) {
      std::fprintf(stderr, "FAIL tree rendering: %s\n", e.what());
      result = 1;
    }
    renderer.Destroy();
  }
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
