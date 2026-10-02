// Map-only visual tests: renders each seeded procedural city (no units, no
// fog, no team panes) full-frame and diffs it against a golden under
// tests/map_goldens/. A change to the generator's output for a seed shows up
// as a golden diff. Separate from the gameplay scenario runner
// (visual_main.cpp); same headless xvfb + software-GL setup.
//
// `--update-baselines` regenerates the goldens instead of comparing.

#include <SDL.h>
#include <GLES3/gl3.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "visual/ImageUtil.h"
#include "game/GameLogic.h"
#include "game/MapGenerator.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"

namespace fs = std::filesystem;

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kPixelThreshold = 25;
constexpr double kMaxDiffFraction = 0.002;
constexpr float kCityCameraZoom = 250.0f;
// Hilly maps are half the city's extent; zoom in proportionally.
constexpr float kHillyCameraZoom = 115.0f;
const std::vector<uint32_t> kCitySeeds = {7, 42, 2024};
// Hilly goldens also pin the navmesh boundary debug overlay, so a terrain or
// navmesh regression shows up as a visual diff.
const std::vector<uint32_t> kHillySeeds = {7, 2024};

struct Options {
  fs::path goldensDir = "tests/map_goldens";
  fs::path outDir = "map_visual_out";
  bool updateBaselines = false;
};

}  // namespace

int main(int argc, char** argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--update-baselines") {
      options.updateBaselines = true;
    } else if (arg == "--goldens-dir" && i + 1 < argc) {
      options.goldensDir = argv[++i];
    } else if (arg == "--out-dir" && i + 1 < argc) {
      options.outDir = argv[++i];
    } else {
      std::fprintf(stderr,
                   "usage: tactics_map_visual_tests [--update-baselines] [--goldens-dir DIR] "
                   "[--out-dir DIR]\n");
      return 1;
    }
  }

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
      SDL_CreateWindow("tbgwaf map tests", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                       kWindowWidth, kWindowHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
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
  int drawableWidth = 0, drawableHeight = 0;
  SDL_GL_GetDrawableSize(window, &drawableWidth, &drawableHeight);
  if (drawableWidth != kWindowWidth || drawableHeight != kWindowHeight) {
    std::fprintf(stderr, "drawable size %dx%d != required %dx%d\n", drawableWidth, drawableHeight,
                 kWindowWidth, kWindowHeight);
    return 1;
  }

  gfx::SceneRenderer renderer;
  if (!renderer.Init()) return 1;

  struct MapCase {
    std::string name;
    tactics::Scene scene;
    float cameraZoom;
    bool showNavMesh;  // Hilly maps pin the navmesh boundary overlay too.
    float pitchOffset = 0.0f;
    float yawOffset = 0.0f;
  };
  std::vector<MapCase> cases;
  for (const uint32_t seed : kCitySeeds) {
    cases.push_back({"city_seed_" + std::to_string(seed), tactics::GenerateUrbanMap(seed),
                     kCityCameraZoom, /*showNavMesh=*/false});
  }
  tactics::MapGeneratorConfig mergeConfig;
  mergeConfig.arteryCount = 2;
  cases.push_back({"city_merge_seed_42", tactics::GenerateUrbanMap(42, mergeConfig),
                   kCityCameraZoom, /*showNavMesh=*/false});
  tactics::MapGeneratorConfig elevatedConfig = mergeConfig;
  elevatedConfig.elevatedHighway = true;
  // Framed low and from the side so the deck, both ramps, the on-ramp fork
  // and the pier bents beneath all read clearly.
  cases.push_back({"city_elevated_seed_7", tactics::GenerateUrbanMap(7, elevatedConfig),
                   180.0f, /*showNavMesh=*/false, /*pitchOffset=*/-0.55f, /*yawOffset=*/-1.6f});
  for (const uint32_t seed : kHillySeeds) {
    cases.push_back({"hilly_seed_" + std::to_string(seed), tactics::GenerateHillyMap(seed),
                     kHillyCameraZoom, /*showNavMesh=*/true});
  }

  int failures = 0;
  for (MapCase& mapCase : cases) {
    const std::string& name = mapCase.name;
    const fs::path goldenPath = options.goldensDir / (name + ".png");

    gfx::OrbitCamera camera;
    camera.Rotate(mapCase.yawOffset, mapCase.pitchOffset);
    camera.Zoom(mapCase.cameraZoom);
    camera.target = glm::vec3(0.0f);
    camera.Update(1.0e3f);

    tactics::Scene scene = std::move(mapCase.scene);
    scene.units.clear();  // Map only: no figures.
    tactics::GameLogic game(scene);
    tactics::NavMesh navMesh;
    gfx::PaneOverlays overlays;
    if (mapCase.showNavMesh) {
      navMesh.Build(game.GetScene().obstacles, game.GetScene().mapHalfExtent,
                    tactics::constants::kAgentRadius, &game.GetScene().ground,
                    &game.GetScene().walkSurfaces);
      overlays.navMeshDebug = &navMesh;
    }
    renderer.RenderPane(game, tactics::Team::Blue, /*fogActive=*/false, tactics::TeamVisibility{},
                        camera, 0, 0, kWindowWidth, kWindowHeight, overlays);
    const visual::Image image = visual::CaptureFramebuffer(kWindowWidth, kWindowHeight);

    if (options.updateBaselines) {
      if (!visual::SavePng(goldenPath, image)) {
        std::fprintf(stderr, "FAIL %s: could not write %s\n", name.c_str(),
                     goldenPath.string().c_str());
        ++failures;
      }
      continue;
    }

    visual::Image golden;
    if (!visual::LoadPng(goldenPath, &golden)) {
      std::fprintf(stderr,
                   "FAIL %s: golden %s missing/unreadable (run "
                   "scripts/update_golden_baselines.sh to regenerate)\n",
                   name.c_str(), goldenPath.string().c_str());
      ++failures;
      continue;
    }
    const visual::DiffResult diff = visual::DiffImages(golden, image, kPixelThreshold);
    if (diff.sizeMismatch || diff.Fraction() > kMaxDiffFraction) {
      visual::SavePng(options.outDir / (name + ".actual.png"), image);
      if (!diff.sizeMismatch) visual::SavePng(options.outDir / (name + ".diff.png"), diff.diffImage);
      std::fprintf(stderr, "FAIL %s: %s (actual/diff written under %s)\n", name.c_str(),
                   diff.sizeMismatch ? "golden has different dimensions"
                                     : (std::to_string(diff.Fraction() * 100.0) + "% of pixels differ").c_str(),
                   options.outDir.string().c_str());
      ++failures;
    } else {
      std::printf("PASS %s\n", name.c_str());
    }
  }

  renderer.Destroy();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();

  if (failures > 0) {
    std::fprintf(stderr, "%d of %zu map(s) failed.\n", failures, cases.size());
    return 1;
  }
  return 0;
}
