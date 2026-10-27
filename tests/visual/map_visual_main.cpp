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
#include <optional>
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
// 50 randomly drawn seeds of the default (seed-driven terrain / overpass)
// city, so the gallery's map view samples what players actually get.
const std::vector<uint32_t> kRandomCitySeeds = {
    6805,  7622,  13514, 15121, 16096, 16478, 16693, 18035, 19376, 22493,
    24711, 25257, 26421, 27757, 32252, 32727, 37632, 38105, 39138, 39683,
    40925, 46219, 47957, 48757, 50859, 51093, 54145, 58507, 59312, 59877,
    64344, 65145, 66154, 66720, 75266, 80231, 80930, 81365, 82666, 83042,
    87158, 87288, 87781, 89225, 90416, 91714, 92269, 92718, 94609, 94951};

struct Options {
  fs::path goldensDir = "tests/map_goldens";
  fs::path outDir = "map_visual_out";
  bool updateBaselines = false;
  fs::path dumpAo;
  std::string mapName;
  bool smallAo = false;
};

// PR #178: native test-only readback; no PNG or filesystem dependency in
// the game/WASM. Each XY slice places +XYZ RGB beside -XYZ RGB; A is validity.
bool DumpAo(const gfx::AmbientOcclusionVolume& volume, const fs::path& directory) {
  if (!volume.positive || !volume.negative) return false;
  fs::create_directories(directory);
  GLuint fbo = 0;
  GLint previous = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &previous);
  glGenFramebuffers(1, &fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  const int w = volume.dims.x, h = volume.dims.y;
  std::vector<unsigned char> pixels(static_cast<size_t>(w) * h * 4);
  std::vector<unsigned char> paired(static_cast<size_t>(w) * h * 8);
  bool ok = true;
  for (int z = 0; z < volume.dims.z && ok; ++z) {
    for (int side = 0; side < 2; ++side) {
      glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0,
                                side == 0 ? volume.positive : volume.negative, 0, z);
      if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        ok = false;
        break;
      }
      glReadPixels(0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
      for (int y = 0; y < h; ++y) {
        std::copy_n(pixels.data() + static_cast<size_t>(y) * w * 4, w * 4,
                    paired.data() + (static_cast<size_t>(h - 1 - y) * 2 * w + side * w) * 4);
      }
    }
    char filename[32];
    std::snprintf(filename, sizeof filename, "slice-%03d.png", z);
    if (ok) ok = stbi_write_png((directory / filename).string().c_str(), w * 2, h, 4,
                                paired.data(), w * 8) != 0;
  }
  ok = ok && glGetError() == GL_NO_ERROR;
  glBindFramebuffer(GL_FRAMEBUFFER, previous);
  glDeleteFramebuffers(1, &fbo);
  return ok;
}

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
    } else if (arg == "--dump-ao" && i + 1 < argc) {
      options.dumpAo = argv[++i];
    } else if (arg == "--map" && i + 1 < argc) {
      options.mapName = argv[++i];
    } else if (arg == "--ao-test-settings") {
      options.smallAo = true;
    } else {
      std::fprintf(stderr,
                   "usage: tactics_map_visual_tests [--update-baselines] [--goldens-dir DIR] "
                   "[--out-dir DIR] [--dump-ao DIR] [--map NAME] [--ao-test-settings]\n");
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
    glm::vec3 cameraTarget = glm::vec3(0.0f);
    // When set, pins the move-frontier overlay of a unit standing here
    // (issue #102: it must stay on the deck, not spill over its edges).
    std::optional<glm::vec3> frontierFrom;
    float frontierBudget = 0.0f;
  };
  std::vector<MapCase> cases;
  // Overpass presence is seed-driven by default (issue #130); every golden
  // here pins an explicit presence (and layout) so it tests one variant.
  tactics::MapGeneratorConfig gradeConfig;
  gradeConfig.elevatedHighway = tactics::OverpassMode::Off;
  // Terrain is seed-driven too; the flat-city goldens pin it off (terrain has its own).
  gradeConfig.features.terrain.mode = tactics::FeatureMode::Off;
  for (const uint32_t seed : kCitySeeds) {
    cases.push_back({"city_seed_" + std::to_string(seed),
                     tactics::GenerateUrbanMap(seed, gradeConfig), kCityCameraZoom,
                     /*showNavMesh=*/false});
  }
  tactics::MapGeneratorConfig terrainConfig = gradeConfig;
  terrainConfig.features.terrain.mode = tactics::FeatureMode::On;
  for (const uint32_t seed : kCitySeeds) {
    cases.push_back({"city_terrain_seed_" + std::to_string(seed),
                     tactics::GenerateUrbanMap(seed, terrainConfig), 180.0f,
                     /*showNavMesh=*/false, /*pitchOffset=*/-0.4f, /*yawOffset=*/-1.0f});
  }
  // Default seed 7 is the reported combination of terrain, an elevated
  // artery and stairs. Keep its complete geometry pinned together.
  tactics::MapGeneratorConfig terrainAutoConfig;
  cases.push_back({"city_terrain_auto_seed_7",
                   tactics::GenerateUrbanMap(7, terrainAutoConfig), 180.0f,
                   /*showNavMesh=*/false, /*pitchOffset=*/-0.4f, /*yawOffset=*/-1.0f});
  // Terrain under an overpass (issue #155): the deck starts on one level,
  // flies over the walls it crosses and settles onto the level at its far
  // end; the branch ramps between its own level and the deck.
  tactics::MapGeneratorConfig terrainElevatedConfig = terrainConfig;
  terrainElevatedConfig.arteryCount = 2;
  terrainElevatedConfig.elevatedHighway = tactics::OverpassMode::On;
  cases.push_back({"city_terrain_elevated_seed_16",
                   tactics::GenerateUrbanMap(16, terrainElevatedConfig), 180.0f,
                   /*showNavMesh=*/false, /*pitchOffset=*/-0.4f, /*yawOffset=*/-1.0f});
  // Street furniture and traffic props (issue #150), forced on (and framed
  // closer) so the golden pins the layer regardless of the seed's Auto draw.
  tactics::MapGeneratorConfig furnitureConfig = gradeConfig;
  furnitureConfig.features.furniture.mode = tactics::FeatureMode::On;
  cases.push_back({"city_furniture_seed_7", tactics::GenerateUrbanMap(7, furnitureConfig),
                   45.0f, /*showNavMesh=*/false, /*pitchOffset=*/-0.45f});
  tactics::MapGeneratorConfig mergeConfig = gradeConfig;
  mergeConfig.arteryCount = 2;
  cases.push_back({"city_merge_seed_42", tactics::GenerateUrbanMap(42, mergeConfig),
                   kCityCameraZoom, /*showNavMesh=*/false});
  tactics::MapGeneratorConfig mergeConfigAuto;  // Seed-driven overpass presence and layout.
  mergeConfigAuto.arteryCount = 2;
  mergeConfigAuto.features.terrain.mode = tactics::FeatureMode::Off;  // Overpass vs terrain: roadmap 4.4.
  tactics::MapGeneratorConfig elevatedConfig = mergeConfig;
  elevatedConfig.elevatedHighway = tactics::OverpassMode::On;
  elevatedConfig.overpassLayout = tactics::OverpassLayout::RampUpRampDown;
  elevatedConfig.branchEnd = tactics::BranchEnd::Ramp;
  // Framed low and from the side so the deck, both ramps, the on-ramp fork
  // and the pier bents beneath all read clearly.
  cases.push_back({"city_elevated_seed_7", tactics::GenerateUrbanMap(7, elevatedConfig),
                   180.0f, /*showNavMesh=*/false, /*pitchOffset=*/-0.55f, /*yawOffset=*/-1.6f});
  // The overpass layouts (issue #130) come from seeds whose default (Auto)
  // draw naturally yields them: nothing is forced. Same framing as above;
  // elevated ends are cut off flush at the map edge and the branch ramps
  // between grade and the deck, or (seed 26) runs off-map with no ramp.
  const std::pair<const char*, uint32_t> kNaturalOverpasses[] = {
      {"city_overpass_ramp_up_ramp_down_seed_33", 33},
      {"city_overpass_through_seed_4", 4},
      {"city_overpass_enter_ramp_up_seed_16", 16},
      {"city_overpass_enter_ramp_down_seed_22", 22},
      {"city_overpass_branch_off_map_seed_26", 26},
  };
  for (const auto& [name, seed] : kNaturalOverpasses) {
    cases.push_back({name, tactics::GenerateUrbanMap(seed, mergeConfigAuto), 180.0f,
                     /*showNavMesh=*/false, /*pitchOffset=*/-0.55f, /*yawOffset=*/-1.6f});
  }
  // Frontier of a unit on the deck, framed low and side-on at the deck edge.
  {
    MapCase deckFrontier{"city_elevated_deck_frontier_seed_7",
                         tactics::GenerateUrbanMap(7, elevatedConfig), 28.0f,
                         /*showNavMesh=*/false, /*pitchOffset=*/-0.35f, /*yawOffset=*/-1.2f};
    deckFrontier.cameraTarget = glm::vec3(-11.52f, 5.0f, 1.869f);
    deckFrontier.frontierFrom = glm::vec3(-11.52f, 5.0f, 1.869f);
    deckFrontier.frontierBudget = 12.0f;
    cases.push_back(std::move(deckFrontier));
  }
  for (const uint32_t seed : kRandomCitySeeds) {
    cases.push_back({"city_random_seed_" + std::to_string(seed),
                     tactics::GenerateUrbanMap(seed, terrainAutoConfig), 180.0f,
                     /*showNavMesh=*/false, /*pitchOffset=*/-0.4f, /*yawOffset=*/-1.0f});
  }
  // Building archetypes and massing (issue #151): one close-framed golden
  // per forced block character, so each archetype mix (and its parapets,
  // roof furniture, awnings and office setbacks) is pinned on its own. Seed
  // 7's downtown-side blocks fill the frame; the layout is identical in all
  // three, only kinds/heights/dressing differ.
  const std::pair<const char*, tactics::BlockCharacter> kCharacters[] = {
      {"city_archetypes_downtown_seed_7", tactics::BlockCharacter::Downtown},
      {"city_archetypes_residential_seed_7", tactics::BlockCharacter::Residential},
      {"city_archetypes_industrial_seed_7", tactics::BlockCharacter::Industrial},
  };
  for (const auto& [name, character] : kCharacters) {
    tactics::MapGeneratorConfig config = gradeConfig;
    config.character = character;
    MapCase archetypes{name, tactics::GenerateUrbanMap(7, config), 60.0f,
                       /*showNavMesh=*/false, /*pitchOffset=*/-0.45f, /*yawOffset=*/-0.9f};
    archetypes.cameraTarget = glm::vec3(-20.0f, 0.0f, 10.0f);
    cases.push_back(std::move(archetypes));
  }
  for (const uint32_t seed : kHillySeeds) {
    cases.push_back({"hilly_seed_" + std::to_string(seed), tactics::GenerateHillyMap(seed),
                     kHillyCameraZoom, /*showNavMesh=*/true});
  }

  if (!options.dumpAo.empty() && options.mapName.empty()) options.mapName = "city_elevated_seed_7";
  int failures = 0, rendered = 0;
  for (MapCase& mapCase : cases) {
    const std::string& name = mapCase.name;
    if (!options.mapName.empty() && name != options.mapName) continue;
    ++rendered;
    const fs::path goldenPath = options.goldensDir / (name + ".png");

    gfx::OrbitCamera camera;
    camera.Rotate(mapCase.yawOffset, mapCase.pitchOffset);
    camera.Zoom(mapCase.cameraZoom);
    camera.target = mapCase.cameraTarget;
    camera.Update(1.0e3f);

    tactics::Scene scene = std::move(mapCase.scene);
    scene.units.clear();  // Map only: no figures.
    tactics::GameLogic game(scene);
    tactics::NavMesh navMesh;
    gfx::PaneOverlays overlays;
    tactics::ReachField reach;
    if (mapCase.frontierFrom) {
      navMesh.Build(game.GetScene().obstacles, game.GetScene().mapHalfExtent,
                    tactics::constants::kAgentRadius, &game.GetScene().ground,
                    &game.GetScene().walkSurfaces);
      reach = navMesh.ComputeReachField(*mapCase.frontierFrom, mapCase.frontierBudget);
      overlays.moveFrontier = &reach;
    }
    if (mapCase.showNavMesh) {
      navMesh.Build(game.GetScene().obstacles, game.GetScene().mapHalfExtent,
                    tactics::constants::kAgentRadius, &game.GetScene().ground,
                    &game.GetScene().walkSurfaces);
      overlays.navMeshDebug = &navMesh;
    }
    gfx::RenderDebugOptions debug;
    if (!options.dumpAo.empty()) {
      debug.thermal = true;
      debug.thermalAo.baked = true;
      debug.disableFov = true;
      if (options.smallAo) {
        debug.thermalAo.bake.directions = 16;
        debug.thermalAo.bake.depthResolution = 256;
        debug.thermalAo.bake.spacing = 1.5f;
      }
    }
    renderer.RenderPane(game, tactics::Team::Blue, /*fogActive=*/false, tactics::TeamVisibility{},
                        camera, 0, 0, kWindowWidth, kWindowHeight, overlays, 0, debug);
    const visual::Image image = visual::CaptureFramebuffer(kWindowWidth, kWindowHeight);

    if (!options.dumpAo.empty()) {
      const fs::path directory = options.dumpAo / name;
      if (!DumpAo(renderer.ThermalBake(), directory) ||
          !visual::SavePng(directory / "thermal.png", image)) {
        std::fprintf(stderr, "FAIL %s: AO dump failed\n", name.c_str());
        ++failures;
      }
      continue;
    }

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

  if (rendered == 0) {
    std::fprintf(stderr, "No map matches %s\n", options.mapName.c_str());
    return 1;
  }
  if (failures > 0) {
    std::fprintf(stderr, "%d of %zu map(s) failed.\n", failures, cases.size());
    return 1;
  }
  return 0;
}
