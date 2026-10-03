// Gallery golden tests (issue #126): renders every item of the asset &
// animation gallery (src/gallery/GalleryScene) at fixed, documented sample
// points and diffs each frame against a golden under tests/gallery_goldens/.
// Weapon turntables are sampled at two fixed rotations (both sides), and
// each animation at three fixed times across its loop, so a change to a
// weapon model or an animation pose shows up as a golden diff.
//
// Same headless xvfb + software-GL setup and pixelmatch-style tolerance as
// the other visual runners; `--update-baselines` regenerates the goldens.

#include <SDL.h>
#include <GLES3/gl3.h>

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

#include "visual/ImageUtil.h"
#include "gallery/GalleryScene.h"

namespace fs = std::filesystem;

namespace {

constexpr int kWindowWidth = 640;
constexpr int kWindowHeight = 480;
constexpr int kPixelThreshold = 25;
constexpr double kMaxDiffFraction = 0.002;

struct Options {
  fs::path goldensDir = "tests/gallery_goldens";
  fs::path outDir = "gallery_visual_out";
  bool updateBaselines = false;
};

// Loop times at which an item is snapshotted. Keep in sync with the
// "Asset & animation gallery" section of README.md.
std::vector<double> SampleTimes(const gallery::GalleryItem& item) {
  if (item.kind == gallery::ItemKind::WeaponModel) {
    // Turntable rotations of 30 and 210 degrees: both sides of the weapon.
    return {item.duration * (30.0 / 360.0), item.duration * (210.0 / 360.0)};
  }
  // Start, one-third, and two-thirds of the animation loop.
  return {0.0, item.duration / 3.0, item.duration * 2.0 / 3.0};
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
    } else {
      std::fprintf(stderr,
                   "usage: tactics_gallery_visual_tests [--update-baselines] "
                   "[--goldens-dir DIR] [--out-dir DIR]\n");
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
      SDL_CreateWindow("tbgwaf gallery tests", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
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

  gallery::GalleryRenderer renderer;
  if (!renderer.Init()) {
    std::fprintf(stderr, "gallery renderer init failed\n");
    return 1;
  }

  int failures = 0;
  const auto& catalog = gallery::Catalog();
  for (size_t i = 0; i < catalog.size(); ++i) {
    const gallery::GalleryItem& item = catalog[i];
    const gallery::ViewState view = gallery::DefaultView(static_cast<int>(i));
    const std::vector<double> times = SampleTimes(item);
    for (size_t frame = 0; frame < times.size(); ++frame) {
      char name[128];
      std::snprintf(name, sizeof(name), "%s_%02zu", item.id, frame);
      const fs::path goldenPath = options.goldensDir / (std::string(name) + ".png");

      renderer.Render(static_cast<int>(i), times[frame], view, kWindowWidth, kWindowHeight);
      const visual::Image image = visual::CaptureFramebuffer(kWindowWidth, kWindowHeight);

      if (options.updateBaselines) {
        if (!visual::SavePng(goldenPath, image)) {
          std::fprintf(stderr, "FAIL %s: could not write %s\n", name,
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
                     name, goldenPath.string().c_str());
        ++failures;
        continue;
      }
      const visual::DiffResult diff = visual::DiffImages(golden, image, kPixelThreshold);
      if (diff.sizeMismatch || diff.Fraction() > kMaxDiffFraction) {
        visual::SavePng(options.outDir / (std::string(name) + ".actual.png"), image);
        if (!diff.sizeMismatch) {
          visual::SavePng(options.outDir / (std::string(name) + ".diff.png"), diff.diffImage);
        }
        std::fprintf(stderr, "FAIL %s: %s (actual/diff written under %s)\n", name,
                     diff.sizeMismatch
                         ? "golden has different dimensions"
                         : (std::to_string(diff.Fraction() * 100.0) + "% of pixels differ").c_str(),
                     options.outDir.string().c_str());
        ++failures;
      } else {
        std::printf("PASS %s\n", name);
      }
    }
  }

  renderer.Destroy();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();

  if (failures > 0) {
    std::fprintf(stderr, "%d gallery frame(s) failed.\n", failures);
    return 1;
  }
  return 0;
}
