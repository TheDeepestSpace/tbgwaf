// Visual gameplay scenario runner (issue #14 stages 2+3): replays the same
// YAML scenarios as the cheap/logic-only `scenario_tests` target, but with
// rendering. For every "turn" (each executed script action, plus the
// initial state) it draws both teams' fog-of-war panes through the exact
// same gfx::SceneRenderer pass the interactive app uses, and:
//
//  - screenshot mode (default): captures one PNG per team per turn and
//    diffs it against a checked-in golden under tests/goldens/ with a
//    pixelmatch-style tolerance (per-channel threshold + max fraction of
//    differing pixels), absorbing driver/AA noise without exact matching.
//    `--update-baselines` regenerates the goldens instead of comparing.
//  - `--video`: additionally records one continuous full-scenario video per
//    team (move animations advanced at a fixed 30 fps tick, with short
//    holds on each turn's outcome), piped as raw frames to ffmpeg.
//
// Meant to run headless under xvfb + LIBGL_ALWAYS_SOFTWARE=1, like the
// existing smoke_test. See CMakeLists.txt (visual_tests) and
// .github/workflows/pr-preview.yml for the CI wiring.

#include <SDL.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <string>
#include <vector>

#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_STATIC
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#include <stb_image_write.h>

#include "game/GameLogic.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"
#include "scenario/Scenario.h"
#include "ui/Hud.h"

namespace fs = std::filesystem;

using tactics::GameLogic;
using tactics::InputMode;
using tactics::Team;
using tactics::TeamVisibility;

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kVideoFps = 30;
// Frames held on the initial state and after each action so video viewers
// can actually read each turn's outcome (0.5 s at kVideoFps).
constexpr int kHoldFrames = 15;
// Frames the click marker is shown before each scripted click lands (video
// only): a ring contracting onto the click point, then a brief hold.
constexpr int kClickFrames = 12;

// Pane 0 is the left half (Blue), pane 1 the right half (Red) -- same
// arbitrary-but-fixed assignment as the interactive app.
using ui::PaneTeam;
const char* TeamName(Team team) { return team == Team::Blue ? "blue" : "red"; }

struct Options {
  fs::path goldensDir = "tests/goldens";
  fs::path outDir = "visual_out";
  bool updateBaselines = false;
  bool skipGoldens = false;
  bool video = false;
  int pixelThreshold = 25;       // Max per-channel delta still considered "same".
  double maxDiffFraction = 0.002;  // Max fraction of differing pixels still passing.
  std::vector<fs::path> scenarios;
};

struct Image {
  int width = 0;
  int height = 0;
  std::vector<unsigned char> rgb;  // Row-major, top-to-bottom, 3 bytes/pixel.
};

// Reads the current back buffer. GL's origin is bottom-left, so rows are
// flipped into the top-to-bottom order PNG/video expect. RGBA readback is
// the only combination ES 3.0 guarantees; converted to RGB here.
Image CaptureFramebuffer(int width, int height) {
  std::vector<unsigned char> rgba(static_cast<size_t>(width) * height * 4);
  glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data());

  Image image;
  image.width = width;
  image.height = height;
  image.rgb.resize(static_cast<size_t>(width) * height * 3);
  for (int y = 0; y < height; ++y) {
    const unsigned char* src = rgba.data() + static_cast<size_t>(height - 1 - y) * width * 4;
    unsigned char* dst = image.rgb.data() + static_cast<size_t>(y) * width * 3;
    for (int x = 0; x < width; ++x) {
      dst[x * 3 + 0] = src[x * 4 + 0];
      dst[x * 3 + 1] = src[x * 4 + 1];
      dst[x * 3 + 2] = src[x * 4 + 2];
    }
  }
  return image;
}

Image CropColumns(const Image& image, int x, int width) {
  Image out;
  out.width = width;
  out.height = image.height;
  out.rgb.resize(static_cast<size_t>(width) * image.height * 3);
  for (int y = 0; y < image.height; ++y) {
    std::memcpy(out.rgb.data() + static_cast<size_t>(y) * width * 3,
                image.rgb.data() + (static_cast<size_t>(y) * image.width + x) * 3,
                static_cast<size_t>(width) * 3);
  }
  return out;
}

bool SavePng(const fs::path& path, const Image& image) {
  fs::create_directories(path.parent_path());
  return stbi_write_png(path.string().c_str(), image.width, image.height, 3, image.rgb.data(),
                        image.width * 3) != 0;
}

bool LoadPng(const fs::path& path, Image* out) {
  int w = 0, h = 0, channels = 0;
  unsigned char* data = stbi_load(path.string().c_str(), &w, &h, &channels, 3);
  if (!data) return false;
  out->width = w;
  out->height = h;
  out->rgb.assign(data, data + static_cast<size_t>(w) * h * 3);
  stbi_image_free(data);
  return true;
}

struct DiffResult {
  bool sizeMismatch = false;
  long long differingPixels = 0;
  long long totalPixels = 0;
  Image diffImage;  // Golden as dimmed grayscale, differing pixels in red.

  double Fraction() const {
    return totalPixels == 0 ? 1.0 : static_cast<double>(differingPixels) / totalPixels;
  }
};

// Pixelmatch-style tolerance diff: a pixel counts as differing only if some
// channel deviates by more than `threshold`, and the image only fails if
// the differing fraction exceeds the caller's budget. Absorbs cross-driver
// shading/edge noise while still catching object-sized changes (a single
// figure is roughly 0.3% of a pane).
DiffResult DiffImages(const Image& golden, const Image& actual, int threshold) {
  DiffResult result;
  if (golden.width != actual.width || golden.height != actual.height) {
    result.sizeMismatch = true;
    return result;
  }
  result.totalPixels = static_cast<long long>(golden.width) * golden.height;
  result.diffImage.width = golden.width;
  result.diffImage.height = golden.height;
  result.diffImage.rgb.resize(golden.rgb.size());
  for (long long i = 0; i < result.totalPixels; ++i) {
    const unsigned char* g = golden.rgb.data() + i * 3;
    const unsigned char* a = actual.rgb.data() + i * 3;
    const int delta = std::max({std::abs(g[0] - a[0]), std::abs(g[1] - a[1]),
                                std::abs(g[2] - a[2])});
    unsigned char* d = result.diffImage.rgb.data() + i * 3;
    if (delta > threshold) {
      ++result.differingPixels;
      d[0] = 255;
      d[1] = 0;
      d[2] = 0;
    } else {
      const unsigned char gray = static_cast<unsigned char>((g[0] + g[1] + g[2]) / 3 / 2);
      d[0] = d[1] = d[2] = gray;
    }
  }
  return result;
}

// One ffmpeg process fed raw RGB frames over a pipe, encoding a browser-
// playable VP9 WebM (the PR review page embeds these directly). Realtime
// deadline keeps VP9 encoding fast enough for CI; -b:v 0 makes -crf a pure
// constant-quality target.
class VideoEncoder {
 public:
  bool Open(const fs::path& outPath, int width, int height) {
    fs::create_directories(outPath.parent_path());
    char cmd[1024];
    std::snprintf(cmd, sizeof(cmd),
                  "ffmpeg -loglevel error -y -f rawvideo -pixel_format rgb24 "
                  "-video_size %dx%d -framerate %d -i - -c:v libvpx-vp9 -b:v 0 "
                  "-crf 32 -deadline realtime -cpu-used 5 -row-mt 1 "
                  "-pix_fmt yuv420p \"%s\"",
                  width, height, kVideoFps, outPath.string().c_str());
    pipe_ = popen(cmd, "w");
    frameBytes_ = static_cast<size_t>(width) * height * 3;
    return pipe_ != nullptr;
  }

  bool WriteFrame(const Image& frame) {
    if (!pipe_ || frame.rgb.size() != frameBytes_) return false;
    return std::fwrite(frame.rgb.data(), 1, frameBytes_, pipe_) == frameBytes_;
  }

  // Returns false if ffmpeg exited non-zero (e.g. it's not installed).
  bool Close() {
    if (!pipe_) return true;
    const int status = pclose(pipe_);
    pipe_ = nullptr;
    return status == 0;
  }

 private:
  FILE* pipe_ = nullptr;
  size_t frameBytes_ = 0;
};

// Golden/output directories are keyed by the scenario *file stem* (not the
// YAML `name` field): stable under renames of the display name, unique
// within tests/scenarios/, and derivable for deleted scenarios too.
std::string SanitizedStem(const fs::path& path) {
  std::string stem = path.stem().string();
  for (char& c : stem) {
    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-' && c != '_') c = '_';
  }
  return stem;
}

struct ScenarioRunStats {
  int failures = 0;
  int comparedImages = 0;
  int updatedImages = 0;
};

void RunOneScenario(const fs::path& file, const Options& options, gfx::SceneRenderer& renderer,
                    ScenarioRunStats* stats) {
  const std::string stem = SanitizedStem(file);

  tactics::scenario::Scenario scenario;
  try {
    scenario = tactics::scenario::LoadScenarioFromFile(file.string());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL %s: %s\n", file.string().c_str(), e.what());
    ++stats->failures;
    return;
  }

  // Fixed default cameras, identical to the app's startup view (each pane
  // is half-width, so start zoomed out enough for both spawns to fit).
  // Nothing perturbs them at runtime, so captures are deterministic.
  std::array<gfx::OrbitCamera, ui::kPaneCount> cameras;
  for (auto& camera : cameras) {
    camera.Zoom(10.0f);
    camera.Update(1.0e3f);  // Snap to the initial zoom (matches main.cpp).
  }

  const int paneWidth = kWindowWidth / 2;
  // Optional cursor marker: `progress` runs 0 -> 1 as the ring closes in.
  struct ClickMarker {
    glm::vec3 world;
    float progress;
  };
  // Draws both 3D panes, then the same HUD the interactive app builds
  // (ui::DrawHud), then the click marker on the acting team's pane.
  auto renderBothPanes = [&](const GameLogic& game, const ClickMarker* marker = nullptr) {
    const bool fogActive = game.Mode() != InputMode::GameOver;
    for (int pane = 0; pane < 2; ++pane) {
      const Team team = PaneTeam(pane);
      TeamVisibility visibility;
      if (fogActive) visibility = game.ComputeVisibility(team);
      renderer.RenderPane(game, team, fogActive, visibility, cameras[pane], pane * paneWidth, 0,
                          paneWidth, kWindowHeight);
    }

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(kWindowWidth), static_cast<float>(kWindowHeight));
    io.DeltaTime = 1.0f / kVideoFps;
    // Each team's pane gets its own HUD, as in its own browser tab.
    auto drawHud = [&](const GameLogic& g) {
      const auto activeTeam = ui::ActiveTeam(g);
      for (int pane = 0; pane < ui::kPaneCount; ++pane) {
        const Team team = PaneTeam(pane);
        ui::DrawHud(g, team, activeTeam && *activeTeam == team,
                    ui::ComputePaneRect(pane, kWindowWidth), kWindowHeight, cameras[pane]);
      }
    };
    // Auto-resize windows need a couple of frames to settle on their content
    // size (and stay hidden meanwhile), as they would in the live app; run
    // throw-away frames first so every capture is fully laid out.
    for (int warmup = 0; warmup < 3; ++warmup) {
      ImGui_ImplOpenGL3_NewFrame();
      ImGui::NewFrame();
      drawHud(game);
      ImGui::EndFrame();
    }
    ImGui_ImplOpenGL3_NewFrame();
    ImGui::NewFrame();
    drawHud(game);
    if (marker) {
      if (const auto team = ui::ActiveTeam(game)) {
        const int pane = *team == Team::Blue ? 0 : 1;
        const glm::vec2 p = ui::WorldToWindow(marker->world, cameras[pane],
                                              ui::ComputePaneRect(pane, kWindowWidth),
                                              kWindowHeight);
        ImDrawList* draw = ImGui::GetForegroundDrawList();
        const ImVec2 c(p.x, p.y);
        const float radius = 18.0f + 60.0f * (1.0f - marker->progress);
        const int alpha = static_cast<int>(140 + 115 * marker->progress);
        // Dark under-stroke keeps the ring readable against any background.
        draw->AddCircle(c, radius, IM_COL32(0, 0, 0, alpha), 48, 9.0f);
        draw->AddCircle(c, radius, IM_COL32(255, 220, 40, alpha), 48, 5.0f);
        draw->AddCircleFilled(c, 7.0f, IM_COL32(0, 0, 0, 255));
        draw->AddCircleFilled(c, 5.0f, IM_COL32(255, 220, 40, 255));
        // Arrow cursor with its tip on the click point.
        const ImVec2 b(c.x + 20, c.y + 36), r(c.x + 36, c.y + 24);
        draw->AddTriangleFilled(c, b, r, IM_COL32(255, 255, 255, 255));
        draw->AddTriangle(c, b, r, IM_COL32(0, 0, 0, 255), 2.5f);
      }
    }
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  };

  std::array<VideoEncoder, 2> encoders;
  bool videoOk = true;
  if (options.video) {
    for (int pane = 0; pane < 2; ++pane) {
      const fs::path videoPath =
          options.outDir / stem / (std::string(TeamName(PaneTeam(pane))) + ".webm");
      if (!encoders[pane].Open(videoPath, paneWidth, kWindowHeight)) {
        std::fprintf(stderr, "FAIL %s: could not start ffmpeg for %s\n", stem.c_str(),
                     videoPath.string().c_str());
        videoOk = false;
      }
    }
  }

  int imageFailures = 0;
  tactics::scenario::PlaybackHooks hooks;
  // Declared outside the `if`: the hooks below outlive its scope.
  auto writeVideoFrame = [&](const GameLogic& game, const ClickMarker* marker) {
    renderBothPanes(game, marker);
    const Image frame = CaptureFramebuffer(kWindowWidth, kWindowHeight);
    for (int pane = 0; pane < 2; ++pane) {
      if (!encoders[pane].WriteFrame(CropColumns(frame, pane * paneWidth, paneWidth))) {
        videoOk = false;
      }
    }
  };
  if (options.video && videoOk) {
    hooks.tickSeconds = 1.0f / kVideoFps;
    hooks.holdFramesAfterAction = kHoldFrames;
    hooks.onFrame = [&](const GameLogic& game) { writeVideoFrame(game, nullptr); };
    // Show the cursor landing on the figure/ground point before the click
    // takes effect, instead of jump-cutting between states.
    hooks.onClick = [&](const GameLogic& game, const glm::vec3& worldPoint) {
      for (int i = 0; i < kClickFrames; ++i) {
        const float progress = std::min(1.0f, static_cast<float>(i + 1) / (kClickFrames * 0.7f));
        const ClickMarker marker{worldPoint, progress};
        writeVideoFrame(game, &marker);
      }
    };
  }
  hooks.onActionComplete = [&](const GameLogic& game, int turn) {
    renderBothPanes(game);
    const Image frame = CaptureFramebuffer(kWindowWidth, kWindowHeight);
    for (int pane = 0; pane < 2; ++pane) {
      const Team team = PaneTeam(pane);
      const Image paneImage = CropColumns(frame, pane * paneWidth, paneWidth);
      char shot[64];
      std::snprintf(shot, sizeof(shot), "turn_%02d_%s.png", turn, TeamName(team));
      const fs::path goldenPath = options.goldensDir / stem / shot;

      if (options.updateBaselines) {
        if (SavePng(goldenPath, paneImage)) {
          ++stats->updatedImages;
        } else {
          std::fprintf(stderr, "FAIL %s: could not write golden %s\n", stem.c_str(),
                       goldenPath.string().c_str());
          ++imageFailures;
        }
        continue;
      }
      if (options.skipGoldens) continue;

      ++stats->comparedImages;
      Image golden;
      if (!LoadPng(goldenPath, &golden)) {
        std::fprintf(stderr,
                     "FAIL %s %s: golden %s missing/unreadable (run "
                     "scripts/update_golden_baselines.sh to regenerate)\n",
                     stem.c_str(), shot, goldenPath.string().c_str());
        ++imageFailures;
        continue;
      }
      const DiffResult diff = DiffImages(golden, paneImage, options.pixelThreshold);
      if (diff.sizeMismatch || diff.Fraction() > options.maxDiffFraction) {
        const fs::path actualPath = options.outDir / stem / (std::string(shot) + ".actual.png");
        SavePng(actualPath, paneImage);
        if (!diff.sizeMismatch) {
          SavePng(options.outDir / stem / (std::string(shot) + ".diff.png"), diff.diffImage);
        }
        const std::string reason =
            diff.sizeMismatch
                ? std::string("golden has different dimensions")
                : std::to_string(diff.Fraction() * 100.0) + "% of pixels differ";
        std::fprintf(stderr,
                     "FAIL %s %s: %s (threshold %d, budget %.4f%%); actual/diff written under "
                     "%s\n",
                     stem.c_str(), shot, reason.c_str(), options.pixelThreshold,
                     options.maxDiffFraction * 100.0, (options.outDir / stem).string().c_str());
        ++imageFailures;
      }
    }
  };

  const auto result = tactics::scenario::RunScenario(scenario, hooks);

  if (options.video) {
    for (auto& encoder : encoders) {
      if (!encoder.Close()) videoOk = false;
    }
    if (!videoOk) {
      std::fprintf(stderr, "FAIL %s: video encoding failed (is ffmpeg installed?)\n",
                   stem.c_str());
    }
  }

  const bool logicPassed = result.Passed();
  if (!logicPassed) {
    std::fprintf(stderr, "FAIL %s (%s): scenario script failed during visual playback:\n",
                 scenario.name.c_str(), file.string().c_str());
    for (const auto& msg : result.failures) std::fprintf(stderr, "  %s\n", msg.c_str());
  }
  if (!logicPassed || imageFailures > 0 || (options.video && !videoOk)) {
    ++stats->failures;
  } else {
    std::printf("PASS %s (%s)\n", scenario.name.c_str(), file.string().c_str());
  }
}

bool ParseArgs(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto NextValue = [&](const char* flag) -> const char* {
      if (i + 1 >= argc) {
        std::fprintf(stderr, "%s requires a value\n", flag);
        return nullptr;
      }
      return argv[++i];
    };
    if (arg == "--goldens-dir") {
      const char* v = NextValue("--goldens-dir");
      if (!v) return false;
      options->goldensDir = v;
    } else if (arg == "--out-dir") {
      const char* v = NextValue("--out-dir");
      if (!v) return false;
      options->outDir = v;
    } else if (arg == "--update-baselines") {
      options->updateBaselines = true;
    } else if (arg == "--skip-goldens") {
      options->skipGoldens = true;
    } else if (arg == "--video") {
      options->video = true;
    } else if (arg == "--pixel-threshold") {
      const char* v = NextValue("--pixel-threshold");
      if (!v) return false;
      options->pixelThreshold = std::atoi(v);
    } else if (arg == "--max-diff-fraction") {
      const char* v = NextValue("--max-diff-fraction");
      if (!v) return false;
      options->maxDiffFraction = std::atof(v);
    } else if (arg == "--help" || arg == "-h") {
      std::printf(
          "usage: tactics_visual_tests [--update-baselines] [--skip-goldens] [--video]\n"
          "         [--goldens-dir DIR] [--out-dir DIR] [--pixel-threshold N]\n"
          "         [--max-diff-fraction F] [scenario.yaml ...]\n"
          "Runs all scenarios under tests/scenarios/ when none are listed.\n");
      return false;
    } else if (!arg.empty() && arg[0] == '-') {
      std::fprintf(stderr, "unknown flag %s (try --help)\n", arg.c_str());
      return false;
    } else {
      options->scenarios.push_back(arg);
    }
  }

  if (options->scenarios.empty()) {
    const fs::path scenarioDir = "tests/scenarios";
    if (!fs::is_directory(scenarioDir)) {
      std::fprintf(stderr, "no scenarios listed and %s not found\n",
                   scenarioDir.string().c_str());
      return false;
    }
    for (const auto& entry : fs::directory_iterator(scenarioDir)) {
      if (!entry.is_regular_file()) continue;
      const auto ext = entry.path().extension();
      if (ext == ".yaml" || ext == ".yml") options->scenarios.push_back(entry.path());
    }
    std::sort(options->scenarios.begin(), options->scenarios.end());
  }
  if (options->scenarios.empty()) {
    std::fprintf(stderr, "no scenario YAML files to run\n");
    return false;
  }
  return true;
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseArgs(argc, argv, &options)) return 1;

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
  // Same GL setup as the interactive app so the captures exercise the
  // identical context configuration (ES 3.0 over EGL, depth+stencil).
  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_Window* window =
      SDL_CreateWindow("tbgwaf visual tests", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
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
    // Captures/goldens are only comparable at the fixed reference size.
    std::fprintf(stderr, "drawable size %dx%d != required %dx%d\n", drawableWidth, drawableHeight,
                 kWindowWidth, kWindowHeight);
    return 1;
  }

  // ImGui without the SDL backend: no input, we set DisplaySize/DeltaTime
  // ourselves per frame, so the HUD renders deterministically.
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui::GetIO().IniFilename = nullptr;
  ImGui_ImplOpenGL3_Init("#version 300 es");

  gfx::SceneRenderer renderer;
  if (!renderer.Init()) return 1;

  ScenarioRunStats stats;
  for (const auto& file : options.scenarios) {
    RunOneScenario(file, options, renderer, &stats);
  }

  renderer.Destroy();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();

  if (options.updateBaselines) {
    std::printf("Updated %d golden image(s) under %s\n", stats.updatedImages,
                options.goldensDir.string().c_str());
  } else if (!options.skipGoldens) {
    std::printf("Compared %d golden image(s)\n", stats.comparedImages);
  }
  if (stats.failures > 0) {
    std::fprintf(stderr, "%d of %zu scenario(s) failed.\n", stats.failures,
                 options.scenarios.size());
    return 1;
  }
  std::printf("All %zu scenario(s) passed.\n", options.scenarios.size());
  return 0;
}
