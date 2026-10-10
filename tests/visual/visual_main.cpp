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
#include <optional>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "visual/ImageUtil.h"
#include "game/GameLogic.h"
#include "game/TurnTimeline.h"
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

using visual::CaptureFramebuffer;
using visual::CropColumns;
using visual::DiffImages;
using visual::DiffResult;
using visual::Image;
using visual::LoadPng;
using visual::SavePng;

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kVideoFps = 30;
// Frames held on the initial state and after each action so video viewers
// can actually read each turn's outcome (0.5 s at kVideoFps).
constexpr int kHoldFrames = 15;
// Frames recorded while the camera eases over a double-clicked figure (2 s).
constexpr int kFocusGlideFrames = 60;

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
  // Issue #110: force the shadow-map FOV mask for every scenario (normally
  // only scenarios with `render: {fov_overlay: shadow_map}` use it), and/or
  // print per-pane scene render timings (glFinish-bracketed, so they
  // include GPU time -- under llvmpipe that is CPU rasterization time).
  bool forceShadowMapFov = false;
  bool profile = false;
  int pixelThreshold = 25;       // Max per-channel delta still considered "same".
  double maxDiffFraction = 0.002;  // Max fraction of differing pixels still passing.
  std::vector<fs::path> scenarios;
};

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

  // Start from the same map-fitted view as the app. Scenario camera fields
  // are optional adjustments to that baseline. Nothing perturbs the cameras
  // at runtime, so captures are deterministic.
  std::array<gfx::OrbitCamera, ui::kPaneCount> cameras;
  for (auto& camera : cameras) {
    camera.FitToExtent(scenario.scene.mapHalfExtent);
    if (scenario.cameraTarget) {
      camera.target = glm::vec3(scenario.cameraTarget->x, 0.0f, scenario.cameraTarget->y);
    }
    camera.Zoom(scenario.cameraZoom);
    camera.Update(1.0e3f);  // Snap any scenario zoom adjustment.
  }

  const int paneWidth = kWindowWidth / 2;

  // Turn timeline (issue #143): every capture draws the HUD timeline strip,
  // as the live app does (RunScenario hands over its recorder/replay
  // controller via hooks.onTimeline below).
  const tactics::TurnTimeline* timelineView = nullptr;
  const tactics::TimelinePlayback* timelinePlayback = nullptr;

  // Issue #110: the scenario (or --fov-shadow-map) picks the FOV overlay
  // path; restored to the CPU default after the scenario.
  const bool shadowMapFov =
      options.forceShadowMapFov || scenario.fovOverlay == tactics::scenario::Scenario::FovOverlay::ShadowMap;
  renderer.SetFovOverlayMode(shadowMapFov ? gfx::FovOverlayMode::ShadowMap
                                          : gfx::FovOverlayMode::CpuAnalytic);
  renderer.SetFovProbeHeight(scenario.fovProbeHeight);
  // --profile: per-pane RenderPane wall time (glFinish before and after)
  // and the number of own living units whose cones were drawn, so the cost
  // per pane and per unit can be read off.
  struct PaneProfile {
    double totalMs = 0.0;
    int frames = 0;
    int unitCones = 0;
  };
  std::array<PaneProfile, ui::kPaneCount> profiles;

  // Optional cursor marker: `progress` runs 0 -> 1 as the ring closes in.
  // Either a world-space click (figure/ground, projected through the pane's
  // camera) or, when `button` is set, a press of the named HUD button at
  // whatever screen position the HUD reports for it this frame.
  struct ClickMarker {
    Team team;
    glm::vec3 world{0.0f};
    float progress = 0.0f;
    const char* button = nullptr;
  };
  // Panes' overlays come from the same BuildPaneOverlays the interactive app
  // uses. Draws both 3D panes, then the same HUD the interactive app builds
  // (ui::DrawHud), then the click marker on the acting team's pane.
  auto renderBothPanes = [&](const GameLogic& game, const ClickMarker* marker = nullptr) {
    const bool fogActive = game.Mode() != InputMode::GameOver;
    for (int pane = 0; pane < 2; ++pane) {
      const Team team = PaneTeam(pane);
      TeamVisibility visibility;
      if (fogActive) visibility = game.ComputeVisibility(team);
      const gfx::PaneOverlays overlays = gfx::BuildPaneOverlays(game, team);
      if (options.profile) {
        glFinish();
        const auto start = std::chrono::steady_clock::now();
        renderer.RenderPane(game, team, fogActive, visibility, cameras[pane], pane * paneWidth,
                            0, paneWidth, kWindowHeight, overlays);
        glFinish();
        const auto end = std::chrono::steady_clock::now();
        PaneProfile& p = profiles[pane];
        p.totalMs += std::chrono::duration<double, std::milli>(end - start).count();
        ++p.frames;
        for (const tactics::Unit& unit : game.GetScene().units) {
          if (unit.alive && unit.team == team) ++p.unitCones;
        }
      } else {
        renderer.RenderPane(game, team, fogActive, visibility, cameras[pane], pane * paneWidth, 0,
                            paneWidth, kWindowHeight, overlays);
      }
    }

    ImGuiIO& io = ImGui::GetIO();
    io.DisplaySize = ImVec2(static_cast<float>(kWindowWidth), static_cast<float>(kWindowHeight));
    io.DeltaTime = 1.0f / kVideoFps;
    // Each team's pane gets its own HUD, as in its own browser tab. Button
    // positions are collected per pane so a menu-click marker can land on
    // the actual button a player would press.
    const bool planning = game.Mode() != InputMode::Executing && game.Mode() != InputMode::GameOver;
    std::array<ui::HudLayout, ui::kPaneCount> layouts;
    auto drawHud = [&](const GameLogic& g) {
      for (int pane = 0; pane < ui::kPaneCount; ++pane) {
        layouts[pane] = ui::HudLayout{};
        ui::DrawHud(g, PaneTeam(pane), planning, ui::ComputePaneRect(pane, kWindowWidth),
                    kWindowHeight, cameras[pane], &layouts[pane]);
        if (timelineView && timelinePlayback) {
          ui::DrawTimeline(*timelineView, *timelinePlayback, PaneTeam(pane),
                           ui::ComputePaneRect(pane, kWindowWidth), kWindowHeight,
                           &layouts[pane]);
        }
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
      const int pane = marker->team == Team::Blue ? 0 : 1;
      glm::vec2 p;
      bool haveTarget = true;
      if (marker->button) {
        // HUD button press: use the position the HUD reported this frame.
        // If the button is not on screen (unexpected), skip the marker but
        // still emit the frame so timing stays intact.
        if (const glm::vec2* center = layouts[pane].FindButton(marker->button)) {
          p = *center;
        } else {
          haveTarget = false;
        }
      } else {
        p = ui::WorldToWindow(marker->world, cameras[pane],
                              ui::ComputePaneRect(pane, kWindowWidth), kWindowHeight);
      }
      if (haveTarget) {
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
  // Frames fed to the (per-team, frame-identical) encoders so far, and the
  // frame count at each completed action: both panes advance in lockstep, so
  // one counter indexes either video. Written out as timing.json next to the
  // webm files for the review page's breakdown sync / click-to-seek.
  int framesWritten = 0;
  std::vector<std::pair<int, int>> actionFrames;  // (completedActions, framesWritten)
  // Wall-clock ms spent producing each captured frame (render + readback), in
  // frame order: the encoded rate is constant, this is what varies.
  std::vector<double> frameMs;
  tactics::scenario::PlaybackHooks hooks;
  // Declared outside the `if`: the hooks below outlive its scope.
  auto writeVideoFrame = [&](const GameLogic& game, const ClickMarker* marker) {
    const Uint64 t0 = SDL_GetPerformanceCounter();
    renderBothPanes(game, marker);
    const Image frame = CaptureFramebuffer(kWindowWidth, kWindowHeight);
    frameMs.push_back(1000.0 * static_cast<double>(SDL_GetPerformanceCounter() - t0) /
                      static_cast<double>(SDL_GetPerformanceFrequency()));
    for (int pane = 0; pane < 2; ++pane) {
      if (!encoders[pane].WriteFrame(CropColumns(frame, pane * paneWidth, paneWidth))) {
        videoOk = false;
      }
    }
    ++framesWritten;
  };
  if (options.video && videoOk) {
    hooks.tickSeconds = 1.0f / kVideoFps;
    hooks.holdFramesAfterAction = kHoldFrames;
    hooks.onFrame = [&](const GameLogic& game) { writeVideoFrame(game, nullptr); };
    // Show the cursor landing on the figure/ground point before the click
    // takes effect, instead of jump-cutting between states.
    hooks.onClick = [&](const GameLogic& game, Team team, const glm::vec3& worldPoint) {
      for (int i = 0; i < kClickFrames; ++i) {
        const float progress = std::min(1.0f, static_cast<float>(i + 1) / (kClickFrames * 0.7f));
        const ClickMarker marker{team, worldPoint, progress};
        writeVideoFrame(game, &marker);
      }
    };
    // Same treatment for HUD action-menu presses (Move/Shoot/Pass/Cancel):
    // the menu is visible in the pre-press state, so the marker closes in on
    // the actual button before the choice takes effect.
    hooks.onMenuClick = [&](const GameLogic& game, Team team, const char* button) {
      for (int i = 0; i < kClickFrames; ++i) {
        ClickMarker marker;
        marker.team = team;
        marker.progress = std::min(1.0f, static_cast<float>(i + 1) / (kClickFrames * 0.7f));
        marker.button = button;
        writeVideoFrame(game, &marker);
      }
    };
  }
  if (options.video && videoOk) {
    hooks.onMoveFrontier = [&](const GameLogic& game, Team) { writeVideoFrame(game, nullptr); };
  }
  // Double-click on a figure: ease that team's pane camera over the unit's
  // movement frontier (same FocusOn call as main.cpp). Video mode records
  // the glide; either way the camera is then settled so the post-action
  // golden is the converged framing.
  hooks.onFocus = [&](const GameLogic& game, int unitId, Team team) {
    const tactics::Unit* unit = game.FindUnit(unitId);
    if (!unit) return;
    gfx::OrbitCamera& camera = cameras[team == Team::Blue ? 0 : 1];
    camera.FocusOn(unit->position,
                   unit->MoveBudget() + 2.0f * tactics::constants::kAgentRadius);
    if (options.video && videoOk) {
      for (int i = 0; i < kFocusGlideFrames; ++i) {
        camera.Update(1.0f / kVideoFps);
        writeVideoFrame(game, nullptr);
      }
    }
    camera.Update(1.0e3f);
  };
  hooks.onTimeline = [&](const tactics::TurnTimeline& timeline,
                         const tactics::TimelinePlayback& playback) {
    timelineView = &timeline;
    timelinePlayback = &playback;
  };
  hooks.onActionComplete = [&](const GameLogic& game, int turn) {
    // The holds for this action have already been emitted, so framesWritten
    // is the exclusive end of the action's video segment (turn 0 = the
    // initial-state hold).
    if (options.video && videoOk) actionFrames.emplace_back(turn, framesWritten);
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

  if (options.profile) {
    for (int pane = 0; pane < ui::kPaneCount; ++pane) {
      const PaneProfile& p = profiles[pane];
      if (p.frames == 0) continue;
      std::printf("PROFILE %s %s fov=%s panes=%d avg_ms=%.2f avg_units=%.2f\n", stem.c_str(),
                  TeamName(PaneTeam(pane)), shadowMapFov ? "shadow_map" : "cpu", p.frames,
                  p.totalMs / p.frames, static_cast<double>(p.unitCones) / p.frames);
    }
  }
  renderer.SetFovOverlayMode(gfx::FovOverlayMode::CpuAnalytic);
  renderer.SetFovProbeHeight(0.0f);

  if (options.video) {
    for (auto& encoder : encoders) {
      if (!encoder.Close()) videoOk = false;
    }
    if (!videoOk) {
      std::fprintf(stderr, "FAIL %s: video encoding failed (is ffmpeg installed?)\n",
                   stem.c_str());
    } else {
      // Sidecar mapping each completed action to its segment-end frame in
      // either team's video (both are frame-identical in time). A missing or
      // unwritable sidecar only degrades the review page's sync, so it does
      // not fail the scenario.
      const fs::path timingPath = options.outDir / stem / "timing.json";
      fs::create_directories(timingPath.parent_path());
      if (FILE* f = std::fopen(timingPath.string().c_str(), "w")) {
        std::fprintf(f, "{\"fps\": %d, \"actions\": [", kVideoFps);
        for (size_t i = 0; i < actionFrames.size(); ++i) {
          std::fprintf(f, "%s{\"index\": %d, \"frame\": %d, \"time\": %.3f}", i ? ", " : "",
                       actionFrames[i].first, actionFrames[i].second,
                       static_cast<double>(actionFrames[i].second) / kVideoFps);
        }
        // Per-frame production cost for the review page's timing graph.
        std::fprintf(f, "], \"frames\": [");
        for (size_t i = 0; i < frameMs.size(); ++i) {
          std::fprintf(f, "%s{\"frame\": %zu, \"ms\": %.2f}", i ? ", " : "", i + 1, frameMs[i]);
        }
        std::fprintf(f, "]}\n");
        std::fclose(f);
      } else {
        std::fprintf(stderr, "WARN %s: could not write %s\n", stem.c_str(),
                     timingPath.string().c_str());
      }
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
    } else if (arg == "--fov-shadow-map") {
      options->forceShadowMapFov = true;
    } else if (arg == "--profile") {
      options->profile = true;
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
          "         [--fov-shadow-map] [--profile]\n"
          "         [--goldens-dir DIR] [--out-dir DIR] [--pixel-threshold N]\n"
          "         [--max-diff-fraction F] [scenario.yaml ...]\n"
          "Runs all scenarios under tests/scenarios/ when none are listed.\n"
          "--fov-shadow-map forces the issue #110 shadow-map FOV mask for every\n"
          "scenario (goldens then won't match); --profile prints per-pane render\n"
          "timings.\n");
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
