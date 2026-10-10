#pragma once

// Per-pane HUD shared by the interactive app (src/main.cpp) and the headless
// visual scenario runner (tests/visual), so screenshots/videos show exactly
// the UI a player sees. Must be called between ImGui::NewFrame() and
// ImGui::Render(). Const w.r.t. the game: button presses are reported back
// via HudActions for the caller to apply.

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>
#include <imgui.h>

#include "game/GameLogic.h"
#include "game/TurnTimeline.h"
#include "game/Types.h"
#include "gfx/Camera.h"
#include "gfx/Mesh.h"

namespace ui {

constexpr int kPaneCount = 2;

// Pane 0 is the left half of the visual runner's window (Blue), pane 1 the
// right half (Red). Arbitrary but fixed.
inline tactics::Team PaneTeam(int pane) {
  return pane == 0 ? tactics::Team::Blue : tactics::Team::Red;
}
inline const char* TeamName(tactics::Team team) {
  return team == tactics::Team::Blue ? "Blue" : "Red";
}

// Horizontal extent of one team's view within the ImGui display.
struct PaneRect {
  int x = 0;
  int width = 0;
};

inline PaneRect ComputePaneRect(int pane, int windowWidth) {
  const int leftWidth = windowWidth / 2;
  if (pane == 0) return PaneRect{0, leftWidth};
  return PaneRect{leftWidth, windowWidth - leftWidth};
}

// Projects a world point to top-left-origin window pixels, as seen through
// `camera` rendered into `rect` (full window height `windowHeight`).
glm::vec2 WorldToWindow(const glm::vec3& world, const gfx::OrbitCamera& camera,
                        const PaneRect& rect, int windowHeight);

struct HudActions {
  bool newMatch = false;
  bool commit = false;
  bool move = false;
  bool shoot = false;
  bool pass = false;
  bool cancel = false;
  bool done = false;
  // Free-aim (issue #129): Fire confirms the placed aim point.
  bool fire = false;
  // Multi-shot bursts (issue #138): set when the player dragged the shot
  // bar to a new level this frame; route to GameLogic::SetPlannedShotCount.
  std::optional<int> shots;
  // Set when the pane's Playbook view edited its team's squad-wide reaction
  // table; a config edit, not a turn action.
  std::optional<tactics::SquadPlaybook> playbook;
};

// Window-space centers of the HUD buttons actually drawn this frame, keyed
// by their visible label ("Move", "Shoot", "Cancel", "Commit Round", ...).
// Lets the visual scenario runner aim its click marker at the real button a
// player would press.
struct HudLayout {
  std::vector<std::pair<std::string, glm::vec2>> buttons;

  const glm::vec2* FindButton(const char* label) const {
    for (const auto& entry : buttons) {
      if (entry.first == label) return &entry.second;
    }
    return nullptr;
  }
};

// Draws `team`'s view of the HUD inside `rect`: the Round / Game Over panel,
// the floating action menu (when `team` has a figure mid-selection), and the
// team label. Both teams plan simultaneously, so every pane is always live;
// `planning` is false while a round executes or the peer sync is pending.
// When `layout` is non-null it is filled with this frame's button positions.
// When `roundPanelBottom` is non-null it receives the Round panel's bottom edge
// (follows collapse); untouched if the panel isn't drawn (game over).
HudActions DrawHud(const tactics::GameLogic& game, tactics::Team team, bool planning,
                   const PaneRect& rect, int windowHeight, const gfx::OrbitCamera& camera,
                   HudLayout* layout = nullptr, float* roundPanelBottom = nullptr);

// Button presses on the turn-timeline strip (issue #143); like HudActions,
// reported back for the caller to apply to its TimelinePlayback.
struct TimelineActions {
  std::optional<int> seekTick;  // Unused by the HUD (scenarios seek by tick).
  std::optional<size_t> seekFrame;  // Handle dragged here: show that frame.
  bool togglePlay = false;      // Play/Pause pressed.
  bool live = false;            // Back to live (Live pressed / handle at right end).
};

// Draws the turn-timeline strip (issue #143) bottom-centered in `rect`: a
// Play/Pause button, a line with a circle at each turn boundary (Start, T1,
// T2, ...) and a draggable handle that moves freely along it but magnetizes
// to a circle when released/dragged near one, and a "Live" toggle that shows
// pushed in while the live game is showing (handle parked at the right end).
// Dragging the handle to the right end locks Live. Touch-friendly (tall frame padding, wide grab), and
// anchored to the pane's bottom edge so it stays clear of the floating
// action menu and the Round panel. Const w.r.t. the timeline/playback:
// presses are reported back via TimelineActions.
TimelineActions DrawTimeline(const tactics::TurnTimeline& timeline,
                             const tactics::TimelinePlayback& playback, tactics::Team team,
                             const PaneRect& rect, int windowHeight,
                             HudLayout* layout = nullptr);

// Interactive-app-only debug panel (render toggles + FPS). Not called by the
// visual runner. `fps` is a smoothed frames-per-second value; `top` is the
// window-space y to place the panel at (just below the Round panel).
// The extra metrics: `frameMs` last frame time, `fovMs` time spent in the
// visibility (FOV-cone) computation, and `stats`
// draw-call/vertex/triangle counters for the frame.
void DrawDebugPanel(bool& disableFov, bool& disableShadows, bool& thermal, bool& thermalBlackHot,
                    bool& disableOcclusionFade, bool& showFps, float fps, float frameMs,
                    float fovMs, const gfx::RenderFrameStats& stats, float top);

// Thermal view (issue #177): restyles every HUD window drawn in between to
// the monochrome green symbology of real thermal footage, pane team label
// included. Push before the frame's HUD windows, pop after.
void PushThermalHudStyle();
void PopThermalHudStyle();

}  // namespace ui
