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
#include "game/Types.h"
#include "gfx/Camera.h"

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
  bool overwatch = false;
  bool pass = false;
  bool cancel = false;
  // Set when the Playbook popup edited the squad-wide reaction table; a
  // config edit, not a turn action.
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
HudActions DrawHud(const tactics::GameLogic& game, tactics::Team team, bool planning,
                   const PaneRect& rect, int windowHeight, const gfx::OrbitCamera& camera,
                   HudLayout* layout = nullptr);

}  // namespace ui
