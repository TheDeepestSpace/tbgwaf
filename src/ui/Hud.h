#pragma once

// Per-team HUD shared by the interactive app (src/main.cpp) and the headless
// visual scenario runner (tests/visual), so screenshots/videos show exactly
// the UI a player sees. Must be called between ImGui::NewFrame() and
// ImGui::Render(). Const w.r.t. the game: button presses are reported back
// via HudActions for the caller to apply.

#include <optional>

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

// The team currently planning its turn; nullopt once the game is over.
std::optional<tactics::Team> ActiveTeam(const tactics::GameLogic& game);

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
};

// Draws `team`'s view of the HUD inside `rect`: the Turn / Game Over panel,
// the floating action menu (only when `isActive`) and the team label.
HudActions DrawHud(const tactics::GameLogic& game, tactics::Team team, bool isActive,
                   const PaneRect& rect, int windowHeight, const gfx::OrbitCamera& camera);

}  // namespace ui
