#pragma once

// Split-screen HUD shared by the interactive app (src/main.cpp) and the
// headless visual scenario runner (tests/visual), so screenshots/videos show
// exactly the UI a player sees. Must be called between ImGui::NewFrame() and
// ImGui::Render(). Const w.r.t. the game: button presses are reported back
// via HudActions for the caller to apply.

#include <array>
#include <optional>

#include <glm/glm.hpp>
#include <imgui.h>

#include "game/GameLogic.h"
#include "game/Types.h"
#include "gfx/Camera.h"

namespace ui {

constexpr int kPaneCount = 2;

// Pane 0 is the left half of the window (Blue), pane 1 the right half (Red).
// Arbitrary but fixed for the lifetime of the app.
inline tactics::Team PaneTeam(int pane) {
  return pane == 0 ? tactics::Team::Blue : tactics::Team::Red;
}
inline const char* TeamName(tactics::Team team) {
  return team == tactics::Team::Blue ? "Blue" : "Red";
}

struct PaneRect {
  int x = 0;
  int width = 0;
};

inline PaneRect ComputePaneRect(int pane, int windowWidth) {
  const int leftWidth = windowWidth / 2;
  if (pane == 0) return PaneRect{0, leftWidth};
  return PaneRect{leftWidth, windowWidth - leftWidth};
}

inline int PaneForX(int x, int windowWidth) { return x < windowWidth / 2 ? 0 : 1; }

// Whichever team currently has the turn is the "active" pane -- only that
// side accepts game-action input. nullopt once the game is over.
std::optional<tactics::Team> ActiveTeam(const tactics::GameLogic& game);

// Projects a world point to top-left-origin window pixels, as seen through
// `camera` rendered into `rect` (full window height `windowHeight`).
glm::vec2 WorldToWindow(const glm::vec3& world, const gfx::OrbitCamera& camera,
                        const PaneRect& rect, int windowHeight);

struct HudActions {
  bool newMatch = false;
  bool move = false;
  bool shoot = false;
  bool overwatch = false;
  bool pass = false;
  bool cancel = false;
};

// Draws the Turn / Game Over panel, the floating action menu, the pane
// divider, team labels and the inactive-pane dimming overlay.
HudActions DrawHud(const tactics::GameLogic& game, int windowWidth, int windowHeight,
                   const std::array<gfx::OrbitCamera, kPaneCount>& cameras);

}  // namespace ui
