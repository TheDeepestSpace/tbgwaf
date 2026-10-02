#pragma once

// Splash and Map Select screens (issue #97). Drawn between ImGui::NewFrame()
// and ImGui::Render(); the screen graph itself comes from src/flow/game_flow.yaml
// via the generated flow/game_flow.h.

#include <optional>

#include "flow/game_flow.h"

namespace ui {

// Draws the menu for `screen` (Splash or MapSelect) centered in the display.
// Returns the flow event the player triggered this frame, if any.
std::optional<flow::Event> DrawMenu(flow::Screen screen);

}  // namespace ui
