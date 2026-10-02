#pragma once

#include <optional>

#include "AppFlow.h"

namespace tactics::ui {

// Draws the Splash / Map Select screen for `state` (call between
// ImGui::NewFrame() and Render()). Returns the flow event the user fired this
// frame, if any. Gameplay draws no menu.
std::optional<tbgwaf_flow::Event> DrawMenu(tbgwaf_flow::State state, int windowWidth,
                                           int windowHeight);

}  // namespace tactics::ui
