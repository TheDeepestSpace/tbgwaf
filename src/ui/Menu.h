#pragma once

#include <cstdint>
#include <optional>

#include "AppFlow.h"

namespace tactics::ui {

// Draws the Splash / Map Select screen for `state` (call between
// ImGui::NewFrame() and Render()). Returns the flow event the user fired this
// frame, if any. Map Select shows an editable seed box bound to `*mapSeed`
// (digits only, plus a Random button). Gameplay draws no menu.
std::optional<tbgwaf_flow::Event> DrawMenu(tbgwaf_flow::State state, int windowWidth,
                                           int windowHeight,
                                           uint32_t* mapSeed);

}  // namespace tactics::ui
