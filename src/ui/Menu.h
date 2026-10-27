#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "AppFlow.h"

namespace tactics::ui {

// One card on the Demo Scenes screen (issue #240): the scene's rendered
// thumbnail (a GL texture the app rendered offscreen; 0 draws a placeholder),
// its name (the button) and what to do in it.
struct DemoSceneCard {
  std::string name;
  std::string description;
  unsigned int thumbnailTexture = 0;
  int thumbnailWidth = 0;
  int thumbnailHeight = 0;
};

// Draws the Splash / Game Mode / Map Select / Demo Scenes screen for `state`
// (call between ImGui::NewFrame() and Render()). Returns the flow event the
// user fired this frame, if any. Game Mode offers Regular / CTF / Demo
// Scenes (fires SelectRegular / SelectCtf / SelectDemo). Map Select shows an
// editable seed box bound to `*mapSeed` (digits only, plus a Random button).
// Demo Scenes lists `demoCards`; pressing a card's button fires SelectScene
// and stores the card's index in `*selectedDemoScene`. Gameplay draws no
// menu.
std::optional<tbgwaf_flow::Event> DrawMenu(tbgwaf_flow::State state, int windowWidth,
                                           int windowHeight, uint32_t* mapSeed,
                                           const std::vector<DemoSceneCard>& demoCards = {},
                                           int* selectedDemoScene = nullptr);

}  // namespace tactics::ui
