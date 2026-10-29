#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "AppFlow.h"

namespace tactics::ui {

// One card on the Builder screen: a player-built scene's rendered thumbnail
// (a GL texture the app rendered offscreen; 0 draws a placeholder) and its
// name.
struct BuilderCard {
  std::string name;
  unsigned int thumbnailTexture = 0;
};

// The Builder screen's inputs (cards, the last game's log) and what the
// user did on it this frame.
struct BuilderMenu {
  std::vector<BuilderCard> cards;
  bool lastLogAvailable = false;  // Shows "Download last log".
  std::string status;             // Shown under the cards (e.g. where a log went).

  // Outputs, reset by DrawMenu every frame.
  int openIndex = -1;         // Card opened (fires OpenBuilderScene).
  bool newScene = false;      // New Scene pressed (also fires OpenBuilderScene).
  int deleteIndex = -1;       // Card whose Delete was pressed.
  bool downloadLastLog = false;
};

// Draws the Splash / Game Mode / Map Select / Builder screen for `state`
// (call between ImGui::NewFrame() and Render()). Returns the flow event the
// user fired this frame, if any. Game Mode offers Regular / CTF / Builder
// (fires SelectRegular / SelectCtf / SelectBuilder). Map Select shows an
// editable seed box bound to `*mapSeed` (digits only, plus a Random button).
// Builder lists `builder->cards` plus a New Scene card: opening a card
// fires OpenBuilderScene and sets `openIndex`; New Scene fires it with
// `newScene` set (the caller appends the scene); Back fires ExitBuilder.
// Gameplay and the Builder editor draw no menu.
std::optional<tbgwaf_flow::Event> DrawMenu(tbgwaf_flow::State state, int windowWidth,
                                           int windowHeight, uint32_t* mapSeed,
                                           BuilderMenu* builder = nullptr);

}  // namespace tactics::ui
