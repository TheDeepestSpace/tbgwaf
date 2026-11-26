#pragma once

#include <cstring>
#include <optional>

#include "AppFlow.h"

namespace tactics::ui {

// The flow-event buttons of the menu screens, by screen and label. DrawMenu
// draws exactly these labels, and the integration runner "clicks" them by the
// same table (no GL needed), so a renamed button cannot silently break the
// scenarios. Buttons that carry data (a game row on My games, the Join box,
// Builder cards) are not here: they have their own inputs.
struct MenuButton {
  tbgwaf_flow::State state;
  const char* label;
  tbgwaf_flow::Event event;
};

inline constexpr const char* kButtonNewGame = "New Game";
inline constexpr const char* kButtonRegular = "Regular";
inline constexpr const char* kButtonCtf = "Capture the Flag";
inline constexpr const char* kButtonBuilder = "Builder";
inline constexpr const char* kButtonOnline = "Online (2 players)";
inline constexpr const char* kButtonMyGames = "My games";
inline constexpr const char* kButtonContinue = "Continue";
inline constexpr const char* kButtonBack = "Back";
inline constexpr const char* kButtonNewRoom = "New room";
inline constexpr const char* kButtonPlayAi = "Play vs AI";
inline constexpr const char* kButtonJoin = "Join";

inline constexpr MenuButton kMenuButtons[] = {
    {tbgwaf_flow::State::Splash, kButtonNewGame, tbgwaf_flow::Event::NewGame},
    {tbgwaf_flow::State::GameMode, kButtonRegular, tbgwaf_flow::Event::SelectRegular},
    {tbgwaf_flow::State::GameMode, kButtonCtf, tbgwaf_flow::Event::SelectCtf},
    {tbgwaf_flow::State::GameMode, kButtonBuilder, tbgwaf_flow::Event::SelectBuilder},
    {tbgwaf_flow::State::GameMode, kButtonOnline, tbgwaf_flow::Event::SelectOnline},
    {tbgwaf_flow::State::GameMode, kButtonMyGames, tbgwaf_flow::Event::SelectMyGames},
    {tbgwaf_flow::State::GameMode, kButtonContinue, tbgwaf_flow::Event::ContinueGame},
    {tbgwaf_flow::State::GameMode, kButtonBack, tbgwaf_flow::Event::Back},
    {tbgwaf_flow::State::Room, kButtonNewRoom, tbgwaf_flow::Event::CreateRoom},
    {tbgwaf_flow::State::Room, kButtonPlayAi, tbgwaf_flow::Event::CreateAiRoom},
    {tbgwaf_flow::State::Room, kButtonJoin, tbgwaf_flow::Event::JoinRoom},
    {tbgwaf_flow::State::Room, kButtonBack, tbgwaf_flow::Event::Back},
    {tbgwaf_flow::State::MyGames, kButtonContinue, tbgwaf_flow::Event::ContinueGame},
    {tbgwaf_flow::State::MyGames, kButtonBack, tbgwaf_flow::Event::Back},
};

inline std::optional<tbgwaf_flow::Event> FindMenuButton(tbgwaf_flow::State state,
                                                        const char* label) {
  for (const MenuButton& b : kMenuButtons) {
    if (b.state == state && std::strcmp(b.label, label) == 0) return b.event;
  }
  return std::nullopt;
}

}  // namespace tactics::ui
