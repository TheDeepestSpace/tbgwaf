#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "AppFlow.h"

namespace tactics::ui {

// State of the Room screen's join-by-code field. `code` is the normalized
// (upper-case) code once JoinRoom fires; `error` is shown under the field
// (set by the menu for a malformed code, by the host for a refused join).
struct RoomForm {
  std::string code;
  std::string error;
};

// Draws the Splash / Game Mode / Map Select screen for `state` (call between
// ImGui::NewFrame() and Render()). Returns the flow event the user fired this
// frame, if any. Game Mode offers Regular / CTF (fires SelectRegular / SelectCtf).
// Game Mode also offers Online (fires SelectOnline) when `onlineAvailable`
// (a game server is configured). The Room screen has New room (CreateRoom: the
// server issues a code), a code box + Join (JoinRoom, `room->code` filled) and Back.
// Map Select shows an editable seed box bound to `*mapSeed`
// (digits only, plus a Random button). Gameplay draws no menu.
std::optional<tbgwaf_flow::Event> DrawMenu(tbgwaf_flow::State state, int windowWidth,
                                           int windowHeight,
                                           uint32_t* mapSeed, bool onlineAvailable = false,
                                           RoomForm* room = nullptr);

}  // namespace tactics::ui
