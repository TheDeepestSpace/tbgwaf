#include "ui/Menu.h"

#include <imgui.h>

#include <cstdio>
#include <cctype>
#include <cstdlib>
#include <random>

namespace tactics::ui {

using tbgwaf_flow::Event;
using tbgwaf_flow::State;

std::optional<Event> DrawMenu(State state, int windowWidth, int windowHeight,
                                  uint32_t* mapSeed, bool onlineAvailable, RoomForm* room) {
  if (state != State::Splash && state != State::GameMode && state != State::MapSelect &&
      state != State::Room) return std::nullopt;

  const ImGuiWindowFlags flags = ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
                                 ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoSavedSettings |
                                 ImGuiWindowFlags_AlwaysAutoResize;
  ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, windowHeight * 0.5f), ImGuiCond_Always,
                          ImVec2(0.5f, 0.5f));
  const ImVec2 button(220.0f, 40.0f);
  std::optional<Event> fired;

  if (state == State::Splash) {
    ImGui::Begin("Firestep", nullptr, flags | ImGuiWindowFlags_NoTitleBar);
    ImGui::SetWindowFontScale(2.5f);
    ImGui::TextUnformatted("Firestep");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Spacing();
    if (ImGui::Button("New Game", button)) fired = Event::NewGame;
    ImGui::End();
  } else if (state == State::GameMode) {
    ImGui::Begin("Game Mode", nullptr, flags);
    if (ImGui::Button("Regular", button)) fired = Event::SelectRegular;
    if (ImGui::Button("Capture the Flag", button)) fired = Event::SelectCtf;
    if (onlineAvailable && ImGui::Button("Online (2 players)", button)) fired = Event::SelectOnline;
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  } else if (state == State::Room) {
    ImGui::Begin("Online match", nullptr, flags);
    if (ImGui::Button("New room", button)) fired = Event::CreateRoom;
    ImGui::Spacing();
    ImGui::TextUnformatted("Room code");
    static char codeText[40] = "";
    ImGui::SetNextItemWidth(button.x - 70.0f);
    const bool entered = ImGui::InputText("##roomcode", codeText, sizeof codeText,
                                          ImGuiInputTextFlags_CharsUppercase |
                                              ImGuiInputTextFlags_CharsNoBlank |
                                              ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if (ImGui::Button("Join") || entered) {
      // Same alphabet and length the server accepts: [A-Z0-9_-]{1,32}.
      std::string code(codeText);
      bool valid = !code.empty() && code.size() <= 32;
      for (char c : code) {
        valid = valid && (std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-');
      }
      if (valid && room) {
        room->code = code;
        room->error.clear();
        fired = Event::JoinRoom;
      } else if (room) {
        room->error = "Enter the room code from the other player.";
      }
    }
    if (room && !room->error.empty()) {
      ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.62f, 0.62f, 1.0f));
      ImGui::PushTextWrapPos(button.x + 20.0f);
      ImGui::TextUnformatted(room->error.c_str());
      ImGui::PopTextWrapPos();
      ImGui::PopStyleColor();
    }
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  } else {
    ImGui::Begin("Map Select", nullptr, flags);
    // Edit buffer persists across frames; refreshed when the seed changes
    // from outside the box (Random button, first show).
    static char seedText[16] = "";
    static uint32_t shownSeed = 0;
    static bool seedShown = false;
    if (!seedShown || shownSeed != *mapSeed) {
      std::snprintf(seedText, sizeof seedText, "%u", static_cast<unsigned>(*mapSeed));
      shownSeed = *mapSeed;
      seedShown = true;
    }
    ImGui::TextUnformatted("Seed");
    ImGui::SetNextItemWidth(button.x - 80.0f);
    if (ImGui::InputText("##seed", seedText, sizeof seedText,
                         ImGuiInputTextFlags_CharsDecimal)) {
      *mapSeed = static_cast<uint32_t>(std::strtoull(seedText, nullptr, 10));
      shownSeed = *mapSeed;
    }
    ImGui::SameLine();
    if (ImGui::Button("Random")) *mapSeed = std::random_device{}();
    ImGui::Spacing();
    if (ImGui::Button("Urban", button)) fired = Event::SelectUrban;
    if (ImGui::Button("Urban x4", button)) fired = Event::SelectUrbanX4;
    if (ImGui::Button("Hills", button)) fired = Event::SelectHills;
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  }
  return fired;
}

}  // namespace tactics::ui
