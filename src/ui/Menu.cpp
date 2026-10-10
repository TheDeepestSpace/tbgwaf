#include "ui/Menu.h"

#include <imgui.h>

#include <cstdio>
#include <cstdlib>
#include <random>

namespace tactics::ui {

using tbgwaf_flow::Event;
using tbgwaf_flow::State;

std::optional<Event> DrawMenu(State state, int windowWidth, int windowHeight,
                                  uint32_t* mapSeed) {
  if (state != State::Splash && state != State::GameMode &&
      state != State::MapSelect) return std::nullopt;

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
    if (ImGui::Button("Hills", button)) fired = Event::SelectHills;
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  }
  return fired;
}

}  // namespace tactics::ui
