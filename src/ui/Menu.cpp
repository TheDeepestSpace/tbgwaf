#include "ui/Menu.h"

#include <imgui.h>

namespace tactics::ui {

using tbgwaf_flow::Event;
using tbgwaf_flow::State;

std::optional<Event> DrawMenu(State state, int windowWidth, int windowHeight) {
  if (state != State::Splash && state != State::MapSelect) return std::nullopt;

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
  } else {
    ImGui::Begin("Map Select", nullptr, flags);
    if (ImGui::Button("Urban", button)) fired = Event::SelectUrban;
    // Hills becomes selectable once the hilly-terrain generator (#90/#93) lands.
    ImGui::BeginDisabled();
    ImGui::Button("Hills (coming soon)", button);
    ImGui::EndDisabled();
    ImGui::Spacing();
    if (ImGui::Button("Back", ImVec2(100.0f, 0.0f))) fired = Event::Back;
    ImGui::End();
  }
  return fired;
}

}  // namespace tactics::ui
