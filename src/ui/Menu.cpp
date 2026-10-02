#include "ui/Menu.h"

#include <imgui.h>

namespace ui {

namespace {

constexpr ImGuiWindowFlags kMenuFlags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                        ImGuiWindowFlags_NoSavedSettings |
                                        ImGuiWindowFlags_AlwaysAutoResize;

void BeginCentered(const char* name) {
  const ImVec2 center = ImGui::GetIO().DisplaySize;
  ImGui::SetNextWindowPos(ImVec2(center.x * 0.5f, center.y * 0.5f), ImGuiCond_Always,
                          ImVec2(0.5f, 0.5f));
  ImGui::Begin(name, nullptr, kMenuFlags);
}

}  // namespace

std::optional<flow::Event> DrawMenu(flow::Screen screen) {
  std::optional<flow::Event> event;
  const ImVec2 button(220.0f, 40.0f);

  if (screen == flow::Screen::Splash) {
    BeginCentered("Splash");
    ImGui::SetWindowFontScale(2.0f);
    ImGui::TextUnformatted("Firestep");
    ImGui::SetWindowFontScale(1.0f);
    ImGui::Spacing();
    if (ImGui::Button("New Game", button)) event = flow::Event::NewGame;
    ImGui::End();
  } else if (screen == flow::Screen::MapSelect) {
    BeginCentered("Map Select");
    ImGui::TextUnformatted("Select a map");
    ImGui::Spacing();
    if (ImGui::Button("Urban", button)) event = flow::Event::SelectUrban;
    // Hills stays disabled until the hilly-terrain generator lands (#90/#93).
    ImGui::BeginDisabled();
    ImGui::Button("Hills (coming soon)", button);
    ImGui::EndDisabled();
    ImGui::Spacing();
    if (ImGui::Button("Back", button)) event = flow::Event::Back;
    ImGui::End();
  }
  return event;
}

}  // namespace ui
