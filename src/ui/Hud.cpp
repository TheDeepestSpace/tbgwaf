#include "ui/Hud.h"

#include <cstdio>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Unit.h"

using tactics::GameLogic;
using tactics::InputMode;
using tactics::Team;
using tactics::Unit;

namespace ui {

std::optional<Team> ActiveTeam(const GameLogic& game) {
  if (game.Mode() == InputMode::GameOver) return std::nullopt;
  if (const auto actorId = game.CurrentActorId()) {
    if (const Unit* actor = game.FindUnit(*actorId)) return actor->team;
  }
  return std::nullopt;
}

glm::vec2 WorldToWindow(const glm::vec3& world, const gfx::OrbitCamera& camera,
                        const PaneRect& rect, int windowHeight) {
  const glm::mat4 view = camera.ViewMatrix();
  const glm::mat4 proj =
      camera.ProjectionMatrix(static_cast<float>(rect.width) / static_cast<float>(windowHeight));
  const glm::vec4 viewport(static_cast<float>(rect.x), 0.0f, static_cast<float>(rect.width),
                           static_cast<float>(windowHeight));
  const glm::vec3 screen = glm::project(world, view, proj, viewport);
  // glm::project assumes a bottom-left viewport origin; flip Y for ImGui's
  // top-left screen space. X is already absolute window space since the
  // viewport carries the pane's own offset.
  return glm::vec2(screen.x, windowHeight - screen.y);
}

HudActions DrawHud(const GameLogic& game, int windowWidth, int windowHeight,
                   const std::array<gfx::OrbitCamera, kPaneCount>& cameras) {
  HudActions actions;
  std::array<PaneRect, kPaneCount> paneRects;
  for (int pane = 0; pane < kPaneCount; ++pane) {
    paneRects[pane] = ComputePaneRect(pane, windowWidth);
  }
  const std::optional<Team> activeTeam = ActiveTeam(game);
  auto isPaneActive = [&](int pane) { return activeTeam && *activeTeam == PaneTeam(pane); };

  if (const auto winner = game.Winner()) {
    ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, windowHeight * 0.3f), ImGuiCond_Always,
                             ImVec2(0.5f, 0.5f));
    ImGui::Begin("Game Over", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("%s team wins!", TeamName(*winner));
    if (ImGui::Button("New Match")) actions.newMatch = true;
    ImGui::End();
  } else {
    ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
    ImGui::Begin("Turn", nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoMove);
    ImGui::Text("Round %d", game.RoundNumber());
    if (const auto actorId = game.CurrentActorId()) {
      const Unit* actor = game.FindUnit(*actorId);
      if (actor) {
        ImGui::Text("%s team's turn (figure #%d)", TeamName(actor->team), actor->id);
      }
    }
    switch (game.Mode()) {
      case InputMode::AwaitingSelection:
        ImGui::TextWrapped("Click the highlighted figure to act.");
        break;
      case InputMode::ActionMenu:
        ImGui::TextWrapped("Choose an action.");
        break;
      case InputMode::AwaitingMoveDestination:
        ImGui::TextWrapped("Click a destination on the ground (Esc to cancel).");
        break;
      case InputMode::AwaitingShootTarget:
        ImGui::TextWrapped("Click an enemy figure to shoot (Esc to cancel).");
        break;
      case InputMode::Moving:
        ImGui::TextWrapped("Figure is moving...");
        break;
      default:
        break;
    }
    ImGui::End();

    if (const auto selectedId = game.SelectedUnitId(); selectedId && activeTeam) {
      const Unit* selected = game.FindUnit(*selectedId);
      if (selected && (game.Mode() == InputMode::ActionMenu ||
                        game.Mode() == InputMode::AwaitingMoveDestination ||
                        game.Mode() == InputMode::AwaitingShootTarget)) {
        const int activePane = *activeTeam == Team::Blue ? 0 : 1;
        const glm::vec2 pos =
            WorldToWindow(selected->position + glm::vec3(0.0f, 1.9f, 0.0f), cameras[activePane],
                          paneRects[activePane], windowHeight);
        ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
        ImGui::Begin("Actions", nullptr,
                     ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                         ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);
        if (game.Mode() == InputMode::ActionMenu) {
          if (ImGui::Button("Move")) actions.move = true;
          ImGui::SameLine();
          if (ImGui::Button("Shoot")) actions.shoot = true;
          ImGui::SameLine();
          if (ImGui::Button("Overwatch")) actions.overwatch = true;
          ImGui::SameLine();
          if (ImGui::Button("Pass")) actions.pass = true;
        } else {
          if (ImGui::Button("Cancel")) actions.cancel = true;
        }
        ImGui::End();
      }
    }
  }

  // Pane divider, per-pane team labels, and a dimming overlay on whichever
  // pane isn't currently allowed to act -- the split-screen equivalent of
  // "grey out the inactive side" for local multiplayer.
  ImDrawList* overlay = ImGui::GetForegroundDrawList();
  overlay->AddLine(ImVec2(static_cast<float>(paneRects[1].x), 0.0f),
                    ImVec2(static_cast<float>(paneRects[1].x), static_cast<float>(windowHeight)),
                    IM_COL32(255, 255, 255, 60), 2.0f);
  for (int pane = 0; pane < kPaneCount; ++pane) {
    const PaneRect& rect = paneRects[pane];
    const bool active = isPaneActive(pane);
    const char* status = game.Winner() ? "" : (active ? " - your turn" : "");
    char label[64];
    std::snprintf(label, sizeof(label), "%s%s", TeamName(PaneTeam(pane)), status);
    overlay->AddText(ImVec2(rect.x + 10.0f, windowHeight - 24.0f), IM_COL32(255, 255, 255, 220),
                      label);
    if (game.Mode() != InputMode::GameOver && !active) {
      overlay->AddRectFilled(ImVec2(static_cast<float>(rect.x), 0.0f),
                              ImVec2(static_cast<float>(rect.x + rect.width),
                                     static_cast<float>(windowHeight)),
                              IM_COL32(0, 0, 0, 110));
    }
  }
  return actions;
}

}  // namespace ui
