#include "ui/Hud.h"

#include <cstdio>
#include <string>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Unit.h"

using tactics::GameLogic;
using tactics::InputMode;
using tactics::Team;
using tactics::Unit;

namespace ui {

std::optional<Team> ActiveTeam(const GameLogic& game) {
  if (game.Mode() == InputMode::GameOver) return std::nullopt;
  return game.CurrentTeam();
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

HudActions DrawHud(const GameLogic& game, Team team, bool isActive, const PaneRect& rect,
                   int windowHeight, const gfx::OrbitCamera& camera) {
  HudActions actions;
  const std::optional<Team> activeTeam = ActiveTeam(game);
  // Window ids are suffixed with the team so two panes can share one ImGui
  // context (the visual runner) without their windows colliding.
  const std::string suffix = std::string("##") + TeamName(team);
  auto id = [&](const char* title) { return std::string(title) + suffix; };
  const float left = static_cast<float>(rect.x);

  if (const auto winner = game.Winner()) {
    ImGui::SetNextWindowPos(ImVec2(left + rect.width * 0.5f, windowHeight * 0.3f),
                             ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin(id("Game Over").c_str(), nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::Text("%s team wins!", TeamName(*winner));
    if (ImGui::Button("New Match")) actions.newMatch = true;
    ImGui::End();
  } else {
    int plannedCount = 0, totalCount = 0;
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || !activeTeam || unit.team != *activeTeam) continue;
      ++totalCount;
      if (unit.plan.type != tactics::PlannedActionType::None) ++plannedCount;
    }

    ImGui::SetNextWindowPos(ImVec2(left + 10, 10), ImGuiCond_Always);
    ImGui::Begin(id("Turn").c_str(), nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoMove);
    ImGui::Text("Round %d", game.RoundNumber());
    if (activeTeam) {
      ImGui::Text("%s team's turn -- plan every figure, then commit.", TeamName(*activeTeam));
      ImGui::Text("Planned: %d / %d", plannedCount, totalCount);
    }
    switch (game.Mode()) {
      case InputMode::AwaitingSelection:
        ImGui::TextWrapped("Click one of your figures to plan its action.");
        break;
      case InputMode::ActionMenu:
        ImGui::TextWrapped("Choose an action to plan.");
        break;
      case InputMode::AwaitingMoveDestination:
        ImGui::TextWrapped("Click a destination on the ground (Esc to cancel).");
        break;
      case InputMode::AwaitingShootTarget:
        ImGui::TextWrapped("Click an enemy figure to plan a shot (Esc to cancel).");
        break;
      case InputMode::Moving:
        ImGui::TextWrapped("Committing turn: figures are moving...");
        break;
      default:
        break;
    }
    ImGui::BeginDisabled(!game.CanCommitTurn());
    if (ImGui::Button("Commit Turn")) actions.commit = true;
    ImGui::EndDisabled();
    ImGui::End();

    if (const auto selectedId = game.SelectedUnitId(); selectedId && isActive) {
      const Unit* selected = game.FindUnit(*selectedId);
      if (selected && (game.Mode() == InputMode::ActionMenu ||
                        game.Mode() == InputMode::AwaitingMoveDestination ||
                        game.Mode() == InputMode::AwaitingShootTarget)) {
        const glm::vec2 pos = WorldToWindow(selected->position + glm::vec3(0.0f, 1.9f, 0.0f),
                                            camera, rect, windowHeight);
        ImGui::SetNextWindowPos(ImVec2(pos.x, pos.y), ImGuiCond_Always, ImVec2(0.5f, 1.0f));
        ImGui::Begin(id("Actions").c_str(), nullptr,
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

  // Team label for this pane.
  const char* status = game.Winner() ? "" : (isActive ? " - your turn" : "");
  char label[64];
  std::snprintf(label, sizeof(label), "%s%s", TeamName(team), status);
  ImGui::GetForegroundDrawList()->AddText(ImVec2(left + 10.0f, windowHeight - 24.0f),
                                           IM_COL32(255, 255, 255, 220), label);
  return actions;
}

}  // namespace ui
