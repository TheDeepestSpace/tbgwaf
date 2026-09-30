#include "ui/Hud.h"

#include <string>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Unit.h"

using tactics::GameLogic;
using tactics::InputMode;
using tactics::Team;
using tactics::Unit;

namespace ui {

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

HudActions DrawHud(const GameLogic& game, Team team, bool planning, const PaneRect& rect,
                   int windowHeight, const gfx::OrbitCamera& camera) {
  HudActions actions;
  // Window ids are suffixed with the team so two panes can share one ImGui
  // context (the visual runner) without their windows colliding.
  const std::string suffix = std::string("##") + TeamName(team);
  auto id = [&](const char* title) { return std::string(title) + suffix; };
  const float left = static_cast<float>(rect.x);

  if (game.Mode() == InputMode::GameOver) {
    ImGui::SetNextWindowPos(ImVec2(left + rect.width * 0.5f, windowHeight * 0.3f),
                             ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::Begin(id("Game Over").c_str(), nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
    if (const auto winner = game.Winner()) {
      ImGui::Text("%s team wins!", TeamName(*winner));
    } else {
      // Simultaneous execution can down the last figure on both sides in
      // the same instant.
      ImGui::Text("Mutual annihilation -- draw!");
    }
    if (ImGui::Button("New Match")) actions.newMatch = true;
    ImGui::End();
  } else {
    int plannedCount[2] = {0, 0}, totalCount[2] = {0, 0};
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive) continue;
      const int idx = unit.team == Team::Blue ? 0 : 1;
      ++totalCount[idx];
      if (unit.plan.type != tactics::PlannedActionType::None) ++plannedCount[idx];
    }

    ImGui::SetNextWindowPos(ImVec2(left + 10, 10), ImGuiCond_Always);
    ImGui::Begin(id("Round").c_str(), nullptr,
                 ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoMove);
    ImGui::Text("Round %d", game.RoundNumber());
    if (planning) {
      ImGui::Text("Both teams plan every figure, then commit the round.");
      for (int t = 0; t < 2; ++t) {
        ImGui::Text("%s planned: %d / %d", TeamName(static_cast<Team>(t)), plannedCount[t],
                    totalCount[t]);
      }
    }
    switch (game.Mode()) {
      case InputMode::AwaitingSelection:
        ImGui::TextWrapped("Click one of your figures (in your own pane) to plan its action.");
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
      case InputMode::Executing:
        ImGui::TextWrapped("Round executing: both teams' plans are playing out...");
        break;
      default:
        break;
    }
    ImGui::BeginDisabled(!game.CanCommitRound());
    if (ImGui::Button("Commit Round")) actions.commit = true;
    ImGui::EndDisabled();
    ImGui::End();

    // The shared selection belongs to one team's figure; only that team's
    // pane shows its action menu.
    if (const auto selectedId = game.SelectedUnitId()) {
      const Unit* selected = game.FindUnit(*selectedId);
      if (selected && selected->team == team &&
          (game.Mode() == InputMode::ActionMenu ||
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

  ImGui::GetForegroundDrawList()->AddText(ImVec2(left + 10.0f, windowHeight - 24.0f),
                                           IM_COL32(255, 255, 255, 220), TeamName(team));
  return actions;
}

}  // namespace ui
