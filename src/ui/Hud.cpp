#include "ui/Hud.h"

#include <algorithm>
#include <cmath>
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

namespace {

bool& PlaybookOpen(Team team) {
  static bool open[2] = {false, false};
  return open[team == Team::Blue ? 0 : 1];
}

const char* ReactionName(tactics::ReactionAction a) {
  static const char* names[] = {"Do Nothing", "Shoot", "Stop", "Continue", "Shoot + Stop",
                                "Shoot + Continue"};
  return names[static_cast<int>(a)];
}

// The squad-wide reaction table as its own centered view: one row per
// (moving|stationary) x (seen|unseen) situation, one checkbox column per
// reaction. Clicking a box selects that reaction for the row; boxes that make
// no sense for the row (e.g. Stop for a stationary figure) are disabled.
void DrawPlaybookView(const GameLogic& game, Team team, const PaneRect& rect, int windowHeight,
                      const std::string& windowId, HudActions& actions) {
  using tactics::ReactionAction;
  ImGui::SetNextWindowPos(ImVec2(rect.x + rect.width * 0.5f, windowHeight * 0.5f),
                          ImGuiCond_Always, ImVec2(0.5f, 0.5f));
  ImGui::Begin(windowId.c_str(), nullptr,
               ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse);
  ImGui::Text("%s squad playbook (applies to every %s figure)", TeamName(team), TeamName(team));
  ImGui::TextUnformatted("When a sighted enemy is in view, a figure will:");

  struct Row { const char* label; bool moving; bool canSeeMe; };
  const Row rows[] = {{"Moving, seen", true, true},
                      {"Moving, unseen", true, false},
                      {"Stationary, seen", false, true},
                      {"Stationary, unseen", false, false}};
  const ReactionAction columns[] = {ReactionAction::DoNothing,    ReactionAction::Shoot,
                                    ReactionAction::Stop,         ReactionAction::Continue,
                                    ReactionAction::ShootStop,    ReactionAction::ShootContinue};

  tactics::SquadPlaybook edited = game.Playbook(team);
  bool changed = false;
  if (ImGui::BeginTable("playbook", 7, ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit)) {
    ImGui::TableSetupColumn("");
    for (ReactionAction c : columns) ImGui::TableSetupColumn(ReactionName(c));
    ImGui::TableHeadersRow();
    for (const Row& row : rows) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(row.label);
      ReactionAction& slot = edited.At(row.moving, row.canSeeMe);
      for (int i = 0; i < 6; ++i) {
        const ReactionAction c = columns[i];
        // Moving figures only get stop/continue variants; stationary only
        // do-nothing / shoot.
        const bool stationaryOnly = c == ReactionAction::DoNothing || c == ReactionAction::Shoot;
        ImGui::TableSetColumnIndex(i + 1);
        ImGui::PushID(&row - rows);
        ImGui::PushID(i);
        ImGui::BeginDisabled(stationaryOnly == row.moving);
        bool checked = slot == c;
        if (ImGui::Checkbox("##cell", &checked) && checked) {
          slot = c;
          changed = true;
        }
        ImGui::EndDisabled();
        ImGui::PopID();
        ImGui::PopID();
      }
    }
    ImGui::EndTable();
  }
  if (changed) actions.playbook = edited;
  if (ImGui::Button("Close")) PlaybookOpen(team) = false;
  ImGui::End();
}

// Shot-level bar (issue #138): a vertical segmented bar next to the aiming
// figure while it picks its shot. Dragging (mouse or touch -- the backend
// feeds fingers through as mouse input) sets how many shots of the burst to
// fire, 1..maxShots; the whole bar is one ImGui item, so a tap anywhere on
// it also sets the level directly. Drawn only in shooting mode, so no other
// capture/golden ever shows it.
void DrawShotLevelBar(const GameLogic& game, const Unit& shooter,
                      const gfx::OrbitCamera& camera, const PaneRect& rect, int windowHeight,
                      const std::string& windowId, HudActions& actions) {
  const int maxShots = game.MaxShotsForSelected();
  const int level = game.PlannedShotCount();
  constexpr float kBarWidth = 18.0f;
  constexpr float kBarHeight = 120.0f;

  // Anchored beside the figure (torso height), clear of the action menu
  // floating above its head.
  const glm::vec2 anchor = WorldToWindow(shooter.position + glm::vec3(0.0f, 0.9f, 0.0f),
                                         camera, rect, windowHeight);
  ImGui::SetNextWindowPos(ImVec2(anchor.x - 52.0f, anchor.y - kBarHeight * 0.5f),
                          ImGuiCond_Always, ImVec2(0.5f, 0.0f));
  ImGui::SetNextWindowBgAlpha(0.0f);
  ImGui::Begin(windowId.c_str(), nullptr,
               ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar |
                   ImGuiWindowFlags_NoScrollbar);
  ImGui::InvisibleButton("##shot_level", ImVec2(kBarWidth, kBarHeight));
  const ImVec2 barMin = ImGui::GetItemRectMin();
  const ImVec2 barMax = ImGui::GetItemRectMax();
  if (ImGui::IsItemActive()) {
    // Top of the bar = full burst, bottom = the 1-shot minimum (never 0).
    const float fraction = 1.0f - (ImGui::GetMousePos().y - barMin.y) / kBarHeight;
    const int dragged = std::max(
        1, std::min(maxShots, static_cast<int>(std::ceil(fraction * static_cast<float>(maxShots)))));
    if (dragged != level) actions.shots = dragged;
  }

  // One segment per shot, capped so a 30-round magazine stays readable; the
  // filled count then scales proportionally and the label carries the exact
  // number.
  constexpr int kMaxVisibleSegments = 12;
  const int segments = std::min(maxShots, kMaxVisibleSegments);
  const int filled = std::max(
      1, static_cast<int>(std::lround(static_cast<float>(level) * segments /
                                      static_cast<float>(maxShots))));
  ImDrawList* drawList = ImGui::GetWindowDrawList();
  constexpr float kGap = 2.0f;
  const float segmentHeight = (kBarHeight - kGap * (segments - 1)) / static_cast<float>(segments);
  for (int i = 0; i < segments; ++i) {
    // Segment 0 sits at the bottom of the bar; fill grows upward.
    const float bottom = barMax.y - i * (segmentHeight + kGap);
    const ImVec2 segMin(barMin.x, bottom - segmentHeight);
    const ImVec2 segMax(barMax.x, bottom);
    const bool on = i < filled;
    drawList->AddRectFilled(segMin, segMax,
                            on ? IM_COL32(70, 220, 90, 230) : IM_COL32(40, 80, 50, 90));
    drawList->AddRect(segMin, segMax, IM_COL32(120, 255, 140, on ? 255 : 110));
  }
  ImGui::Text("%d/%d", level, maxShots);
  ImGui::End();
}

}  // namespace

HudActions DrawHud(const GameLogic& game, Team team, bool planning, const PaneRect& rect,
                   int windowHeight, const gfx::OrbitCamera& camera, HudLayout* layout,
                   float* roundPanelBottom) {
  HudActions actions;
  // Window ids are suffixed with the team so two panes can share one ImGui
  // context (the visual runner) without their windows colliding.
  const std::string suffix = std::string("##") + TeamName(team);
  auto id = [&](const char* title) { return std::string(title) + suffix; };
  const float left = static_cast<float>(rect.x);
  // All HUD buttons go through this wrapper so their on-screen positions can
  // be reported back via `layout`.
  auto Button = [&](const char* label) {
    const bool pressed = ImGui::Button(label);
    if (layout) {
      const ImVec2 mn = ImGui::GetItemRectMin();
      const ImVec2 mx = ImGui::GetItemRectMax();
      layout->buttons.emplace_back(label,
                                   glm::vec2((mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f));
    }
    return pressed;
  };

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
    if (Button("New Match")) actions.newMatch = true;
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
        ImGui::TextWrapped("Click ground within reach to plan this round's leg; each further click adds a leg for the next round. Enter or Done to finish, Esc to cancel.");
        break;
      case InputMode::AwaitingShootTarget:
        ImGui::TextWrapped(
            "Click an enemy figure to lock on, or any surface point in the green area to place "
            "a free-aim shot ('+'). Set the shot count with the bar, then Fire. Esc cancels.");
        break;
      case InputMode::Executing:
        ImGui::TextWrapped("Round executing: both teams' plans are playing out...");
        break;
      default:
        break;
    }
    ImGui::BeginDisabled(!game.CanCommitRound());
    if (Button("Commit Round")) actions.commit = true;
    ImGui::EndDisabled();
    // Squad-wide config, not tied to any figure: opens its own view.
    ImGui::SameLine();
    bool& playbookOpen = PlaybookOpen(team);
    if (Button(playbookOpen ? "Close Playbook" : "Playbook")) playbookOpen = !playbookOpen;
    // Window height tracks collapse, so anything anchored below follows.
    if (roundPanelBottom) *roundPanelBottom = ImGui::GetWindowPos().y + ImGui::GetWindowHeight();
    ImGui::End();

    if (playbookOpen) DrawPlaybookView(game, team, rect, windowHeight, id("Playbook"), actions);

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
          if (Button("Move")) actions.move = true;
          ImGui::SameLine();
          if (Button("Shoot")) actions.shoot = true;
          ImGui::SameLine();
          if (Button("Pass")) actions.pass = true;
        } else {
          if (game.Mode() == InputMode::AwaitingMoveDestination) {
            if (Button("Done")) actions.done = true;
            ImGui::SameLine();
          }
          // Free-aim: a placed "+" gets its confirm button (the touch flow's
          // tap-to-place -> confirm) -- no hover reliance.
          if (game.Mode() == InputMode::AwaitingShootTarget &&
              (game.GetAimPreview() || game.GetLockPreview())) {
            if (Button("Fire")) actions.fire = true;
            ImGui::SameLine();
          }
          if (Button("Cancel")) actions.cancel = true;
        }
        ImGui::End();

        // Shooting mode only: the burst-size bar beside the aiming figure.
        if (game.Mode() == InputMode::AwaitingShootTarget) {
          DrawShotLevelBar(game, *selected, camera, rect, windowHeight, id("Shot Level"),
                           actions);
        }
      }
    }
  }

  ImGui::GetForegroundDrawList()->AddText(ImVec2(left + 10.0f, windowHeight - 24.0f),
                                           IM_COL32(255, 255, 255, 220), TeamName(team));
  return actions;
}

TimelineActions DrawTimeline(const tactics::TurnTimeline& timeline,
                             const tactics::TimelinePlayback& playback, Team team,
                             const PaneRect& rect, int windowHeight, HudLayout* layout) {
  TimelineActions actions;
  const auto& ticks = timeline.Ticks();
  if (ticks.empty()) return actions;  // No history yet (first frame).
  const int tickCount = static_cast<int>(ticks.size());

  const std::string suffix = std::string("##") + TeamName(team);
  auto Button = [&](const char* label) {
    const bool pressed = ImGui::Button(label);
    if (layout) {
      const ImVec2 mn = ImGui::GetItemRectMin();
      const ImVec2 mx = ImGui::GetItemRectMax();
      layout->buttons.emplace_back(label,
                                   glm::vec2((mn.x + mx.x) * 0.5f, (mn.y + mx.y) * 0.5f));
    }
    return pressed;
  };

  // Taller frames and a wide slider grab so the strip works with a finger,
  // not just a mouse pointer.
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(14.0f, 10.0f));
  ImGui::PushStyleVar(ImGuiStyleVar_GrabMinSize, 22.0f);
  // Bottom-centered, above the pane's bottom edge; auto-resize keeps it
  // narrow enough to leave the corners (team label, Round panel) free.
  ImGui::SetNextWindowPos(ImVec2(rect.x + rect.width * 0.5f, windowHeight - 10.0f),
                          ImGuiCond_Always, ImVec2(0.5f, 1.0f));
  ImGui::Begin(("Timeline" + suffix).c_str(), nullptr,
               ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                   ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);

  const bool live = !playback.Active();
  // Play only makes sense from a replay position (live has nothing ahead).
  ImGui::BeginDisabled(live || timeline.FrameCount() <= 1);
  if (Button(playback.Playing() ? "Pause" : "Play")) actions.togglePlay = true;
  ImGui::EndDisabled();

  // Track: a line with a circle per turn boundary and a draggable handle.
  // Turns get equal widths (see below); the handle sits at the
  // right end while live.
  const float trackW = std::max(140.0f, std::min(static_cast<float>(rect.width) - 280.0f, 420.0f));
  const float trackH = 52.0f;
  const float pad = 14.0f;  // Room for end circles and the handle.
  ImGui::SameLine();
  const ImVec2 origin = ImGui::GetCursorScreenPos();
  ImGui::InvisibleButton(("##track" + suffix).c_str(), ImVec2(trackW, trackH));
  const bool dragging = ImGui::IsItemActive();
  const float x0 = origin.x + pad;
  const float x1 = origin.x + trackW - pad;
  const float lineY = origin.y + 18.0f;
  // Every turn gets an equal share of the track (rounds are the same length
  // in game terms, but a client may record them with different wall-clock
  // durations); frames sit proportionally to recorded time within their turn.
  // Frames after the last tick (a round in progress) form one trailing turn.
  const size_t frameCount = timeline.FrameCount();
  const bool hasTail = ticks.back().frame + 1 < frameCount;
  const int segments = std::max(1, (tickCount - 1) + (hasTail ? 1 : 0));
  auto SegStart = [&](int s) { return timeline.FrameTime(ticks[s].frame); };
  auto SegEnd = [&](int s) {
    return s + 1 < tickCount ? timeline.FrameTime(ticks[s + 1].frame) : timeline.EndTime();
  };
  auto Norm = [&](size_t frame) {  // Frame index -> [0, 1] along the track.
    const int s = std::min(timeline.TickForFrame(frame), segments - 1);
    const float len = SegEnd(s) - SegStart(s);
    const float f = len > 0.0f ? (timeline.FrameTime(frame) - SegStart(s)) / len : 0.0f;
    return (static_cast<float>(s) + std::max(0.0f, std::min(f, 1.0f))) / segments;
  };
  auto FrameAtNorm = [&](float u) {
    const float pos = std::max(0.0f, std::min(u, 1.0f)) * segments;
    const int s = std::min(static_cast<int>(pos), segments - 1);
    return timeline.FrameAtTime(SegStart(s) + (pos - s) * (SegEnd(s) - SegStart(s)));
  };
  auto TickX = [&](int i) { return x0 + (x1 - x0) * std::min(float(i) / segments, 1.0f); };

  float handleX = live ? x1 : x0 + (x1 - x0) * Norm(playback.FrameIndex());
  if (dragging && frameCount > 1) {
    constexpr float kMagnetPx = 14.0f;
    float mx = std::max(x0, std::min(ImGui::GetIO().MousePos.x, x1));
    float best = kMagnetPx;
    float snapped = mx;
    for (int i = 0; i < tickCount; ++i) {
      const float d = std::fabs(TickX(i) - mx);
      if (d < best) {
        best = d;
        snapped = TickX(i);
      }
    }
    handleX = snapped;
    if (handleX >= x1 - 0.5f) {
      if (!live) actions.live = true;
      handleX = x1;
    } else {
      const size_t frame = FrameAtNorm((handleX - x0) / (x1 - x0));
      if (live || frame != playback.FrameIndex()) actions.seekFrame = frame;
    }
  }

  ImDrawList* dl = ImGui::GetWindowDrawList();
  const ImU32 lineCol = IM_COL32(200, 200, 210, 200);
  const ImU32 playedCol = IM_COL32(110, 190, 255, 255);
  dl->AddLine(ImVec2(x0, lineY), ImVec2(x1, lineY), lineCol, 3.0f);
  dl->AddLine(ImVec2(x0, lineY), ImVec2(handleX, lineY), playedCol, 3.0f);
  for (int i = 0; i < tickCount; ++i) {
    const float tx = TickX(i);
    dl->AddCircleFilled(ImVec2(tx, lineY), 6.0f,
                        tx <= handleX + 0.5f ? playedCol : IM_COL32(60, 60, 70, 255));
    dl->AddCircle(ImVec2(tx, lineY), 6.0f, IM_COL32(235, 235, 245, 255), 0, 2.0f);
    const char* label = ticks[i].label.c_str();
    const ImVec2 ts = ImGui::CalcTextSize(label);
    dl->AddText(ImVec2(tx - ts.x * 0.5f, lineY + 12.0f), IM_COL32(230, 230, 240, 230), label);
  }
  dl->AddCircleFilled(ImVec2(handleX, lineY), 10.0f,
                      dragging ? IM_COL32(255, 255, 255, 255) : IM_COL32(245, 245, 250, 255));
  dl->AddCircle(ImVec2(handleX, lineY), 10.0f, playedCol, 0, 3.0f);

  // Live toggle: pushed in while the live game is showing; pressing it while
  // replaying returns to live (and it stays pushed in -- it is a lock).
  ImGui::SameLine();
  if (live) {
    ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
  }
  if (Button("Live") && !live) actions.live = true;
  if (live) ImGui::PopStyleColor(2);

  ImGui::End();
  ImGui::PopStyleVar(2);
  return actions;
}

void DrawDebugPanel(bool& disableFov, bool& disableShadows, bool& disableOcclusionFade,
                    bool& showFps, float fps, float frameMs, float fovMs,
                    const gfx::RenderFrameStats& stats, float top) {
  ImGui::SetNextWindowPos(ImVec2(10.0f, top), ImGuiCond_Always);
  ImGui::Begin("Debug", nullptr,
               ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove);
  ImGui::Checkbox("Disable FOV cones", &disableFov);
  ImGui::Checkbox("Disable shadows", &disableShadows);
  ImGui::Checkbox("Disable occlusion fade", &disableOcclusionFade);
  ImGui::Checkbox("Show FPS", &showFps);
  if (showFps) ImGui::Text("%.1f FPS (%.2f ms)", fps, frameMs);
  ImGui::Text("Draw calls: %d", stats.drawCalls);
  ImGui::Text("Triangles: %lld", stats.triangles);
  ImGui::Text("Vertices: %lld", stats.vertices);
  ImGui::Text("FOV cones: %.2f ms", fovMs);
  ImGui::End();
}

}  // namespace ui
