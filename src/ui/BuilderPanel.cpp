#include "ui/BuilderPanel.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cfloat>
#include <cstdio>
#include <cstring>

#include "game/Types.h"

namespace tactics::ui {
namespace {

constexpr float kFigureClearance = 2.0f * constants::kAgentRadius + 0.1f;  // Center to center.
// Kept a little inside the navmesh's own climb-top inset (0.15).
constexpr float kTopInset = 0.2f;
constexpr float kLabelHeight = 2.1f;  // Above the feet.

constexpr WeaponType kWeapons[] = {WeaponType::DesertEagle, WeaponType::AssaultRifle,
                                   WeaponType::SniperRifle};

bool BoxContains(const BuilderBox& box, float x, float z) {
  return std::fabs(x - box.cx) <= box.hx && std::fabs(z - box.cz) <= box.hz;
}

// Highest box whose footprint contains (x, z), or -1.
int TopBoxAt(const BuilderScene& bs, float x, float z) {
  int best = -1;
  for (int i = 0; i < static_cast<int>(bs.boxes.size()); ++i) {
    const BuilderBox& box = bs.boxes[i];
    if (BoxContains(box, x, z) && (best < 0 || BoxTop(box) > BoxTop(bs.boxes[best]))) best = i;
  }
  return best;
}

// Does a figure's footprint at (x, z) reach into a box taller than `standY`?
bool ClipsCube(const BuilderScene& bs, float x, float z, float standY) {
  for (const BuilderBox& box : bs.boxes) {
    if (BoxTop(box) <= standY + 1e-3f) continue;
    const float dx = std::max(std::fabs(x - box.cx) - box.hx, 0.0f);
    const float dz = std::max(std::fabs(z - box.cz) - box.hz, 0.0f);
    if (dx * dx + dz * dz < constants::kAgentRadius * constants::kAgentRadius) return true;
  }
  return false;
}

float PlayableHalfExtent(const BuilderScene& bs) {
  return std::max(bs.mapHalfExtent - kBuilderMapMargin, 0.0f);
}

bool FigureSpotFree(const BuilderScene& bs, float x, float z, int ignoreIndex) {
  for (int i = 0; i < static_cast<int>(bs.figures.size()); ++i) {
    if (i == ignoreIndex) continue;
    if (std::hypot(bs.figures[i].x - x, bs.figures[i].z - z) < kFigureClearance + 0.3f) {
      return false;
    }
  }
  return !ClipsCube(bs, x, z, SurfaceHeightAt(bs, x, z));
}

bool FootprintsOverlap(const BuilderBox& a, const BuilderBox& b) {
  return std::fabs(a.cx - b.cx) < a.hx + b.hx - 1e-3f &&
         std::fabs(a.cz - b.cz) < a.hz + b.hz - 1e-3f;
}

std::string FigureLabel(const BuilderScene& bs, int index) {
  int ordinal = 0;
  for (int i = 0; i <= index; ++i) {
    if (bs.figures[i].team == bs.figures[index].team) ++ordinal;
  }
  char buf[32];
  std::snprintf(buf, sizeof buf, "%s %d", bs.figures[index].team == Team::Blue ? "Blue" : "Red",
                ordinal);
  return buf;
}

ImU32 TeamColor(Team team, int alpha = 255) {
  return team == Team::Blue ? IM_COL32(90, 150, 255, alpha) : IM_COL32(255, 95, 85, alpha);
}

}  // namespace

const char* BuilderWeaponName(WeaponType weapon) {
  switch (weapon) {
    case WeaponType::DesertEagle: return "Desert Eagle";
    case WeaponType::AssaultRifle: return "Assault Rifle";
    case WeaponType::SniperRifle: return "Sniper Rifle";
  }
  return "?";
}

std::vector<std::string> BuilderSceneProblems(const BuilderScene& bs) {
  std::vector<std::string> problems;
  bool hasBlue = false, hasRed = false;
  for (const BuilderFigure& figure : bs.figures) {
    (figure.team == Team::Blue ? hasBlue : hasRed) = true;
  }
  if (!hasBlue) problems.push_back("Add at least one Blue figure.");
  if (!hasRed) problems.push_back("Add at least one Red figure.");
  const float limit = PlayableHalfExtent(bs) + 1e-3f;
  for (int i = 0; i < static_cast<int>(bs.figures.size()); ++i) {
    const BuilderFigure& figure = bs.figures[i];
    const std::string label = FigureLabel(bs, i);
    if (std::fabs(figure.x) > limit || std::fabs(figure.z) > limit) {
      problems.push_back(label + " is outside the map.");
    }
    if (ClipsCube(bs, figure.x, figure.z, SurfaceHeightAt(bs, figure.x, figure.z))) {
      problems.push_back(label + " is clipping into a cube.");
    }
    for (int j = i + 1; j < static_cast<int>(bs.figures.size()); ++j) {
      if (std::hypot(bs.figures[j].x - figure.x, bs.figures[j].z - figure.z) < kFigureClearance) {
        problems.push_back(label + " and " + FigureLabel(bs, j) + " overlap.");
      }
    }
  }
  return problems;
}

void PlaceBuilderFigure(BuilderScene* bs, int index, float x, float z) {
  if (index < 0 || index >= static_cast<int>(bs->figures.size())) return;
  const float limit = PlayableHalfExtent(*bs);
  x = std::clamp(x, -limit, limit);
  z = std::clamp(z, -limit, limit);
  const int top = TopBoxAt(*bs, x, z);
  if (top >= 0) {
    const BuilderBox& box = bs->boxes[top];
    const float insetX = std::min(kTopInset, box.hx);
    const float insetZ = std::min(kTopInset, box.hz);
    x = std::clamp(x, box.cx - box.hx + insetX, box.cx + box.hx - insetX);
    z = std::clamp(z, box.cz - box.hz + insetZ, box.cz + box.hz - insetZ);
  } else {
    // On the ground right beside a cube: slide out until the footprint
    // clears its side (a few passes settle corners between cubes).
    constexpr float kClear = constants::kAgentRadius + 0.02f;
    for (int pass = 0; pass < 4; ++pass) {
      bool moved = false;
      for (const BuilderBox& box : bs->boxes) {
        const float px = std::clamp(x, box.cx - box.hx, box.cx + box.hx);
        const float pz = std::clamp(z, box.cz - box.hz, box.cz + box.hz);
        const float dx = x - px, dz = z - pz;
        const float distance = std::hypot(dx, dz);
        if (distance >= constants::kAgentRadius || distance < 1e-5f) continue;
        x = px + dx / distance * kClear;
        z = pz + dz / distance * kClear;
        moved = true;
      }
      if (!moved) break;
    }
    x = std::clamp(x, -limit, limit);
    z = std::clamp(z, -limit, limit);
  }
  bs->figures[index].x = x;
  bs->figures[index].z = z;
}

void RotateBuilderFigure(BuilderScene* bs, int index, float yaw) {
  if (index < 0 || index >= static_cast<int>(bs->figures.size()) || !std::isfinite(yaw)) return;
  bs->figures[index].yaw = std::remainder(yaw, 2.0f * 3.14159265358979323846f);
  bs->figures[index].hasYaw = true;
}

void UpdateBuilderBox(BuilderScene* bs, int index, const BuilderBox& next) {
  if (index < 0 || index >= static_cast<int>(bs->boxes.size())) return;
  const BuilderBox old = bs->boxes[index];

  // Cubes resting on this one, transitively (underside at the top of the
  // cube below, footprints overlapping).
  std::vector<int> carried;
  std::vector<bool> isCarried(bs->boxes.size(), false);
  isCarried[index] = true;
  for (size_t head = 0, n = 1; head < n; ++head) {
    const BuilderBox below = head == 0 ? old : bs->boxes[carried[head - 1]];
    for (int j = 0; j < static_cast<int>(bs->boxes.size()); ++j) {
      if (isCarried[j] || std::fabs(bs->boxes[j].y - BoxTop(below)) > 1e-3f ||
          !FootprintsOverlap(bs->boxes[j], below)) {
        continue;
      }
      isCarried[j] = true;
      carried.push_back(j);
      ++n;
    }
  }

  // Settle: rise from the ground over every non-carried cube it overlaps and
  // would otherwise sink into.
  BuilderBox settled = next;
  settled.y = 0.0f;
  for (int pass = 0; pass < static_cast<int>(bs->boxes.size()) + 1; ++pass) {
    float y = 0.0f;
    for (int j = 0; j < static_cast<int>(bs->boxes.size()); ++j) {
      if (isCarried[j] || !FootprintsOverlap(settled, bs->boxes[j]) ||
          bs->boxes[j].y >= settled.y + settled.height - 1e-3f) {
        continue;
      }
      y = std::max(y, BoxTop(bs->boxes[j]));
    }
    if (y <= settled.y) break;
    settled.y = y;
  }

  const float dx = settled.cx - old.cx, dz = settled.cz - old.cz;
  const float dy = BoxTop(settled) - BoxTop(old);
  bs->boxes[index] = settled;
  for (const int j : carried) {
    bs->boxes[j].cx += dx;
    bs->boxes[j].cz += dz;
    bs->boxes[j].y += dy;
  }
}

int AddBuilderBox(BuilderScene* bs) {
  BuilderBox box;
  constexpr float kStep = 4.0f;
  const float limit = PlayableHalfExtent(*bs);
  auto free = [&](float cx, float cz) {
    for (const BuilderBox& other : bs->boxes) {
      if (std::fabs(cx - other.cx) < box.hx + other.hx + 0.5f &&
          std::fabs(cz - other.cz) < box.hz + other.hz + 0.5f) {
        return false;
      }
    }
    for (const BuilderFigure& figure : bs->figures) {
      if (std::fabs(figure.x - cx) < box.hx + 1.0f && std::fabs(figure.z - cz) < box.hz + 1.0f) {
        return false;
      }
    }
    return true;
  };
  bool placed = false;
  for (int ring = 0; ring <= 4 && !placed; ++ring) {
    for (int i = -ring; i <= ring && !placed; ++i) {
      for (int j = -ring; j <= ring && !placed; ++j) {
        if (std::max(std::abs(i), std::abs(j)) != ring) continue;
        const float cx = i * kStep, cz = j * kStep;
        if (std::fabs(cx) + box.hx > limit || std::fabs(cz) + box.hz > limit) continue;
        if (free(cx, cz)) {
          box.cx = cx;
          box.cz = cz;
          placed = true;
        }
      }
    }
  }
  bs->boxes.push_back(box);
  return static_cast<int>(bs->boxes.size()) - 1;
}

int AddBuilderFigure(BuilderScene* bs, Team team) {
  const float limit = PlayableHalfExtent(*bs);
  const float side = team == Team::Blue ? -1.0f : 1.0f;
  float x = side * std::min(10.0f, limit), z = 0.0f;
  bool placed = false;
  // Columns step back toward the team's own edge, then toward the center;
  // within a column, spots alternate around z = 0.
  for (int column = 0; column < 12 && !placed; ++column) {
    const float offset = (column % 2 == 0 ? 1.0f : -1.0f) * 2.0f * ((column + 1) / 2);
    const float cx = std::clamp(side * std::min(10.0f, limit) + side * offset, -limit, limit);
    for (int k = 0; k < 2 * static_cast<int>(limit / 2.0f) + 1 && !placed; ++k) {
      const float cz = (k % 2 == 0 ? 1.0f : -1.0f) * 2.0f * ((k + 1) / 2);
      if (std::fabs(cz) > limit) continue;
      if (FigureSpotFree(*bs, cx, cz, -1)) {
        x = cx;
        z = cz;
        placed = true;
      }
    }
  }
  BuilderFigure figure;
  figure.team = team;
  figure.weapon = WeaponType::AssaultRifle;
  bs->figures.push_back(figure);
  const int index = static_cast<int>(bs->figures.size()) - 1;
  PlaceBuilderFigure(bs, index, x, z);
  return index;
}

BuilderEditorActions DrawBuilderEditor(BuilderScene* bs, BuilderEditorState* state,
                                       int windowWidth, int windowHeight) {
  BuilderEditorActions actions;
  if (state->selectedBox >= static_cast<int>(bs->boxes.size())) state->selectedBox = -1;
  if (state->selectedFigure >= static_cast<int>(bs->figures.size())) state->selectedFigure = -1;

  const float panelWidth = BuilderPanelWidth(windowWidth);
  ImGui::SetNextWindowPos(ImVec2(windowWidth - panelWidth, 0.0f), ImGuiCond_Always);
  ImGui::SetNextWindowSize(ImVec2(panelWidth, static_cast<float>(windowHeight)),
                           ImGuiCond_Always);
  ImGui::Begin("Builder Editor", nullptr,
               ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                   ImGuiWindowFlags_NoSavedSettings);

  ImGui::TextUnformatted("Name");
  char name[64];
  std::snprintf(name, sizeof name, "%s", bs->name.c_str());
  ImGui::SetNextItemWidth(-1.0f);
  if (ImGui::InputText("##name", name, sizeof name)) bs->name = name;
  ImGui::SetNextItemWidth(120.0f);
  if (ImGui::InputInt("Round limit (0 = none)", &bs->roundLimit)) {
    bs->roundLimit = std::clamp(bs->roundLimit, 0, 99);
  }

  // --- Cubes ---
  ImGui::SeparatorText("Cubes");
  if (ImGui::Button("Add Cube")) {
    state->selectedBox = AddBuilderBox(bs);
    state->selectedFigure = -1;
  }
  if (ImGui::BeginListBox("##cubes", ImVec2(-1.0f, 5.0f * ImGui::GetTextLineHeightWithSpacing()))) {
    for (int i = 0; i < static_cast<int>(bs->boxes.size()); ++i) {
      const BuilderBox& box = bs->boxes[i];
      char label[112];
      std::snprintf(label, sizeof label,
                    "Cube %d  (%.1f, %.1f)  %.1f x %.1f x %.1f  base %.1f##cube%d", i + 1,
                    box.cx, box.cz, 2.0f * box.hx, 2.0f * box.hz, box.height, box.y, i);
      if (ImGui::Selectable(label, state->selectedBox == i)) {
        state->selectedBox = i;
        state->selectedFigure = -1;
      }
    }
    ImGui::EndListBox();
  }
  if (state->selectedBox >= 0) {
    BuilderBox box = bs->boxes[state->selectedBox];
    const float limit = bs->mapHalfExtent;
    bool changed = false;
    ImGui::PushItemWidth(-110.0f);
    changed |= ImGui::DragFloat("Center X", &box.cx, 0.05f, -limit, limit, "%.2f",
                                ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Center Z", &box.cz, 0.05f, -limit, limit, "%.2f",
                                ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Half size X", &box.hx, 0.02f, kBuilderMinHalfExtent,
                                kBuilderMaxHalfExtent, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Half size Z", &box.hz, 0.02f, kBuilderMinHalfExtent,
                                kBuilderMaxHalfExtent, "%.2f", ImGuiSliderFlags_AlwaysClamp);
    changed |= ImGui::DragFloat("Height", &box.height, 0.02f, kBuilderMinHeight, FLT_MAX, "%.2f",
                                ImGuiSliderFlags_AlwaysClamp);
    if (changed) UpdateBuilderBox(bs, state->selectedBox, box);
    ImGui::Text("Base height %.2f (cubes stack on cubes beneath)", bs->boxes[state->selectedBox].y);
    ImGui::PopItemWidth();
    if (ImGui::Button("Delete Cube")) {
      bs->boxes.erase(bs->boxes.begin() + state->selectedBox);
      state->selectedBox = -1;
    }
  }

  // --- Figures ---
  ImGui::SeparatorText("Figures");
  if (ImGui::Button("Add Blue Figure")) {
    state->selectedFigure = AddBuilderFigure(bs, Team::Blue);
    state->selectedBox = -1;
  }
  ImGui::SameLine();
  if (ImGui::Button("Add Red Figure")) {
    state->selectedFigure = AddBuilderFigure(bs, Team::Red);
    state->selectedBox = -1;
  }
  if (ImGui::BeginListBox("##figures",
                          ImVec2(-1.0f, 6.0f * ImGui::GetTextLineHeightWithSpacing()))) {
    for (int i = 0; i < static_cast<int>(bs->figures.size()); ++i) {
      const BuilderFigure& figure = bs->figures[i];
      const std::string label = FigureLabel(*bs, i) + " - " + BuilderWeaponName(figure.weapon) +
                                "##figure" + std::to_string(i);
      ImGui::PushStyleColor(ImGuiCol_Text, TeamColor(figure.team));
      if (ImGui::Selectable(label.c_str(), state->selectedFigure == i)) {
        state->selectedFigure = i;
        state->selectedBox = -1;
      }
      ImGui::PopStyleColor();
    }
    ImGui::EndListBox();
  }
  if (state->selectedFigure >= 0) {
    BuilderFigure& figure = bs->figures[state->selectedFigure];
    ImGui::PushItemWidth(-110.0f);
    if (ImGui::BeginCombo("Weapon", BuilderWeaponName(figure.weapon))) {
      for (const WeaponType weapon : kWeapons) {
        if (ImGui::Selectable(BuilderWeaponName(weapon), figure.weapon == weapon)) {
          figure.weapon = weapon;
        }
      }
      ImGui::EndCombo();
    }
    float position[2] = {figure.x, figure.z};
    if (ImGui::DragFloat2("Position X/Z", position, 0.05f, -bs->mapHalfExtent, bs->mapHalfExtent,
                          "%.2f")) {
      PlaceBuilderFigure(bs, state->selectedFigure, position[0], position[1]);
    }
    float facing = figure.hasYaw ? figure.yaw : (figure.team == Team::Blue ? 0.0f : 3.14159265f);
    float facingDegrees = facing * 180.0f / 3.14159265f;
    if (ImGui::SliderFloat("Facing", &facingDegrees, -180.0f, 180.0f, "%.0f deg")) {
      RotateBuilderFigure(bs, state->selectedFigure, facingDegrees * 3.14159265f / 180.0f);
    }
    ImGui::PopItemWidth();
    if (figure.hasYaw && ImGui::Button("Face enemy side")) figure.hasYaw = false;
    ImGui::Text("Standing at height %.2f", SurfaceHeightAt(*bs, figure.x, figure.z));
    if (ImGui::Button("Delete Figure")) {
      bs->figures.erase(bs->figures.begin() + state->selectedFigure);
      state->selectedFigure = -1;
    }
  }

  // --- Start / Back ---
  ImGui::SeparatorText("Play");
  const std::vector<std::string> problems = BuilderSceneProblems(*bs);
  for (const std::string& problem : problems) {
    ImGui::PushStyleColor(ImGuiCol_Text, IM_COL32(255, 140, 120, 255));
    ImGui::TextWrapped("%s", problem.c_str());
    ImGui::PopStyleColor();
  }
  ImGui::BeginDisabled(!problems.empty());
  if (ImGui::Button("Start", ImVec2(120.0f, 36.0f))) actions.start = true;
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Back", ImVec2(100.0f, 36.0f))) actions.back = true;
  ImGui::Spacing();
  ImGui::TextWrapped(
      "Left-drag a figure or cube to move it (figures snap onto the ground or a cube top; cubes "
      "stack on cubes beneath them). Shift+left-drag a figure to turn it toward the cursor. "
      "Click a figure or cube to select it. Right-drag rotates the view, left-drag the "
      "background pans, wheel zooms.");
  ImGui::End();
  return actions;
}

void DrawBuilderViewportOverlay(const BuilderScene& bs, const BuilderEditorState& state,
                                const gfx::OrbitCamera& camera, const ::ui::PaneRect& rect,
                                int windowHeight) {
  ImDrawList* draw = ImGui::GetBackgroundDrawList();
  const glm::vec3 eye = camera.Position();
  const glm::vec3 forward = camera.target - eye;
  auto project = [&](const glm::vec3& world, ImVec2* out) {
    if (glm::dot(world - eye, forward) <= 0.0f) return false;  // Behind the camera.
    const glm::vec2 p = ::ui::WorldToWindow(world, camera, rect, windowHeight);
    *out = ImVec2(p.x, p.y);
    return true;
  };
  draw->PushClipRect(ImVec2(static_cast<float>(rect.x), 0.0f),
                     ImVec2(static_cast<float>(rect.x + rect.width), static_cast<float>(windowHeight)));
  if (state.selectedBox >= 0 && state.selectedBox < static_cast<int>(bs.boxes.size())) {
    const BuilderBox& box = bs.boxes[state.selectedBox];
    glm::vec3 corners[8];
    for (int i = 0; i < 8; ++i) {
      corners[i] = glm::vec3(box.cx + ((i & 1) ? box.hx : -box.hx), (i & 4) ? BoxTop(box) : box.y,
                             box.cz + ((i & 2) ? box.hz : -box.hz));
    }
    constexpr int kEdges[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3},
                                   {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
    for (const auto& edge : kEdges) {
      ImVec2 a, b;
      if (project(corners[edge[0]], &a) && project(corners[edge[1]], &b)) {
        draw->AddLine(a, b, IM_COL32(255, 220, 60, 255), 2.0f);
      }
    }
  }
  for (int i = 0; i < static_cast<int>(bs.figures.size()); ++i) {
    const BuilderFigure& figure = bs.figures[i];
    ImVec2 p;
    if (!project(glm::vec3(figure.x, SurfaceHeightAt(bs, figure.x, figure.z) + kLabelHeight,
                           figure.z),
                 &p)) {
      continue;
    }
    {
      // Facing arrow along the ground at the figure's feet.
      const float yaw = figure.hasYaw ? figure.yaw
                                      : (figure.team == Team::Blue ? 0.0f : 3.14159265f);
      const float y = SurfaceHeightAt(bs, figure.x, figure.z) + 0.05f;
      const glm::vec3 from(figure.x, y, figure.z);
      const glm::vec3 to = from + glm::vec3(std::cos(yaw), 0.0f, std::sin(yaw)) * 1.2f;
      ImVec2 a, b;
      if (project(from, &a) && project(to, &b)) {
        draw->AddLine(a, b, TeamColor(figure.team), 3.0f);
        draw->AddCircleFilled(b, 4.0f, TeamColor(figure.team));
      }
    }
    std::string label = FigureLabel(bs, i);
    label = std::string(1, label[0]) + label.substr(label.find(' ') + 1);  // "Blue 2" -> "B2".
    const ImVec2 size = ImGui::CalcTextSize(label.c_str());
    const ImVec2 min(p.x - size.x * 0.5f - 4.0f, p.y - size.y - 4.0f);
    const ImVec2 max(p.x + size.x * 0.5f + 4.0f, p.y);
    const bool selected = state.selectedFigure == i;
    draw->AddRectFilled(min, max, selected ? IM_COL32(255, 220, 60, 230) : TeamColor(figure.team, 200),
                        3.0f);
    draw->AddText(ImVec2(min.x + 4.0f, min.y + 2.0f),
                  selected ? IM_COL32(20, 20, 20, 255) : IM_COL32(255, 255, 255, 255),
                  label.c_str());
  }
  draw->PopClipRect();
}

}  // namespace tactics::ui
