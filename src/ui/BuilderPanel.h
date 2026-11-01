#pragma once

// Builder editor UI (the builder_edit screen): the side panel that edits a
// BuilderScene (name, cubes, figures, Start/Back) plus the labels and
// selection outline drawn over its 3D viewport, and the small placement
// rules both share (snapping figures onto walkable surfaces, free spawn
// spots, what keeps a scene from starting). Call the Draw* functions
// between ImGui::NewFrame() and Render(). The 3D view itself, its camera
// and the figure drag are the app's (src/main.cpp).

#include <string>
#include <vector>

#include "game/BuilderScene.h"
#include "gfx/Camera.h"
#include "ui/Hud.h"

namespace tactics::ui {

// Width of the editor's side panel in a window `windowWidth` wide (narrower
// in a half-page web canvas); the 3D viewport gets the rest.
inline float BuilderPanelWidth(int windowWidth) {
  const float width = 0.45f * static_cast<float>(windowWidth);
  return width < 260.0f ? 260.0f : (width > 340.0f ? 340.0f : width);
}
// Figures stay this far inside the map edge.
constexpr float kBuilderMapMargin = 1.0f;
// Cube limits: only a tiny positive minimum so a cube never degenerates
// (very small cubes just get no walkable top); heights below 0.2 are barely
// visible. There is no maximum width, depth or height.
constexpr float kBuilderMinHalfExtent = 0.05f;
constexpr float kBuilderMinHeight = 0.2f;

// Selection in the editor (-1 = none). At most one of the two is set.
struct BuilderEditorState {
  int selectedBox = -1;
  int selectedFigure = -1;
};

struct BuilderEditorActions {
  bool start = false;  // Start pressed (only possible with no problems).
  bool back = false;   // Back pressed.
};

const char* BuilderWeaponName(WeaponType weapon);

// Why `bs` can't be played yet, one line each; empty when it can (both teams
// have a figure, every figure is inside the map, clear of the others and not
// clipping into a cube taller than the surface it stands on).
std::vector<std::string> BuilderSceneProblems(const BuilderScene& bs);

// Moves figure `index` to (x, z), snapped onto a walkable spot: clamped
// inside the map margin and, when the spot is on a cube top, kept inside the
// top's walkable inset so the figure stands on the cube rather than its edge.
void PlaceBuilderFigure(BuilderScene* bs, int index, float x, float z);

// Turns figure `index` to face `yaw` (radians from +X).
void RotateBuilderFigure(BuilderScene* bs, int index, float yaw);

// Replaces cube `index` with `next` (position, size, height; `next.y` is
// ignored) and settles it: it rests on the highest cube beneath its
// footprint (stacking) or on the ground. Cubes resting on it follow it
// sideways and up/down, so stacks move and grow as a unit.
void UpdateBuilderBox(BuilderScene* bs, int index, const BuilderBox& next);

// Appends a default cube at a free spot near the map center / a figure of
// `team` at a free spot on its side; returns the new index.
int AddBuilderBox(BuilderScene* bs);
int AddBuilderFigure(BuilderScene* bs, Team team);

// Draws the side panel (right edge of the window) editing `*bs` and
// `*state`'s selection.
BuilderEditorActions DrawBuilderEditor(BuilderScene* bs, BuilderEditorState* state,
                                       int windowWidth, int windowHeight);

// Viewport overlay: a label over every figure ("B1", "R2"; the selected one
// highlighted) and the selected cube's outline, projected through `camera`
// rendered into `rect`.
void DrawBuilderViewportOverlay(const BuilderScene& bs, const BuilderEditorState& state,
                                const gfx::OrbitCamera& camera, const ::ui::PaneRect& rect,
                                int windowHeight);

}  // namespace tactics::ui
