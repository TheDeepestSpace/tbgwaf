#pragma once

#include <optional>
#include <vector>

#include <GLES3/gl3.h>
#include <glm/glm.hpp>

#include "game/GameLogic.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/Mesh.h"
#include "gfx/Shader.h"

namespace gfx {

// Stage-B fog-of-war: a unit is drawable/pickable in a given team's view if
// it's on the viewing team (you always see your own figures) or, when fog
// is active, currently inside that team's combined FOV. Shared between the
// render pass here and main.cpp's click picking, which must agree exactly:
// a hidden enemy's collision box must not be clickable just because it
// happens to sit behind something that is drawn.
bool IsUnitVisibleForRender(const tactics::Unit& unit, tactics::Team viewingTeam, bool fogActive,
                            const tactics::TeamVisibility& visibility);

// Per-pane overlay state (selection ring, movement frontier, move-path
// preview). Always build it with BuildPaneOverlays so the interactive app
// and the visual scenario harness show the same overlays.
struct PaneOverlays {
  std::optional<glm::vec3> selectionHighlight;     // Yellow ring under the selected figure.
  std::optional<glm::vec3> invalidHoverHighlight;  // Red ring on an unreachable hover point.
  const std::vector<glm::vec3>* movePreviewPath = nullptr;  // Yellow preview polyline.
  const tactics::ReachField* moveFrontier = nullptr;  // Reachable-area gradient + border.
};

// Overlays `pane` shows for the current game state. They belong to the team
// of the selected figure, so only that player's pane gets them (the other
// side must not see enemy plans). `hoveredGroundPoint` is the cursor's
// ground hit, if any (interactive only; used for the invalid-move ring).
PaneOverlays BuildPaneOverlays(const tactics::GameLogic& game, tactics::Team paneTeam,
                               const std::optional<glm::vec3>& hoveredGroundPoint = std::nullopt);

// Owns the GL resources (shaders, meshes, the shadow map) for the per-team
// fog-of-war scene pass, and renders one team's view of the game into a
// caller-specified viewport rect. Extracted from main.cpp's frame loop so
// the interactive app and the headless visual scenario runner (issue #14)
// draw through the exact same code path.
class SceneRenderer {
 public:
  // Requires a current GL context. Returns false if a shader fails to
  // compile or the shadow framebuffer is incomplete.
  bool Init();
  void Destroy();

  // Renders `team`'s fog-of-war view into the rect [x, x+width) x
  // [y, y+height) of `targetFramebuffer` (0 = default framebuffer): shadow
  // pass with only that team's visible casters, then ground, obstacles,
  // visible units, that team's own FOV cones, the current-actor highlight,
  // and any caller-supplied overlays. Uses scissored clears, so other panes
  // already drawn to the same framebuffer are left intact; scissor is
  // disabled again before returning.
  void RenderPane(const tactics::GameLogic& game, tactics::Team team, bool fogActive,
                  const tactics::TeamVisibility& visibility, const OrbitCamera& camera, int x,
                  int y, int width, int height, const PaneOverlays& overlays = {},
                  GLuint targetFramebuffer = 0);

 private:
  Shader unlitShader_;
  Shader litShader_;
  Shader colorShader_;
  Shader depthShader_;
  CubeMesh cubeMesh_;
  LineMesh pathLine_;
  TriangleMesh fovConeMesh_;
  TriangleMesh highlightRing_;
  ColorTriangleMesh frontierFill_;
  LineMesh frontierBorder_;
  // Frontier geometry is rebuilt only when the field changes.
  const tactics::ReachField* frontierKeyField_ = nullptr;
  glm::vec2 frontierKeyOrigin_{0.0f};
  float frontierKeyBudget_ = -1.0f;
  GLuint shadowFbo_ = 0;
  GLuint shadowDepthTex_ = 0;
  glm::vec3 lightDir_{0.0f, -1.0f, 0.0f};
  glm::mat4 lightSpaceMatrix_{1.0f};
};

}  // namespace gfx
