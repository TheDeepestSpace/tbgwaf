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

// Local-only rendering toggles for the interactive app's debug panel. Not
// game state: never synced. Defaults render normally (visual runner never sets it).
struct RenderDebugOptions {
  bool disableFov = false;
  bool disableShadows = false;
  // Thermal camera prototype (issue #177): scene geometry is grayscale with
  // figures the hottest signal, the shadow pass is skipped, and gameplay
  // overlays swap to monochrome sensor-green symbology. `thermalBlackHot`
  // picks the polarity, like the toggle on a real FLIR: true renders hot
  // objects dark on a washed-out bright world (matching the PR #178 review's
  // aerial reference footage), false is classic white-hot.
  bool thermal = false;
  bool thermalBlackHot = true;
  // Issue #136: skip the camera-occlusion see-through pass (occluding
  // blocks/decks are then always drawn fully opaque).
  bool disableOcclusionFade = false;
  // Optional sink for per-frame draw counters; accumulates across panes, so
  // the caller zeroes it once per frame. Null (default) = not collected.
  RenderFrameStats* stats = nullptr;
};

// Per-pane overlay state (selection ring, movement frontier, move-path
// preview). Always build it with BuildPaneOverlays so the interactive app
// and the visual scenario harness show the same overlays.
struct PaneOverlays {
  std::optional<glm::vec3> selectionHighlight;     // Yellow ring under the selected figure.
  std::optional<glm::vec3> invalidHoverHighlight;  // Red ring on an unreachable hover point.
  const std::vector<glm::vec3>* movePreviewPath = nullptr;  // Yellow preview polyline.
  const tactics::ReachField* moveFrontier = nullptr;  // Reachable-area gradient + border.
  bool moveFrontierSubsequentLeg = false;             // Border drawn yellow instead of green.
  bool showShotCone = false;                          // Selected figure is actively choosing a target.
  // Free-aim (issue #129), set while this pane's selected figure is choosing
  // a target: aimShooter gets the acid-green 360-degree LOS surface
  // highlight (everything it could aim at directly). aimMarker is the
  // placed-but-unconfirmed "+" selector.
  const tactics::Unit* aimShooter = nullptr;
  std::optional<glm::vec3> aimMarker;
  // Where the preview cone points: the placed marker, else the hovered
  // point, so it never sticks to the facing left by an earlier round's shot.
  std::optional<glm::vec3> aimConeTarget;
  // Debug: walkable-cell boundaries of this navmesh (ground cells cyan,
  // climb-top cells orange), hugging the terrain. Not set by
  // BuildPaneOverlays; the app's debug toggle / the map golden harness
  // supply a mesh built over the whole scene.
  const tactics::NavMesh* navMeshDebug = nullptr;
};

// Overlays `pane` shows for the current game state. They belong to the team
// of the selected figure, so only that player's pane gets them (the other
// side must not see enemy plans). `hoveredGroundPoint` is the cursor's
// ground hit, if any (interactive only; used for the invalid-move ring).
PaneOverlays BuildPaneOverlays(const tactics::GameLogic& game, tactics::Team paneTeam,
                               const std::optional<glm::vec3>& hoveredGroundPoint = std::nullopt);

// How a pane draws its own team's FOV cones (issue #110).
//  - CpuAnalytic (default): the analytic ground overlay in DrawFovCone --
//    corner rays, box/deck ground shadows, deck-top spans. Crisp edges, but
//    it only covers ground and deck tops.
//  - ShadowMap (prototype): per unit, the scene's depth is rendered from the
//    eye into two perspective maps that together span the 150-degree cone;
//    the main pass then re-draws the static geometry with a projective mask
//    shader that flat-tints every fragment inside the cone whose eye
//    sightline passes the depth test. Covers walls, roofs, deck sides and
//    terrain as well as ground; edges are shadow-map-resolution limited.
enum class FovOverlayMode { CpuAnalytic, ShadowMap };

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

  void SetFovOverlayMode(FovOverlayMode mode) { fovOverlayMode_ = mode; }
  FovOverlayMode GetFovOverlayMode() const { return fovOverlayMode_; }
  // ShadowMap mode only: height above an upward-facing fragment (ground,
  // deck top, roof) at which its sightline is tested. 0 (default) tints
  // ground wherever the ground point itself is visible -- the same "could a
  // target crouched at ground level hide here" semantics the CPU overlay
  // encodes. Setting it to e.g. kEyeHeight instead tints ground wherever a
  // standing target's eye would be visible (pure target-LOS semantics).
  void SetFovProbeHeight(float height) { fovProbeHeight_ = height; }
  float GetFovProbeHeight() const { return fovProbeHeight_; }

  // `debug` is a local-only dev toggle (interactive app); the default leaves
  // output unchanged, which the visual runner relies on.
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
                  GLuint targetFramebuffer = 0, const RenderDebugOptions& debug = {});

 private:
  Shader unlitShader_;
  Shader litShader_;
  Shader colorShader_;
  Shader coneSurfaceShader_;
  Shader depthShader_;
  CubeMesh cubeMesh_;
  SphereMesh sphereMesh_;
  LineMesh pathLine_;
  ColorTriangleMesh fovConeMesh_;
  ColorTriangleMesh shotConeMesh_;
  ColorTriangleMesh aimOverlayMesh_;  // "+" aim selector / free-aim markers.
  // Reuse the terrain-clipped geometry until the unit or map changes.
  struct TerrainFovCache {
    int unitId = -1;
    glm::vec3 eye{0.0f};
    float facingYaw = 0.0f;
    std::vector<glm::vec3> points;
  };
  std::vector<TerrainFovCache> terrainFovCache_;
  std::vector<tactics::Obstacle> fovKeyObstacles_;
  std::vector<tactics::AABB> fovKeySidewalks_;
  float fovKeyMapHalfExtent_ = 0.0f;
  TriangleMesh highlightRing_;
  ColorTriangleMesh frontierFill_;
  LineMesh frontierBorder_;
  // Frontier geometry is rebuilt only when the field changes.
  const tactics::ReachField* frontierKeyField_ = nullptr;
  glm::vec2 frontierKeyOrigin_{0.0f};
  float frontierKeyBudget_ = -1.0f;
  // Terrain ground mesh, rebuilt only when the scene's heightfield changes.
  // Keyed on the field's contents, not its address: successive scenes can
  // reuse the same storage address (e.g. stack-allocated GameLogic
  // instances), which an address key would mistake for "unchanged".
  LitTriangleMesh terrainMesh_;
  // Reused scratch mesh for polygon prisms and road/deck patches. Geometry
  // is already in world space, so the same upload path works in shadow and
  // lit passes without approximating wedges by their AABB.
  LitTriangleMesh geometryMesh_;
  tactics::HeightField terrainKey_;
  GLuint shadowFbo_ = 0;
  GLuint shadowDepthTex_ = 0;
  glm::vec3 lightDir_{0.0f, -1.0f, 0.0f};
  glm::mat4 lightSpaceMatrix_{1.0f};

  // Issue #110 shadow-map FOV prototype. Two eye-space depth maps per unit
  // (left/right halves of the cone), re-rendered for every unit of every
  // pane, plus the projective mask shader and a static-geometry mesh
  // (obstacles, decks, sidewalks, roads) built once per scene so the extra
  // passes are a couple of draw calls rather than a re-tessellation.
  FovOverlayMode fovOverlayMode_ = FovOverlayMode::CpuAnalytic;
  float fovProbeHeight_ = 0.0f;
  Shader fovMaskShader_;
  GLuint fovFbo_[2] = {0, 0};
  GLuint fovDepthTex_[2] = {0, 0};
  LitTriangleMesh fovSceneMesh_;
  unsigned long long fovSceneKey_ = 0;
  // Renders the two depth maps for `unit` looking along `facingYaw`, then
  // re-draws the static scene into the pane with the mask shader, tinted
  // `color` within `halfFovDegrees` of that heading. Leaves the pane framebuffer,
  // viewport, scissor and the FOV-overlay blend/stencil state as it found
  // them.
  void DrawFovShadowMask(const tactics::GameLogic& game, const tactics::Unit& unit,
                         float facingYaw, float halfFovDegrees, const glm::vec4& color,
                         const glm::mat4& viewProj, bool drawTerrain, GLuint targetFramebuffer,
                         int x, int y, int width, int height);
};

}  // namespace gfx
