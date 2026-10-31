#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <GLES3/gl3.h>
#include <glm/glm.hpp>

#include "game/GameLogic.h"
#include "game/Scene.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/AmbientOcclusion.h"
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
// A downed enemy drawn as a ragdoll is judged by where its falling body
// actually is (a few joints against the viewing team's FOV), not by the spot
// it was shot at, so a figure that drops out of sight behind cover vanishes.
bool IsUnitVisibleForRender(const tactics::Unit& unit, tactics::Team viewingTeam, bool fogActive,
                            const tactics::TeamVisibility& visibility,
                            const tactics::Scene& scene);

// Local-only rendering toggles for the interactive app's debug panel. Not
// game state: never synced. Defaults render normally (visual runner never sets it).
struct RenderDebugOptions {
  bool disableFov = false;
  bool disableShadows = false;
  // Thermal camera prototype (issue #177): material-based grayscale with
  // pale surfaces and ambient contact shading; figures keep their team
  // color. Sensor blur and grain precede the overlays; sunlight shadows are
  // skipped. Gameplay overlays keep their in-game colors (only the HUD goes
  // green). `thermalBlackHot` picks the polarity, like the toggle on a real
  // FLIR: true renders a washed-out bright world (matching the PR #178
  // review's aerial reference footage), false is classic white-hot.
  bool thermal = false;
  bool thermalBlackHot = true;
  // Alpha of the team FOV tint in thermal view (normal view uses 0.15);
  // a debug-panel slider so it can be calibrated against the pale world.
  float thermalFovAlpha = 0.25f;
  // Contact-shading tunables for the thermal view's debug-panel sliders.
  AmbientOcclusionSettings thermalAo;
  // Issue #136: skip the camera-occlusion see-through pass (occluding
  // blocks/decks are then always drawn fully opaque).
  bool disableOcclusionFade = false;
  // Drop all surface colour (lit geometry is drawn in greyscale); used by
  // the gallery's map-seed viewer.
  bool monochrome = false;
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
  bool moveFrontierSubsequentLeg = false;             // Border/path drawn at 50% alpha.
  // Zipline reach (walk -> ride -> walk), drawn as a second, cyan region with
  // the line itself highlighted; the preview/planned path marks its ride.
  const std::vector<tactics::GameLogic::ZiplineFrontier>* ziplineFrontiers = nullptr;
  const std::vector<tactics::PathRide>* movePreviewRides = nullptr;
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
  // Jev-vs-Jev spectator view: draw both teams' FOV cones (each team's
  // overlap flattened separately, so Blue/Red overlap still reads). Callers
  // also pass fogActive=false so every figure is drawn.
  bool spectatorFov = false;
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
  Shader thermalSensorShader_;
  AmbientOcclusion thermalOcclusion_;
  GLuint thermalSceneTex_ = 0;
  GLuint thermalSensorVao_ = 0;
  int thermalSceneWidth_ = 0;
  int thermalSceneHeight_ = 0;
  void ApplyThermalSensor(int x, int y, int width, int height);
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
  // Issue #256: cached result of the camera-occlusion see-through raycasts,
  // one slot per pane team. The fade flags are a pure function of (camera
  // position, probe points, occluder geometry); while planning, frames
  // usually repeat all three exactly, so the O(rays x occluders) pass is
  // skipped. Geometry is content-hashed, not address-keyed, for the same
  // reason as the terrain mesh key below.
  struct FadeCache {
    bool valid = false;
    glm::vec3 cameraPos{0.0f};
    uint64_t geometryKey = 0;
    std::vector<glm::vec3> probes;
    std::vector<char> fadeObstacle, fadeSurface, fadeMassing, fadeBuilding;
    bool anyFaded = false;
  };
  FadeCache fadeCache_[2];
  TriangleMesh highlightRing_;
  ColorTriangleMesh frontierFill_;
  LineMesh frontierBorder_;
  // Frontier geometry is rebuilt only when the field changes.
  const tactics::ReachField* frontierKeyField_ = nullptr;
  glm::vec2 frontierKeyOrigin_{0.0f};
  float frontierKeyBudget_ = -1.0f;
  int frontierKeyThermal_ = 0;  // 0 off, 1 white-hot, 2 black-hot.
  ColorTriangleMesh ziplineFill_;
  LineMesh ziplineBorder_;
  std::vector<glm::vec4> ziplineKey_;  // (minX, minZ, budget, zipline) per region.
  // Terrain ground mesh, rebuilt only when the scene's heightfield changes.
  // Keyed on the field's contents, not its address: successive scenes can
  // reuse the same storage address (e.g. stack-allocated GameLogic
  // instances), which an address key would mistake for "unchanged".
  LitTriangleMesh terrainMesh_;
  LitTriangleMesh terrainRaisedMesh_;  // Concrete level tops and cliff faces of a city.
  LitTriangleMesh terrainRoadMesh_, terrainSidewalkMesh_, terrainStairMesh_;
  unsigned long long pavementKey_ = 0;
  // Reused scratch mesh for polygon prisms and road/deck patches. Geometry
  // is already in world space, so the same upload path works in shadow and
  // lit passes without approximating wedges by their AABB.
  LitTriangleMesh geometryMesh_;
  // Static street fixtures are assembled once per scene instead of
  // submitting every small cube separately each frame. The lit prop meshes
  // are grouped by material because the shader color is a uniform. Keep this
  // count in sync with StreetPropMaterial in the .cpp.
  static constexpr size_t kStreetPropMaterialCount = 11;
  LitTriangleMesh streetFurnitureMesh_;
  LitTriangleMesh streetPropDepthMesh_;
  std::array<LitTriangleMesh, kStreetPropMaterialCount> streetPropMeshes_;
  std::vector<tactics::Obstacle> streetFurnitureKey_;
  std::vector<tactics::StreetProp> streetPropKey_;
  tactics::HeightField terrainKey_;
  GLuint shadowFbo_ = 0;
  GLuint shadowDepthTex_ = 0;
  glm::vec3 lightDir_{0.0f, -1.0f, 0.0f};
  glm::mat4 lightSpaceMatrix_{1.0f};

  // Issue #110 shadow-map FOV prototype. Two eye-space depth maps per unit
  // (left/right halves of the cone), plus the projective mask shader and a
  // static-geometry mesh (obstacles, decks, sidewalks, roads) built once per
  // scene so the extra passes are a couple of draw calls rather than a
  // re-tessellation.
  FovOverlayMode fovOverlayMode_ = FovOverlayMode::CpuAnalytic;
  float fovProbeHeight_ = 0.0f;
  Shader fovMaskShader_;
  LitTriangleMesh fovSceneMesh_;
  unsigned long long fovSceneKey_ = 0;
  // Each figure's depth maps (and each free-aim sweep sector's) are kept
  // across frames and re-rendered only when its eye, heading or the scene
  // changes: figures stand still most of the time, and on tile-based
  // (mobile) GPUs writing 16 MB of depth per figure per frame is the costly
  // part. Rendered before the pane's main pass so the pane framebuffer is
  // never switched away from mid-pass.
  struct FovMapSlot {
    int unitId = -1;
    int sector = -1;  // -1: the figure's FOV cone; 0-2: a free-aim sector.
    glm::vec3 eye{0.0f};
    float yaw = 0.0f;
    bool drawTerrain = false;
    float mapHalfExtent = 0.0f;
    unsigned long long generation = 0;  // fovMapGeneration_ when rendered.
    unsigned long long lastUsed = 0;    // fovMapClock_ when last requested.
    GLuint fbo[2] = {0, 0};
    GLuint tex[2] = {0, 0};
    glm::mat4 matrix[2]{glm::mat4(1.0f), glm::mat4(1.0f)};
  };
  // One figure's mask in a DrawFovMasks call: its slot and cone half-angle.
  struct FovMask {
    int slot = -1;
    float halfFovDegrees = 0.0f;
  };
  std::vector<FovMapSlot> fovMaps_;
  unsigned long long fovMapGeneration_ = 1;  // Bumped when the casters change.
  unsigned long long fovMapClock_ = 0;       // Counts RenderPane calls.
  // Visual-only building massing (issue #151: parapets, roof furniture,
  // awnings), one static mesh rebuilt when the scene's massing records
  // change. Drawn in the shadow and lit passes only.
  LitTriangleMesh massingMesh_;
  unsigned long long massingKey_ = 0;
  // Facade openings as pane extrusions (issue #152): one static mesh
  // (frames, sills, mullions, niche reveals/backs; building color) rebuilt when the scene's opening records change. Drawn in the lit pass
  // right after the building prisms. Recessed openings (doors, factory
  // windows) also cut holes into their wall faces: the rectangles per
  // obstacle/edge are cached alongside the meshes.
  LitTriangleMesh facadeMesh_;
  std::unordered_map<int, std::vector<std::vector<glm::vec4>>> facadeRecessHoles_;
  unsigned long long facadeKey_ = 0;
  // All walk surfaces (decks, ramps, roof slabs, facade stoops/platforms)
  // batched into one static mesh for the shadow and lit passes; the
  // per-surface path remains only for the camera-fade re-draw.
  LitTriangleMesh deckMesh_;
  unsigned long long deckKey_ = 0;
  LitTriangleMesh obstacleMesh_;        // Drawn prisms, recessed openings cut out.
  LitTriangleMesh obstacleCasterMesh_;  // The same, solid, for the shadow map.
  unsigned long long obstacleKey_ = 0;
  std::vector<int> surfaceBuilding_;  // Roof slab -> building, see RoofBuildings.
  unsigned long long surfaceBuildingKey_ = 0;
  // The slot holding `unit`'s depth maps looking along `facingYaw` (for
  // `sector`, see FovMapSlot), rendering them first if the cached ones are
  // stale. Binds the slot's framebuffers when it renders, so call it before
  // the pane's main pass. -1 if the maps could not be allocated.
  int AcquireFovMaps(const tactics::Unit& unit, int sector, float facingYaw, bool drawTerrain,
                     float mapHalfExtent);
  // Re-draws the static scene into the pane with the mask shader, tinted
  // `color` wherever any of `masks`' figures sees it. Leaves the pane
  // framebuffer, viewport, scissor and the FOV-overlay blend/stencil state
  // as it found them.
  void DrawFovMasks(const std::vector<FovMask>& masks, const glm::vec4& color,
                    const glm::mat4& viewProj, bool drawTerrain, float mapHalfExtent);
  void DestroyFovMaps(FovMapSlot* slot);
};

}  // namespace gfx
