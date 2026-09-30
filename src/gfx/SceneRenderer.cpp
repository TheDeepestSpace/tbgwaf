#include "gfx/SceneRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Raycast.h"

using tactics::AABB;
using tactics::GameLogic;
using tactics::InputMode;
using tactics::Team;
using tactics::TeamVisibility;
using tactics::Unit;

namespace gfx {
namespace {

constexpr int kShadowMapSize = 2048;

// Flat, unlit shader used for UI-ish overlays (selection highlights, the
// move-path preview line) that should stay crisp regardless of shadowing.
const char* kUnlitVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kUnlitFragmentShaderSrc = R"(#version 300 es
precision mediump float;
uniform vec4 uColor;
out vec4 FragColor;
void main() {
  FragColor = uColor;
}
)";

// Lit shader used for the actual scene geometry (ground, obstacles,
// figures): a single directional light with a basic shadow map, and a
// per-face normal derived from screen-space position derivatives (GLSL ES
// 3.00 has dFdx/dFdy as core, so every cube face gets a correct flat normal
// without needing a dedicated per-face-vertex mesh).
const char* kLitVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat4 uLightSpaceMatrix;
out vec3 vWorldPos;
out vec4 vLightSpacePos;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorldPos = world.xyz;
  vLightSpacePos = uLightSpaceMatrix * world;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kLitFragmentShaderSrc = R"(#version 300 es
precision highp float;
in vec3 vWorldPos;
in vec4 vLightSpacePos;
uniform vec4 uColor;
uniform vec3 uLightDir;  // Direction the light travels; surfaces face -uLightDir.
uniform vec3 uViewPos;
uniform sampler2D uShadowMap;
out vec4 FragColor;

float ComputeShadow(vec3 normal) {
  vec3 proj = vLightSpacePos.xyz / vLightSpacePos.w;
  proj = proj * 0.5 + 0.5;
  if (proj.x < 0.0 || proj.x > 1.0 || proj.y < 0.0 || proj.y > 1.0 || proj.z > 1.0) {
    return 0.0;  // Outside the light's frustum: treat as unshadowed.
  }
  float closestDepth = texture(uShadowMap, proj.xy).r;
  float currentDepth = proj.z;
  float bias = max(0.003 * (1.0 - max(dot(normal, -uLightDir), 0.0)), 0.0008);
  return (currentDepth - bias) > closestDepth ? 1.0 : 0.0;
}

void main() {
  vec3 normal = normalize(cross(dFdx(vWorldPos), dFdy(vWorldPos)));
  vec3 viewDir = normalize(uViewPos - vWorldPos);
  if (dot(normal, viewDir) < 0.0) normal = -normal;

  float diffuse = max(dot(normal, -uLightDir), 0.0);
  float shadow = ComputeShadow(normal);
  const float kAmbient = 0.35;
  float lit = kAmbient + (1.0 - shadow) * diffuse * 0.65;
  FragColor = vec4(uColor.rgb * lit, uColor.a);
}
)";

// Depth-only shader for the shadow map pass.
const char* kDepthVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uLightMVP;
void main() {
  gl_Position = uLightMVP * vec4(aPos, 1.0);
}
)";

const char* kDepthFragmentShaderSrc = R"(#version 300 es
precision mediump float;
void main() {}
)";

void DrawBox(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
             const glm::vec3& minCorner, const glm::vec3& size, const glm::vec4& color) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetVec4("uColor", color);
  cube.Draw();
}

// Lit variant that takes a full model matrix rather than an axis-aligned
// minCorner/size pair, for geometry (e.g. the gun box) that needs a
// rotation term a translate*scale composition can't express. Also uploads
// the model matrix (for world-space position/light coordinates in the
// fragment shader) alongside the color. Light direction, view position, and
// the shadow map itself are set once per pane, not per-object, since they
// don't vary between draw calls.
void DrawBoxLitModel(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
                     const glm::mat4& lightSpaceMatrix, const glm::mat4& model,
                     const glm::vec4& color) {
  shader.SetMat4("uModel", model);
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetMat4("uLightSpaceMatrix", lightSpaceMatrix);
  shader.SetVec4("uColor", color);
  cube.Draw();
}

void DrawBoxLit(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
                const glm::mat4& lightSpaceMatrix, const glm::vec3& minCorner,
                const glm::vec3& size, const glm::vec4& color) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
  DrawBoxLitModel(shader, cube, viewProj, lightSpaceMatrix, model, color);
}

void DrawBoxDepth(const Shader& shader, const CubeMesh& cube, const glm::mat4& lightSpaceMatrix,
                  const glm::vec3& minCorner, const glm::vec3& size) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
  shader.SetMat4("uLightMVP", lightSpaceMatrix * model);
  cube.Draw();
}

void UnitBoxes(const Unit& unit, glm::vec3* outBodyMin, glm::vec3* outBodySize,
               glm::vec3* outHeadMin, glm::vec3* outHeadSize) {
  constexpr float kHalfWidth = tactics::constants::kUnitHalfWidth;
  constexpr float kBodyHeight = 1.4f;
  constexpr float kHeadSize = 0.4f;
  *outBodyMin = unit.position - glm::vec3(kHalfWidth, 0.0f, kHalfWidth);
  *outBodySize = glm::vec3(kHalfWidth * 2.0f, kBodyHeight, kHalfWidth * 2.0f);
  *outHeadMin = unit.position + glm::vec3(-kHeadSize * 0.5f, kBodyHeight, -kHeadSize * 0.5f);
  *outHeadSize = glm::vec3(kHeadSize, kHeadSize, kHeadSize);
}

// Model matrix for a small gun box attached to the body, oriented along
// unit.facingYaw and offset forward from unit.position. The rotation axis
// is (0,-1,0) rather than the more usual (0,1,0): FacingDirection() defines
// "forward" directly as (cos(yaw), 0, sin(yaw)) rather than via a rotation
// matrix, and glm::rotate(yaw, {0,1,0}) turns the local +X axis into
// (cos(yaw), 0, -sin(yaw)) — the mirror image. Negating the axis cancels
// that sign flip so the gun visually points the same way as the FOV cone.
glm::mat4 GunModel(const Unit& unit) {
  constexpr float kGunLength = 0.5f;
  constexpr float kGunThickness = 0.08f;
  constexpr float kGunChestHeight = 0.9f;
  const glm::vec3 size(kGunLength, kGunThickness, kGunThickness);
  const glm::vec3 localMin(tactics::constants::kUnitHalfWidth, kGunChestHeight,
                           -kGunThickness * 0.5f);
  return glm::translate(glm::mat4(1.0f), unit.position) *
         glm::rotate(glm::mat4(1.0f), unit.facingYaw, glm::vec3(0.0f, -1.0f, 0.0f)) *
         glm::translate(glm::mat4(1.0f), localMin) * glm::scale(glm::mat4(1.0f), size);
}

void DrawUnit(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
              const glm::mat4& lightSpaceMatrix, const Unit& unit) {
  const glm::vec4 color = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                  : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  constexpr glm::vec4 kGunColor(0.12f, 0.12f, 0.12f, 1.0f);
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  DrawBoxLit(shader, cube, viewProj, lightSpaceMatrix, bodyMin, bodySize, color);
  DrawBoxLit(shader, cube, viewProj, lightSpaceMatrix, headMin, headSize, color);
  DrawBoxLitModel(shader, cube, viewProj, lightSpaceMatrix, GunModel(unit), kGunColor);
}

void DrawUnitDepth(const Shader& shader, const CubeMesh& cube, const glm::mat4& lightSpaceMatrix,
                   const Unit& unit) {
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  DrawBoxDepth(shader, cube, lightSpaceMatrix, bodyMin, bodySize);
  DrawBoxDepth(shader, cube, lightSpaceMatrix, headMin, headSize);
}

void DrawHighlight(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
                   const glm::vec3& position, const glm::vec4& color) {
  constexpr float kHalf = 0.5f;
  const glm::vec3 minCorner = position + glm::vec3(-kHalf, 0.01f, -kHalf);
  DrawBox(shader, cube, viewProj, minCorner, glm::vec3(kHalf * 2.0f, 0.04f, kHalf * 2.0f), color);
}

// Renders a unit's FOV as a flat, ground-level, lightly team-colored
// translucent triangle fan spanning kShootHalfFovDegrees around
// FacingDirection(), capped at kFovConeVisualRange (bigger than the map
// diagonal, so it always visually reaches the map edge). Each ray stops at the
// nearest obstacle footprint hit from the unit's eye position; all obstacles
// occlude, even ones shorter than eye height.
// Caller is responsible for enabling blending around this call.
void DrawFovCone(const Shader& shader, TriangleFanMesh& mesh, const glm::mat4& viewProj,
                 const Unit& unit, const std::vector<tactics::Obstacle>& obstacles) {
  constexpr int kArcSegments = 24;
  constexpr float kGroundOffset = 0.015f;
  constexpr float kConeAlpha = 0.15f;
  // Angular nudge to either side of an obstacle corner: one ray lands on the
  // occluding face right at the corner, the other shoots past it.
  constexpr float kCornerEpsilon = 1e-3f;
  const float halfFovRad = glm::radians(tactics::constants::kShootHalfFovDegrees);
  const float range = tactics::constants::kFovConeVisualRange;
  const glm::vec3 eye = unit.EyePosition();

  // Boundary ray angles as offsets from facingYaw in [-halfFov, +halfFov].
  // A uniform fan alone puts the occlusion edge on a chord between the two
  // samples straddling an obstacle corner, which reads as a skewed edge that
  // misses the corner; casting extra rays at each obstacle
  // corner (nudged to either side) pins the edge exactly onto the corner.
  std::vector<float> offsets;
  offsets.reserve(kArcSegments + 1 + obstacles.size() * 12);
  for (int i = 0; i <= kArcSegments; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kArcSegments);
    offsets.push_back(-halfFovRad + 2.0f * halfFovRad * t);
  }
  constexpr float kTwoPi = 6.28318530717958647692f;
  for (const auto& obstacle : obstacles) {
    const AABB& b = obstacle.bounds;
    for (const float x : {b.min.x, b.max.x}) {
      for (const float z : {b.min.z, b.max.z}) {
        const float delta =
            std::remainder(std::atan2(z - eye.z, x - eye.x) - unit.facingYaw, kTwoPi);
        for (const float nudged : {delta - kCornerEpsilon, delta, delta + kCornerEpsilon}) {
          if (nudged >= -halfFovRad && nudged <= halfFovRad) offsets.push_back(nudged);
        }
      }
    }
  }
  std::sort(offsets.begin(), offsets.end());

  std::vector<glm::vec3> points;
  points.reserve(offsets.size() + 1);
  points.push_back(unit.position + glm::vec3(0.0f, kGroundOffset, 0.0f));
  for (const float offset : offsets) {
    const float angle = unit.facingYaw + offset;
    const glm::vec3 dir(std::cos(angle), 0.0f, std::sin(angle));
    float reach = range;
    for (const auto& obstacle : obstacles) {
      // The cone is a ground overlay, so every obstacle occludes it by its
      // footprint -- including ones shorter than eye height. Clamp the ray
      // origin's height into the box so the horizontal ray tests footprint only.
      const AABB& b = obstacle.bounds;
      const glm::vec3 origin(eye.x, glm::clamp(eye.y, b.min.y, b.max.y), eye.z);
      float hitT = 0.0f;
      if (tactics::RayIntersectsAABB(origin, dir, b, &hitT) && hitT < reach) {
        reach = hitT;
      }
    }
    points.push_back(unit.position + glm::vec3(dir.x * reach, kGroundOffset, dir.z * reach));
  }
  mesh.SetPoints(points);

  const glm::vec4 baseColor = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                      : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  shader.SetMat4("uMVP", viewProj);
  shader.SetVec4("uColor", glm::vec4(baseColor.r, baseColor.g, baseColor.b, kConeAlpha));
  mesh.Draw();
}

}  // namespace

bool IsUnitVisibleForRender(const Unit& unit, Team viewingTeam, bool fogActive,
                            const TeamVisibility& visibility) {
  if (unit.team == viewingTeam) return true;
  if (!fogActive) return true;
  return visibility.UnitVisible(unit.id);
}

bool SceneRenderer::Init() {
  if (!unlitShader_.Compile(kUnlitVertexShaderSrc, kUnlitFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the unlit shader\n");
    return false;
  }
  if (!litShader_.Compile(kLitVertexShaderSrc, kLitFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the lit shader\n");
    return false;
  }
  if (!depthShader_.Compile(kDepthVertexShaderSrc, kDepthFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the shadow depth shader\n");
    return false;
  }
  cubeMesh_.Init();
  pathLine_.Init();
  fovConeMesh_.Init();

  // Stage-C: a single directional light (simulating overhead factory
  // lighting) casting a basic shadow map, single cascade, hard-edged. The
  // light and the static map geometry are shared by all panes; only the
  // *casters* (which units are drawn into it) change per pane, since each
  // team's shadow map must not leak the position of units hidden by their
  // own fog-of-war. The single FBO/texture is simply re-rendered once per
  // pane, immediately before that pane's color pass consumes it.
  glGenFramebuffers(1, &shadowFbo_);
  glGenTextures(1, &shadowDepthTex_);
  glBindTexture(GL_TEXTURE_2D, shadowDepthTex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowMapSize, kShadowMapSize, 0,
               GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowDepthTex_, 0);
  {
    const GLenum noColorBuffer = GL_NONE;
    glDrawBuffers(1, &noColorBuffer);
    glReadBuffer(GL_NONE);
  }
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    std::fprintf(stderr, "Shadow map framebuffer incomplete\n");
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    return false;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // The light and map geometry are both static, so the light-space matrix
  // is fixed for the whole session.
  lightDir_ = glm::normalize(glm::vec3(0.35f, -1.0f, 0.25f));
  const float mapHalfExtentForLight = tactics::constants::kMapHalfExtent;
  const float lightDistance = mapHalfExtentForLight * 3.0f;
  const glm::vec3 lightPos = -lightDir_ * lightDistance;
  const glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const float orthoHalfExtent = mapHalfExtentForLight * 1.5f;
  const glm::mat4 lightProj =
      glm::ortho(-orthoHalfExtent, orthoHalfExtent, -orthoHalfExtent, orthoHalfExtent, 0.1f,
                 lightDistance * 2.0f);
  lightSpaceMatrix_ = lightProj * lightView;

  glEnable(GL_DEPTH_TEST);
  return true;
}

void SceneRenderer::Destroy() {
  pathLine_.Destroy();
  fovConeMesh_.Destroy();
  cubeMesh_.Destroy();
  if (shadowDepthTex_) glDeleteTextures(1, &shadowDepthTex_);
  if (shadowFbo_) glDeleteFramebuffers(1, &shadowFbo_);
  shadowDepthTex_ = 0;
  shadowFbo_ = 0;
}

void SceneRenderer::RenderPane(const GameLogic& game, Team team, bool fogActive,
                               const TeamVisibility& visibility, const OrbitCamera& camera, int x,
                               int y, int width, int height, const PaneOverlays& overlays,
                               GLuint targetFramebuffer) {
  const auto& obstacles = game.GetScene().obstacles;

  // Shadow pass: only casters this team can currently see.
  glDisable(GL_SCISSOR_TEST);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
  glViewport(0, 0, kShadowMapSize, kShadowMapSize);
  glClear(GL_DEPTH_BUFFER_BIT);
  depthShader_.Use();
  for (const auto& obstacle : obstacles) {
    const AABB& bounds = obstacle.bounds;
    DrawBoxDepth(depthShader_, cubeMesh_, lightSpaceMatrix_, bounds.min, bounds.max - bounds.min);
  }
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive) continue;
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawUnitDepth(depthShader_, cubeMesh_, lightSpaceMatrix_, unit);
  }
  glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);

  // Scissor, not just viewport, is required here: glClear() would
  // otherwise clear the whole framebuffer (including other panes'
  // already-drawn rects) regardless of the glViewport rect.
  glEnable(GL_SCISSOR_TEST);
  glViewport(x, y, width, height);
  glScissor(x, y, width, height);
  glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
  glClearStencil(0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  const glm::mat4 view = camera.ViewMatrix();
  const glm::mat4 proj =
      camera.ProjectionMatrix(static_cast<float>(width) / static_cast<float>(height));
  const glm::mat4 viewProj = proj * view;

  litShader_.Use();
  litShader_.SetVec3("uLightDir", lightDir_);
  litShader_.SetVec3("uViewPos", camera.Position());
  litShader_.SetInt("uShadowMap", 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, shadowDepthTex_);

  const float mapHalfExtent = tactics::constants::kMapHalfExtent;
  DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_,
             glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
             glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f),
             glm::vec4(0.16f, 0.18f, 0.20f, 1.0f));

  for (size_t i = 0; i < obstacles.size(); ++i) {
    const bool obstacleVisible = !fogActive || visibility.ObstacleVisible(i);
    const AABB& bounds = obstacles[i].bounds;
    const glm::vec4 baseColor = obstacles[i].climbable ? glm::vec4(0.55f, 0.48f, 0.3f, 1.0f)
                                                       : glm::vec4(0.55f, 0.55f, 0.6f, 1.0f);
    const glm::vec4 color = obstacleVisible ? baseColor : glm::vec4(0.22f, 0.22f, 0.24f, 1.0f);
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, bounds.min,
               bounds.max - bounds.min, color);
  }

  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive) continue;
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawUnit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, unit);
  }

  // Each pane shows only its own team's FOV cones -- your own vision,
  // not intel about what the enemy can see. Translucent overlay: blend
  // on, no depth writes (so it never occludes anything drawn after it).
  // Every cone in a pane shares identical color+alpha, so where
  // teammates' cones overlap, blending each one in would compound into
  // a darker/more opaque patch that isn't actually meaningful (it's
  // still just this team's own vision). The stencil buffer (cleared per
  // pane above) caps each pixel to a single cone's worth of blending:
  // the first cone to touch a pixel blends and claims it (stencil
  // 0 -> 1), any later cone covering that same pixel is discarded, so
  // overlaps read as one flat shade instead of stacking.
  unlitShader_.Use();
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_EQUAL, 0, 0xFF);
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.team != team) continue;
    DrawFovCone(unlitShader_, fovConeMesh_, viewProj, unit, obstacles);
  }
  glDisable(GL_STENCIL_TEST);
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);

  if (const auto actorId = game.CurrentActorId(); actorId && game.Mode() != InputMode::GameOver) {
    if (const Unit* actor = game.FindUnit(*actorId)) {
      if (IsUnitVisibleForRender(*actor, team, fogActive, visibility)) {
        DrawHighlight(unlitShader_, cubeMesh_, viewProj, actor->position,
                      glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
      }
    }
  }
  // Overwatch indicator: a minimal PoC-grade ground marker (distinct from
  // the white current-actor ring and the yellow selection ring) under
  // every figure currently armed to fire during an enemy's move.
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.triggerAction != tactics::TriggerAction::Shoot) continue;
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawHighlight(unlitShader_, cubeMesh_, viewProj, unit.position,
                  glm::vec4(1.0f, 0.55f, 0.0f, 1.0f));
  }
  // Selection/move-preview overlays belong to whichever pane the input
  // layer says is acting; callers pass them only for that pane.
  if (overlays.selectionHighlight) {
    DrawHighlight(unlitShader_, cubeMesh_, viewProj, *overlays.selectionHighlight,
                  glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
  }
  if (overlays.movePreviewPath && overlays.movePreviewPath->size() >= 2) {
    pathLine_.SetPoints(*overlays.movePreviewPath);
    unlitShader_.SetMat4("uMVP", viewProj);
    unlitShader_.SetVec4("uColor", glm::vec4(1.0f, 0.85f, 0.2f, 1.0f));
    pathLine_.Draw();
    // Mark the final position with the same square used for selection.
    DrawHighlight(unlitShader_, cubeMesh_, viewProj, overlays.movePreviewPath->back(),
                  glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
  } else if (overlays.invalidHoverHighlight) {
    DrawHighlight(unlitShader_, cubeMesh_, viewProj, *overlays.invalidHoverHighlight,
                  glm::vec4(0.9f, 0.15f, 0.15f, 1.0f));
  }
  glDisable(GL_SCISSOR_TEST);
}

}  // namespace gfx
