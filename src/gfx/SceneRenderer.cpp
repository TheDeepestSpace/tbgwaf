#include "gfx/SceneRenderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include <glm/gtc/matrix_transform.hpp>

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

// Tip-over transform pivoting at the feet (unit.position) around
// knockdownAxis, ease-out from 0 to ~85 degrees. Identity for standing units.
glm::mat4 KnockdownModel(const Unit& unit) {
  if (unit.alive || unit.knockdownElapsed < 0.0f) return glm::mat4(1.0f);
  constexpr float kMaxTilt = glm::radians(85.0f);
  const float t = glm::clamp(unit.knockdownElapsed / tactics::constants::kKnockdownDuration, 0.0f,
                             1.0f);
  const float eased = 1.0f - (1.0f - t) * (1.0f - t);
  return glm::translate(glm::mat4(1.0f), unit.position) *
         glm::rotate(glm::mat4(1.0f), kMaxTilt * eased, unit.knockdownAxis) *
         glm::translate(glm::mat4(1.0f), -unit.position);
}

glm::mat4 BoxModel(const glm::vec3& minCorner, const glm::vec3& size) {
  return glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
}

void DrawUnit(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
              const glm::mat4& lightSpaceMatrix, const Unit& unit) {
  const glm::vec4 color = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                  : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  constexpr glm::vec4 kGunColor(0.12f, 0.12f, 0.12f, 1.0f);
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  const glm::mat4 fall = KnockdownModel(unit);
  DrawBoxLitModel(shader, cube, viewProj, lightSpaceMatrix, fall * BoxModel(bodyMin, bodySize),
                  color);
  DrawBoxLitModel(shader, cube, viewProj, lightSpaceMatrix, fall * BoxModel(headMin, headSize),
                  color);
  DrawBoxLitModel(shader, cube, viewProj, lightSpaceMatrix, fall * GunModel(unit), kGunColor);
}

// Wireframe unit cube transformed by `model` (same convention as BoxModel:
// the unit cube [0,1]^3), drawn as one line strip that retraces a few edges
// to cover all 12. Line strips rather than GL_LINE polygon mode, which
// WebGL2/GLES don't have.
void DrawWireBox(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                 const glm::mat4& model, const glm::vec4& color) {
  glm::vec3 b[4] = {{0, 0, 0}, {1, 0, 0}, {1, 0, 1}, {0, 0, 1}};
  glm::vec3 t[4] = {{0, 1, 0}, {1, 1, 0}, {1, 1, 1}, {0, 1, 1}};
  const std::vector<glm::vec3> local = {b[0], b[1], b[2], b[3], b[0], t[0], t[1], b[1],
                                        t[1], t[2], b[2], t[2], t[3], b[3], t[3], t[0]};
  std::vector<glm::vec3> points;
  points.reserve(local.size());
  for (const glm::vec3& p : local) points.push_back(glm::vec3(model * glm::vec4(p, 1.0f)));
  lines.SetPoints(points);
  shader.SetMat4("uMVP", viewProj);
  shader.SetVec4("uColor", color);
  lines.Draw();
}

void DrawUnitWireframe(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                       const Unit& unit) {
  // Common highlight green for both teams.
  const glm::vec4 color(0.3f, 0.9f, 0.4f, 1.0f);
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  DrawWireBox(shader, lines, viewProj, BoxModel(bodyMin, bodySize), color);
  DrawWireBox(shader, lines, viewProj, BoxModel(headMin, headSize), color);
  DrawWireBox(shader, lines, viewProj, GunModel(unit), color);
}

void DrawUnitDepth(const Shader& shader, const CubeMesh& cube, const glm::mat4& lightSpaceMatrix,
                   const Unit& unit) {
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  const glm::mat4 fall = KnockdownModel(unit);
  shader.SetMat4("uLightMVP", lightSpaceMatrix * fall * BoxModel(bodyMin, bodySize));
  cube.Draw();
  shader.SetMat4("uLightMVP", lightSpaceMatrix * fall * BoxModel(headMin, headSize));
  cube.Draw();
}

void DrawHighlight(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
                   const glm::vec3& position, const glm::vec4& color) {
  constexpr float kHalf = 0.5f;
  const glm::vec3 minCorner = position + glm::vec3(-kHalf, 0.01f, -kHalf);
  DrawBox(shader, cube, viewProj, minCorner, glm::vec3(kHalf * 2.0f, 0.04f, kHalf * 2.0f), color);
}

// A [begin, end) stretch of ground along one sight ray, as horizontal
// distances from the eye.
struct GroundSpan {
  float begin = 0.0f;
  float end = 0.0f;
};

// Horizontal distances at which a ray from `eye` along the XZ direction
// `dir` enters and exits `box`'s footprint. False if the footprint is missed
// entirely (or lies fully behind the eye).
bool FootprintSpan(const glm::vec3& eye, const glm::vec2& dir, const AABB& box, float* outEnter,
                   float* outExit) {
  float enter = 0.0f;
  float exit = std::numeric_limits<float>::max();
  const float origin[2] = {eye.x, eye.z};
  const float d[2] = {dir.x, dir.y};
  const float boxMin[2] = {box.min.x, box.min.z};
  const float boxMax[2] = {box.max.x, box.max.z};
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(d[axis]) < 1e-8f) {
      if (origin[axis] < boxMin[axis] || origin[axis] > boxMax[axis]) return false;
      continue;
    }
    float t0 = (boxMin[axis] - origin[axis]) / d[axis];
    float t1 = (boxMax[axis] - origin[axis]) / d[axis];
    if (t0 > t1) std::swap(t0, t1);
    enter = std::max(enter, t0);
    exit = std::min(exit, t1);
  }
  if (exit < enter) return false;
  *outEnter = enter;
  *outExit = exit;
  return true;
}

// `range` clipped to where the ray leaves the playable map footprint, so the
// FOV cone stops at the boundary. Zero if the eye stands outside the map and
// the ray never enters it.
float ClipToMap(const glm::vec3& eye, const glm::vec2& dir, float range, float half) {
  AABB map;
  map.min = glm::vec3(-half, 0.0f, -half);
  map.max = glm::vec3(half, 0.0f, half);
  float enter = 0.0f;
  float exit = 0.0f;
  if (!FootprintSpan(eye, dir, map, &enter, &exit)) return 0.0f;
  return std::min(range, exit);
}

// The stretch of ground along one sight ray that the eye cannot see, i.e.
// where a target could hide crouched at ground level. The sightline from the
// eye down to the ground point at distance t drops linearly from eye.y to 0,
// so the box hides the ground from its near face until the sightline over
// its top far edge lands: t = exit * eye.y / (eye.y - top). A box whose top
// reaches eye level hides everything behind it.
bool GroundShadow(const glm::vec3& eye, const glm::vec2& dir, const AABB& box, float range,
                  GroundSpan* outShadow) {
  float enter = 0.0f;
  float exit = 0.0f;
  if (!FootprintSpan(eye, dir, box, &enter, &exit)) return false;
  if (box.min.y >= eye.y) return false;  // Sightlines only descend; a box above the eye never blocks.
  // A raised box bottom lets sightlines pass underneath: the shadow only
  // starts once the sightline through the bottom near edge lands.
  const float begin = enter / (1.0f - box.min.y / eye.y);
  const float end =
      box.max.y >= eye.y ? range : std::min(range, exit * eye.y / (eye.y - box.max.y));
  if (begin >= end || begin >= range) return false;
  *outShadow = GroundSpan{begin, end};
  return true;
}

// The visible stretches of ground along one sight ray within [0, range]:
// the complement of the union of every obstacle's ground shadow.
std::vector<GroundSpan> VisibleGroundSpans(const glm::vec3& eye, const glm::vec2& dir,
                                           const std::vector<tactics::Obstacle>& obstacles,
                                           float range) {
  std::vector<GroundSpan> shadows;
  for (const auto& obstacle : obstacles) {
    GroundSpan shadow;
    if (GroundShadow(eye, dir, obstacle.bounds, range, &shadow)) shadows.push_back(shadow);
  }
  std::sort(shadows.begin(), shadows.end(),
            [](const GroundSpan& a, const GroundSpan& b) { return a.begin < b.begin; });

  std::vector<GroundSpan> visible;
  float cursor = 0.0f;
  for (const auto& shadow : shadows) {
    if (shadow.begin > cursor) visible.push_back(GroundSpan{cursor, shadow.begin});
    cursor = std::max(cursor, shadow.end);
    if (cursor >= range) break;
  }
  if (cursor < range) visible.push_back(GroundSpan{cursor, range});
  return visible;
}

// Renders a unit's FOV as a flat, ground-level, lightly team-colored
// translucent overlay spanning kShootHalfFovDegrees around
// FacingDirection(), capped at kFovConeVisualRange (bigger than the map
// diagonal) and clipped at the map boundary. Occlusion is 3D:
// the cone's tip is the unit's eye, so an obstacle below eye level only
// shadows the strip of ground it actually hides -- the overlay resumes where
// the sightline over its top edge lands, and only a target crouched at
// ground level right behind the obstacle stays hidden. Obstacles at or above
// eye level shadow everything behind them.
// Caller is responsible for enabling blending around this call.
void DrawFovCone(const Shader& shader, TriangleMesh& mesh, const glm::mat4& viewProj,
                 const Unit& unit, const std::vector<tactics::Obstacle>& obstacles,
                 const std::vector<AABB>& sidewalks, float mapHalfExtent) {
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

  std::vector<glm::vec2> dirs;
  std::vector<std::vector<GroundSpan>> spansPerRay;
  dirs.reserve(offsets.size());
  spansPerRay.reserve(offsets.size());
  for (const float offset : offsets) {
    const float angle = unit.facingYaw + offset;
    const glm::vec2 dir(std::cos(angle), std::sin(angle));
    dirs.push_back(dir);
    spansPerRay.push_back(VisibleGroundSpans(eye, dir, obstacles, ClipToMap(eye, dir, range, mapHalfExtent)));
  }

  // Stitch adjacent rays into quads, one per matching visible span. Corner
  // rays keep the span structure identical across a slice except in the
  // epsilon-thin slivers at corners, where dropping unmatched spans is
  // invisible. Shadow boundaries of straight box edges are straight lines on
  // the ground, so the quads trace them exactly.
  const auto groundPoint = [&](const glm::vec2& dir, float t) {
    return glm::vec3(eye.x + dir.x * t, kGroundOffset, eye.z + dir.y * t);
  };
  std::vector<glm::vec3> points;
  points.reserve(offsets.size() * 6);
  for (size_t i = 0; i + 1 < offsets.size(); ++i) {
    const auto& left = spansPerRay[i];
    const auto& right = spansPerRay[i + 1];
    const size_t pairCount = std::min(left.size(), right.size());
    for (size_t k = 0; k < pairCount; ++k) {
      const glm::vec3 l0 = groundPoint(dirs[i], left[k].begin);
      const glm::vec3 l1 = groundPoint(dirs[i], left[k].end);
      const glm::vec3 r0 = groundPoint(dirs[i + 1], right[k].begin);
      const glm::vec3 r1 = groundPoint(dirs[i + 1], right[k].end);
      points.push_back(l0);
      points.push_back(r0);
      points.push_back(r1);
      points.push_back(l0);
      points.push_back(r1);
      points.push_back(l1);
    }
  }
  // Every walkable surface gets its own copy of the cone at its own height:
  // the ground-level cone above is buried under raised sidewalk slabs, so
  // clip the cone to each slab's footprint and lay that piece on its top.
  const size_t groundPointCount = points.size();
  for (const AABB& slab : sidewalks) {
    for (size_t i = 0; i + 2 < groundPointCount; i += 3) {
      std::vector<glm::vec2> poly = {{points[i].x, points[i].z},
                                     {points[i + 1].x, points[i + 1].z},
                                     {points[i + 2].x, points[i + 2].z}};
      // Sutherland-Hodgman against the four slab edges.
      for (int edge = 0; edge < 4 && !poly.empty(); ++edge) {
        const auto inside = [&](const glm::vec2& v) {
          switch (edge) {
            case 0: return v.x >= slab.min.x;
            case 1: return v.x <= slab.max.x;
            case 2: return v.y >= slab.min.z;
            default: return v.y <= slab.max.z;
          }
        };
        const auto cross = [&](const glm::vec2& a, const glm::vec2& b) {
          const float bound = edge == 0 ? slab.min.x : edge == 1 ? slab.max.x
                              : edge == 2 ? slab.min.z : slab.max.z;
          const float t = edge < 2 ? (bound - a.x) / (b.x - a.x) : (bound - a.y) / (b.y - a.y);
          return a + (b - a) * t;
        };
        std::vector<glm::vec2> out;
        for (size_t v = 0; v < poly.size(); ++v) {
          const glm::vec2& cur = poly[v];
          const glm::vec2& prev = poly[(v + poly.size() - 1) % poly.size()];
          if (inside(cur)) {
            if (!inside(prev)) out.push_back(cross(prev, cur));
            out.push_back(cur);
          } else if (inside(prev)) {
            out.push_back(cross(prev, cur));
          }
        }
        poly = std::move(out);
      }
      const float y = slab.max.y + kGroundOffset;
      for (size_t v = 1; v + 1 < poly.size(); ++v) {
        points.emplace_back(poly[0].x, y, poly[0].y);
        points.emplace_back(poly[v].x, y, poly[v].y);
        points.emplace_back(poly[v + 1].x, y, poly[v + 1].y);
      }
    }
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

  // The light direction is fixed; the light-space matrix depends on the
  // scene's map size and is recomputed per RenderPane.
  lightDir_ = glm::normalize(glm::vec3(0.35f, -1.0f, 0.25f));

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
  const float mapHalfExtent = game.GetScene().mapHalfExtent;

  // Light frustum sized to cover the whole map.
  const float lightDistance = mapHalfExtent * 3.0f;
  const glm::mat4 lightView = glm::lookAt(-lightDir_ * lightDistance, glm::vec3(0.0f),
                                          glm::vec3(0.0f, 1.0f, 0.0f));
  const float orthoHalfExtent = mapHalfExtent * 1.5f;
  lightSpaceMatrix_ = glm::ortho(-orthoHalfExtent, orthoHalfExtent, -orthoHalfExtent,
                                 orthoHalfExtent, 0.1f, lightDistance * 2.0f) *
                      lightView;

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

  DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_,
             glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
             glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f),
             glm::vec4(0.16f, 0.18f, 0.20f, 1.0f));

  for (const AABB& slab : game.GetScene().sidewalks) {
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, slab.min, slab.max - slab.min,
               glm::vec4(0.36f, 0.37f, 0.39f, 1.0f));
  }

  for (size_t i = 0; i < obstacles.size(); ++i) {
    const bool obstacleVisible = !fogActive || visibility.ObstacleVisible(i);
    const AABB& bounds = obstacles[i].bounds;
    const glm::vec4 color = obstacleVisible ? glm::vec4(0.55f, 0.55f, 0.6f, 1.0f) : glm::vec4(0.22f, 0.22f, 0.24f, 1.0f);
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, bounds.min,
               bounds.max - bounds.min, color);
  }

  for (const Unit& unit : game.GetScene().units) {
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
  // The cone sits a hair above the ground; on a large map the depth buffer's
  // resolution at distance exceeds that gap, so bias it toward the camera to
  // avoid z-fighting speckle.
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(-2.0f, -4.0f);
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.team != team) continue;
    DrawFovCone(unlitShader_, fovConeMesh_, viewProj, unit, obstacles,
                game.GetScene().sidewalks, game.GetScene().mapHalfExtent);
  }
  glDisable(GL_POLYGON_OFFSET_FILL);
  glDisable(GL_STENCIL_TEST);
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);

  // WEGO planning: both teams plan concurrently, so during the planning
  // phase every pane highlights its own team's living figures (dim white =
  // still needs a plan, green = plan set) -- this is squad-wide, not a
  // single actor.
  const bool planning =
      game.Mode() != InputMode::GameOver && game.Mode() != InputMode::Executing;
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || unit.team != team) continue;
      if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
      const bool planned = unit.plan.type != tactics::PlannedActionType::None;
      const glm::vec4 color = planned ? glm::vec4(0.25f, 0.9f, 0.35f, 1.0f)
                                        : glm::vec4(1.0f, 1.0f, 1.0f, 0.6f);
      DrawHighlight(unlitShader_, cubeMesh_, viewProj, unit.position, color);
    }
  }
  // Overwatch indicator: a minimal PoC-grade ground marker (distinct from
  // the plan-then-commit ring and the yellow selection ring) under
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
  // An executing round animates both teams' planned moves at once; each
  // pane highlights its own team's figures currently mid-move.
  if (game.Mode() == InputMode::Executing) {
    for (const Unit& unit : game.GetScene().units) {
      if (unit.team == team && game.IsUnitMoving(unit.id)) {
        DrawHighlight(unlitShader_, cubeMesh_, viewProj, unit.position,
                      glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
      }
    }
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
  // Visual feedback for the whole squad's plan so far: a planned move reuses
  // the same path-line rendering as the live preview above; a planned shot
  // gets a simple shooter->target line. Own team only -- the enemy's plans
  // stay hidden even where its figures are visible.
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || unit.team != team) continue;
      if (unit.plan.type == tactics::PlannedActionType::Move && unit.plan.movePath.size() >= 2) {
        pathLine_.SetPoints(unit.plan.movePath);
        unlitShader_.SetMat4("uMVP", viewProj);
        unlitShader_.SetVec4("uColor", glm::vec4(0.3f, 0.9f, 0.4f, 1.0f));
        pathLine_.Draw();
        // Wireframe stand-in at the destination, showing the planned final
        // facing (persists until the turn is committed).
        Unit ghost = unit;
        ghost.position = unit.plan.movePath.back();
        ghost.facingYaw = unit.plan.endFacingYaw;
        DrawUnitWireframe(unlitShader_, pathLine_, viewProj, ghost);
      } else if (unit.plan.type == tactics::PlannedActionType::Shoot) {
        if (const Unit* shotTarget = game.FindUnit(unit.plan.shootTargetId)) {
          const std::vector<glm::vec3> shotLine = {unit.EyePosition(), shotTarget->EyePosition()};
          pathLine_.SetPoints(shotLine);
          unlitShader_.SetMat4("uMVP", viewProj);
          unlitShader_.SetVec4("uColor", glm::vec4(0.95f, 0.25f, 0.2f, 1.0f));
          pathLine_.Draw();
        }
      }
    }
  }
  glDisable(GL_SCISSOR_TEST);
}

}  // namespace gfx
