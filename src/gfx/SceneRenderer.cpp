#include "gfx/SceneRenderer.h"

#include "gfx/FigureRig.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include "game/Geometry.h"

using tactics::AABB;
using tactics::GameLogic;
using tactics::InputMode;
using tactics::Team;
using tactics::TeamVisibility;
using tactics::Unit;

namespace gfx {

PaneOverlays BuildPaneOverlays(const tactics::GameLogic& game, tactics::Team paneTeam,
                               const std::optional<glm::vec3>& hoveredGroundPoint) {
  PaneOverlays overlays;
  const auto selectedId = game.SelectedUnitId();
  if (!selectedId) return overlays;
  const tactics::Unit* selected = game.FindUnit(*selectedId);
  if (!selected || selected->team != paneTeam) return overlays;
  overlays.selectionHighlight = selected->position;
  if (game.Mode() == tactics::InputMode::AwaitingMoveDestination) {
    overlays.moveFrontier = game.MoveFrontier();
    overlays.ziplineFrontiers = &game.ZiplineFrontiers();
    overlays.movePreviewRides = &game.MovePreviewRides();
    // A later leg's boundary is yellow, matching the selector; the first leg's stays green.
    overlays.moveFrontierSubsequentLeg = selected->plan.type == tactics::PlannedActionType::Move &&
                                         selected->plan.movePath.size() >= 2;
    if (game.MovePreviewValid()) {
      overlays.movePreviewPath = &game.MovePreviewPath();
    } else if (hoveredGroundPoint) {
      overlays.invalidHoverHighlight = hoveredGroundPoint;
    }
  }
  overlays.showShotCone = game.Mode() == tactics::InputMode::AwaitingShootTarget;
  return overlays;
}
namespace {

constexpr int kShadowMapSize = 2048;
// Deck/ramp slabs form one continuous ribbon but are submitted as separate
// shadow casters. A stronger-than-default slope bias keeps each shallow
// joint from shadowing the next slab while the deck still shadows geometry
// below its 0.45-unit thickness.
constexpr float kDeckShadowSlopeBias = 16.0f;
constexpr float kDeckShadowConstantBias = 64.0f;
const glm::vec3 kSetupColor(0.2f, 1.0f, 0.3f);
// Scene geometry base colors, shared by the opaque pass and the issue #136
// see-through pass so a faded block keeps its normal tint.
const glm::vec3 kObstacleColor(0.55f, 0.55f, 0.6f);
const glm::vec3 kDeckColor(0.44f, 0.45f, 0.48f);

// Issue #136 camera-occlusion see-through: map geometry (buildings, deck/
// ramp slabs) standing between the camera and something the player needs to
// see is drawn at this opacity instead of fully opaque. First guess per the
// issue; tune freely.
constexpr float kOccluderFadeOpacity = 0.5f;
// Spacing of the visibility probes laid along paths and across the movement
// frontier. Finer spacing catches narrower slivers of hidden overlay at the
// cost of more camera raycasts per frame.
constexpr float kOcclusionProbeSpacing = 1.0f;

// Flat, unlit shader used for UI-ish overlays (selection highlights, the
// move-path preview line) that should stay crisp regardless of shadowing.
const char* kUnlitVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

// Unlit shader with a per-vertex color, for gradient overlays.
const char* kColorVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uMVP;
out vec4 vColor;
void main() {
  vColor = aColor;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kColorFragmentShaderSrc = R"(#version 300 es
precision mediump float;
in vec4 vColor;
out vec4 FragColor;
void main() {
  FragColor = vColor;
}
)";

// Shot-cone footprint: drawn over world boxes (ground, slabs, obstacles); each
// fragment is kept only if its world position lies inside the cone volume, so
// the cone's true intersection with every surface shows up per pixel.
const char* kConeSurfaceVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vWorld;
out vec3 vWorldNormal;
void main() {
  vWorld = (uModel * vec4(aPos, 1.0)).xyz;
  vWorldNormal = transpose(inverse(mat3(uModel))) * aNormal;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kConeSurfaceFragmentShaderSrc = R"(#version 300 es
precision highp float;
in vec3 vWorld;
in vec3 vWorldNormal;
uniform vec3 uApex;
uniform vec3 uAxis;
uniform vec4 uCone;   // x = tan(half angle), y = clipped range, z = fade range, w = start alpha
uniform vec3 uColor;
uniform int uRequireFacing;
out vec4 FragColor;
void main() {
  vec3 d = vWorld - uApex;
  float t = dot(d, uAxis);
  if (t <= 0.0 || t >= uCone.y) discard;
  float radial = length(d - uAxis * t);
  if (radial > t * uCone.x) discard;
  // A figure receives the overlay only on the side the shot reaches first.
  // World boxes keep showing every cone/surface intersection (including the ground).
  if (uRequireFacing != 0 && dot(normalize(vWorldNormal), -uAxis) <= 0.0) discard;
  // Brightness tracks ShotProfileHitChance: cosine falloff from the axis to
  // the cone edge times quartic falloff with range, so the brightest part of
  // the mask is where a shot is most likely to land.
  float angleFalloff = cos(atan(radial, t) / atan(uCone.x) * 1.57079632679);
  float rangeFalloff = pow(max(1.0 - t / uCone.z, 0.0), 4.0);
  FragColor = vec4(uColor, uCone.w * angleFalloff * rangeFalloff);
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
// vertex normals. CubeMesh duplicates its face vertices for hard edges while
// SphereMesh shares smooth radial normals, so obstacles remain blocky and the
// figures can have the soft silhouette of the concept sketch.
const char* kLitVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat4 uLightSpaceMatrix;
out vec3 vWorldPos;
out vec3 vWorldNormal;
out vec4 vLightSpacePos;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorldPos = world.xyz;
  vWorldNormal = transpose(inverse(mat3(uModel))) * aNormal;
  vLightSpacePos = uLightSpaceMatrix * world;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kLitFragmentShaderSrc = R"(#version 300 es
precision highp float;
in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec4 vLightSpacePos;
uniform vec4 uColor;
uniform vec3 uLightDir;  // Direction the light travels; surfaces face -uLightDir.
uniform vec3 uViewPos;
uniform sampler2D uShadowMap;
uniform int uDisableShadows;
out vec4 FragColor;

float ComputeShadow(vec3 normal) {
  if (uDisableShadows != 0) return 0.0;
  vec3 proj = vLightSpacePos.xyz / vLightSpacePos.w;
  proj = proj * 0.5 + 0.5;
  if (proj.x < 0.0 || proj.x > 1.0 || proj.y < 0.0 || proj.y > 1.0 || proj.z > 1.0) {
    return 0.0;  // Outside the light's frustum: treat as unshadowed.
  }
  float currentDepth = proj.z;
  float bias = max(0.003 * (1.0 - max(dot(normal, -uLightDir), 0.0)), 0.0008);
  vec2 texelSize = 1.0 / vec2(textureSize(uShadowMap, 0));
  float shadow = 0.0;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      float sampledDepth = texture(uShadowMap, proj.xy + vec2(x, y) * texelSize).r;
      shadow += (currentDepth - bias) > sampledDepth ? 1.0 : 0.0;
    }
  }
  return shadow / 9.0;
}

void main() {
  vec3 normal = normalize(vWorldNormal);
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

// Issue #110: per-unit projective FOV mask. The static scene is re-drawn
// with this shader after the lit pass (depth test LEQUAL, no depth writes):
// a fragment is tinted when it faces the unit's eye, lies inside the
// 150-degree horizontal cone (analytic, so the cone's angular edges stay
// crisp), and its eye sightline passes a depth test against one of the two
// eye-space depth maps. Flat tint, no attenuation or N.L -- this is a mask,
// not lighting. Everything else is discarded, which also keeps the stencil
// untouched so overlapping teammates' masks don't stack (see RenderPane).
constexpr int kFovMapWidth = 1024;
constexpr int kFovMapHeight = 2048;
// Each of the two maps covers half the cone's azimuth plus a small overlap
// so the seam between them never shows; the fragment shader cuts the exact
// cone edge analytically.
constexpr float kFovMapHalfSplitDegrees = tactics::constants::kShootHalfFovDegrees * 0.5f;
constexpr float kFovMapHalfAzimuthDegrees = kFovMapHalfSplitDegrees + 1.5f;
// Vertical half-angle. The ground right under a standing viewer (closer
// than eyeHeight / tan(60 deg) = 0.87 units) and anything steeper than 60
// degrees above the eye fall outside both maps and stay untinted.
constexpr float kFovMapHalfElevationDegrees = 60.0f;
constexpr float kFovMapNear = 0.2f;

const char* kFovMaskVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uModel;
out vec3 vWorldPos;
out vec3 vWorldNormal;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorldPos = world.xyz;
  vWorldNormal = transpose(inverse(mat3(uModel))) * aNormal;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kFovMaskFragmentShaderSrc = R"(#version 300 es
precision highp float;
in vec3 vWorldPos;
in vec3 vWorldNormal;
uniform vec3 uEye;
uniform vec3 uFacing;      // Unit forward, horizontal, normalized.
uniform float uCosHalfFov;
uniform float uRange;
uniform mat4 uFovMatrix0;
uniform mat4 uFovMatrix1;
// highp is essential: GLSL ES defaults fragment samplers to lowp, and at
// least Mesa llvmpipe then returns depth reads at half-float precision
// (~11 bits), which after linearization is a 0.5-unit sawtooth at 15 units
// out -- far more than any bias can hide. (Diagnosed on this prototype;
// see docs/fov-shadow-map.md.)
uniform highp sampler2D uFovMap0;
uniform highp sampler2D uFovMap1;
uniform float uNear;
uniform float uFar;
uniform float uTexelPerDepth;  // World size of one map texel per unit of map depth.
uniform float uProbeHeight;
uniform vec4 uColor;
out vec4 FragColor;

// Perspective depth-buffer value -> distance along the map's view axis.
float LinearDepth(float bufferDepth) {
  float ndcZ = bufferDepth * 2.0 - 1.0;
  return 2.0 * uNear * uFar / (uFar + uNear - ndcZ * (uFar - uNear));
}

// 1 = sightline clear, 0 = occluded, in between = PCF edge; -1 = the point
// projects outside this map.
float SampleMap(highp sampler2D map, mat4 matrix, vec3 p, float slope) {
  vec4 clip = matrix * vec4(p, 1.0);
  if (clip.w <= 0.0) return -1.0;
  vec3 ndc = clip.xyz / clip.w;
  if (any(greaterThan(abs(ndc), vec3(1.0)))) return -1.0;
  float depth = LinearDepth(ndc.z * 0.5 + 0.5);
  // Modest slope-scaled bias in linear depth. Grazing angles are handled
  // mainly by the normal offset applied in main() (it pushes the sample
  // along the surface by a couple of texels, however oblique the view);
  // this only has to absorb the residual depth variation across the 3x3
  // PCF kernel on moderately tilted surfaces, so the slope term is capped.
  float texelWorld = uTexelPerDepth * depth;
  float bias = texelWorld * (1.0 + min(slope, 4.0)) + 0.02;
  vec2 uv = ndc.xy * 0.5 + 0.5;
  vec2 texel = 1.0 / vec2(textureSize(map, 0));
  float lit = 0.0;
  for (int x = -1; x <= 1; ++x) {
    for (int y = -1; y <= 1; ++y) {
      float stored = LinearDepth(texture(map, uv + vec2(x, y) * texel).r);
      lit += (depth - bias) <= stored ? 1.0 : 0.0;
    }
  }
  return lit / 9.0;
}

void main() {
  vec3 n = normalize(vWorldNormal);
  vec3 toEye = uEye - vWorldPos;
  float dist = length(toEye);
  if (dist > uRange) discard;
  vec3 dir = toEye / dist;
  // A face turned away from the eye is the back of something; the eye sees
  // its front, which is drawn (and tested) separately.
  float ndotl = dot(n, dir);
  if (ndotl <= 0.0) discard;
  // Analytic azimuth test: the fragment's bearing from the eye must be
  // within the half-FOV of the unit's facing.
  vec2 bearing = vec2(-dir.x, -dir.z);
  float bearingLen = length(bearing);
  if (bearingLen > 1e-5 && dot(bearing / bearingLen, uFacing.xz) < uCosHalfFov) discard;
  // Normal-offset sampling: lift the sample point off the surface along its
  // normal by ~2 texels' worth, scaled by sin(theta) so a surface seen
  // head-on isn't moved at all while a grazing one is moved enough that
  // its projection lands a couple of texels further along itself, clear of
  // its own depth. Then (ground semantics) probe at the target height
  // above upward-facing surfaces.
  float sinTheta = sqrt(max(1.0 - ndotl * ndotl, 0.0));
  float slope = sinTheta / max(ndotl, 1e-3);
  vec3 p = vWorldPos + n * (2.0 * uTexelPerDepth * dist * sinTheta);
  if (n.y > 0.7) p.y += uProbeHeight;
  float lit = SampleMap(uFovMap0, uFovMatrix0, p, slope);
  if (lit < 0.0) lit = SampleMap(uFovMap1, uFovMatrix1, p, slope);
  // Binary edge: a partially lit fragment would claim the stencil with a
  // faint alpha and block a teammate with a clearer view of the same pixel.
  if (lit < 0.5) discard;
  FragColor = uColor;
}
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
template <typename Mesh>
void DrawLitModel(const Shader& shader, const Mesh& mesh, const glm::mat4& viewProj,
                  const glm::mat4& lightSpaceMatrix, const glm::mat4& model,
                  const glm::vec4& color) {
  shader.SetMat4("uModel", model);
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetMat4("uLightSpaceMatrix", lightSpaceMatrix);
  shader.SetVec4("uColor", color);
  mesh.Draw();
}

void DrawBoxLitModel(const Shader& shader, const CubeMesh& cube, const glm::mat4& viewProj,
                     const glm::mat4& lightSpaceMatrix, const glm::mat4& model,
                     const glm::vec4& color) {
  DrawLitModel(shader, cube, viewProj, lightSpaceMatrix, model, color);
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

glm::mat4 BoxModel(const glm::vec3& minCorner, const glm::vec3& size) {
  return glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
}

using LitVertices = std::vector<LitTriangleMesh::Vertex>;

void AppendTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal,
                    LitVertices* vertices, std::vector<GLuint>* indices) {
  const GLuint base = static_cast<GLuint>(vertices->size());
  vertices->push_back({a, normal});
  vertices->push_back({b, normal});
  vertices->push_back({c, normal});
  indices->insert(indices->end(), {base, base + 1, base + 2});
}

void AppendPolygonPrism(const tactics::Obstacle& obstacle, LitVertices* vertices,
                        std::vector<GLuint>* indices) {
  const std::vector<glm::vec2> polygon = tactics::ObstacleFootprint(obstacle);
  auto triangle = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal) {
    AppendTriangle(a, b, c, normal, vertices, indices);
  };
  const float lo = obstacle.bounds.min.y, hi = obstacle.bounds.max.y;
  for (size_t i = 1; i + 1 < polygon.size(); ++i) {
    triangle({polygon[0].x, hi, polygon[0].y}, {polygon[i].x, hi, polygon[i].y},
             {polygon[i + 1].x, hi, polygon[i + 1].y}, {0.0f, 1.0f, 0.0f});
    triangle({polygon[0].x, lo, polygon[0].y}, {polygon[i + 1].x, lo, polygon[i + 1].y},
             {polygon[i].x, lo, polygon[i].y}, {0.0f, -1.0f, 0.0f});
  }
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i], b = polygon[(i + 1) % polygon.size()];
    const glm::vec2 edge = glm::normalize(b - a);
    const glm::vec3 normal(edge.y, 0.0f, -edge.x);
    const glm::vec3 a0(a.x, lo, a.y), a1(a.x, hi, a.y);
    const glm::vec3 b0(b.x, lo, b.y), b1(b.x, hi, b.y);
    triangle(a0, b0, b1, normal);
    triangle(a0, b1, a1, normal);
  }
}

void BuildPolygonPrism(const tactics::Obstacle& obstacle, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  AppendPolygonPrism(obstacle, &vertices, &indices);
  mesh->SetMesh(vertices, indices);
}

// `hiddenEdges[i]` (optional) skips the side wall under polygon edge i, for
// edges butted against a neighbouring slab: the wall would only show as a
// seam, most visibly through the faded see-through pass.
void AppendSurfacePatch(const std::vector<glm::vec3>& polygon, float thickness,
                        LitVertices* vertices, std::vector<GLuint>* indices,
                        const std::vector<char>* hiddenEdges = nullptr) {
  if (polygon.size() < 3) return;
  glm::vec3 topNormal = glm::normalize(glm::cross(polygon[1] - polygon[0], polygon[2] - polygon[0]));
  if (topNormal.y < 0.0f) topNormal = -topNormal;
  auto triangle = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal) {
    AppendTriangle(a, b, c, normal, vertices, indices);
  };
  for (size_t i = 1; i + 1 < polygon.size(); ++i) {
    triangle(polygon[0], polygon[i], polygon[i + 1], topNormal);
    if (thickness > 0.0f) {
      triangle(polygon[0] - glm::vec3(0, thickness, 0),
               polygon[i + 1] - glm::vec3(0, thickness, 0),
               polygon[i] - glm::vec3(0, thickness, 0), -topNormal);
    }
  }
  if (thickness > 0.0f) {
    for (size_t i = 0; i < polygon.size(); ++i) {
      if (hiddenEdges && (*hiddenEdges)[i]) continue;
      const glm::vec3 a = polygon[i], b = polygon[(i + 1) % polygon.size()];
      glm::vec3 normal = glm::cross(b - a, glm::vec3(0, -1, 0));
      if (glm::length(normal) > 1e-6f) normal = glm::normalize(normal);
      triangle(a, b - glm::vec3(0, thickness, 0), b, normal);
      triangle(a, a - glm::vec3(0, thickness, 0), b - glm::vec3(0, thickness, 0), normal);
    }
  }
}

void BuildSurfacePatch(const std::vector<glm::vec3>& polygon, float thickness,
                       LitTriangleMesh* mesh, const std::vector<char>* hiddenEdges = nullptr) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  AppendSurfacePatch(polygon, thickness, &vertices, &indices, hiddenEdges);
  mesh->SetMesh(vertices, indices);
}

// Edges of `surface` shared (same endpoints, either direction) with one of its
// chain neighbours, i.e. the joints between consecutive deck/ramp slabs.
std::vector<char> SharedEdges(const std::vector<tactics::WalkSurface>& surfaces,
                              const tactics::WalkSurface& surface) {
  const std::vector<glm::vec3>& v = surface.vertices;
  std::vector<char> shared(v.size(), 0);
  auto same = [](const glm::vec3& p, const glm::vec3& q) { return glm::distance(p, q) < 1e-3f; };
  for (int neighbor : surface.neighbors) {
    const std::vector<glm::vec3>& w = surfaces[neighbor].vertices;
    for (size_t i = 0; i < v.size(); ++i) {
      const glm::vec3& a = v[i];
      const glm::vec3& b = v[(i + 1) % v.size()];
      for (size_t j = 0; j < w.size(); ++j) {
        const glm::vec3& c = w[j];
        const glm::vec3& d = w[(j + 1) % w.size()];
        if ((same(a, c) && same(b, d)) || (same(a, d) && same(b, c))) shared[i] = 1;
      }
    }
  }
  return shared;
}

// Axis-aligned box with outward normals (same faces as CubeMesh, in world
// space), for the legacy sidewalk slabs in the static scene mesh.
void AppendBox(const AABB& box, LitVertices* vertices, std::vector<GLuint>* indices) {
  const glm::vec3 lo = box.min, hi = box.max;
  auto quad = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 n) {
    AppendTriangle(a, b, c, n, vertices, indices);
    AppendTriangle(a, c, d, n, vertices, indices);
  };
  quad({lo.x, hi.y, lo.z}, {lo.x, hi.y, hi.z}, {hi.x, hi.y, hi.z}, {hi.x, hi.y, lo.z}, {0, 1, 0});
  quad({lo.x, lo.y, lo.z}, {hi.x, lo.y, lo.z}, {hi.x, lo.y, hi.z}, {lo.x, lo.y, hi.z}, {0, -1, 0});
  quad({lo.x, lo.y, lo.z}, {lo.x, hi.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, lo.y, lo.z}, {0, 0, -1});
  quad({lo.x, lo.y, hi.z}, {hi.x, lo.y, hi.z}, {hi.x, hi.y, hi.z}, {lo.x, hi.y, hi.z}, {0, 0, 1});
  quad({lo.x, lo.y, lo.z}, {lo.x, lo.y, hi.z}, {lo.x, hi.y, hi.z}, {lo.x, hi.y, lo.z}, {-1, 0, 0});
  quad({hi.x, lo.y, lo.z}, {hi.x, hi.y, lo.z}, {hi.x, hi.y, hi.z}, {hi.x, lo.y, hi.z}, {1, 0, 0});
}

// Everything static the shadow-map FOV prototype (issue #110) casts from
// and receives on, apart from the ground itself: obstacles (walls + roofs),
// decks/ramps as 0.45-thick slabs (matching the gameplay LOS slab in
// LineOfSightClear and the lit pass), sidewalk slabs and patches, and
// roads. Built once per scene so each extra per-unit pass is one draw call.
void BuildFovSceneMesh(const tactics::Scene& scene, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  for (const tactics::Obstacle& obstacle : scene.obstacles) {
    AppendPolygonPrism(obstacle, &vertices, &indices);
  }
  for (const tactics::WalkSurface& surface : scene.walkSurfaces) {
    AppendSurfacePatch(surface.vertices, 0.45f, &vertices, &indices);
  }
  for (const AABB& slab : scene.sidewalks) AppendBox(slab, &vertices, &indices);
  for (const tactics::RoadSurface& sidewalk : scene.sidewalkSurfaces) {
    AppendSurfacePatch(sidewalk.vertices, 0.10f, &vertices, &indices);
  }
  for (const tactics::RoadSurface& road : scene.roads) {
    AppendSurfacePatch(road.vertices, 0.0f, &vertices, &indices);
  }
  mesh->SetMesh(vertices, indices);
}

// Content hash of the static geometry BuildFovSceneMesh consumes, so the
// mesh is rebuilt only when the scene actually changes (successive scenes
// can reuse the same GameLogic address, so an address key won't do).
unsigned long long FovSceneFingerprint(const tactics::Scene& scene) {
  unsigned long long h = 1469598103934665603ULL;
  auto mix = [&](const void* data, size_t bytes) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
      h ^= p[i];
      h *= 1099511628211ULL;
    }
  };
  auto mixVec3s = [&](const std::vector<glm::vec3>& v) {
    mix(v.data(), v.size() * sizeof(glm::vec3));
  };
  for (const tactics::Obstacle& o : scene.obstacles) {
    mix(&o.bounds, sizeof(o.bounds));
    mix(o.footprint.data(), o.footprint.size() * sizeof(glm::vec2));
  }
  for (const tactics::WalkSurface& s : scene.walkSurfaces) mixVec3s(s.vertices);
  mix(scene.sidewalks.data(), scene.sidewalks.size() * sizeof(AABB));
  for (const tactics::RoadSurface& s : scene.sidewalkSurfaces) mixVec3s(s.vertices);
  for (const tactics::RoadSurface& s : scene.roads) mixVec3s(s.vertices);
  return h;
}

bool PatchContainsXZ(const tactics::RoadSurface& patch, float x, float z) {
  if (patch.vertices.size() < 3) return false;
  float area = 0.0f;
  for (size_t i = 0; i < patch.vertices.size(); ++i) {
    const glm::vec3& a = patch.vertices[i];
    const glm::vec3& b = patch.vertices[(i + 1) % patch.vertices.size()];
    area += a.x * b.z - a.z * b.x;
  }
  const float sign = area >= 0.0f ? 1.0f : -1.0f;
  for (size_t i = 0; i < patch.vertices.size(); ++i) {
    const glm::vec3& a = patch.vertices[i];
    const glm::vec3& b = patch.vertices[(i + 1) % patch.vertices.size()];
    if (sign * ((b.x - a.x) * (z - a.z) - (b.z - a.z) * (x - a.x)) < -1e-4f) {
      return false;
    }
  }
  return true;
}

void DrawUnit(const Shader& shader, const CubeMesh& cube, const SphereMesh& sphere,
              const glm::mat4& viewProj, const glm::mat4& lightSpaceMatrix, const Unit& unit) {
  for (const FigurePart& part : BuildFigure(unit)) {
    if (part.primitive == FigurePrimitive::Rounded) {
      DrawLitModel(shader, sphere, viewProj, lightSpaceMatrix, part.model, part.color);
    } else {
      DrawLitModel(shader, cube, viewProj, lightSpaceMatrix, part.model, part.color);
    }
  }
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

// Three great circles form a compact wire cage for an ellipsoid. This keeps
// plan ghosts and remembered sightings faithful to the rounded live figure
// without needing polygon-line mode (which GLES/WebGL2 do not expose).
void DrawWireRounded(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                     const glm::mat4& model, const glm::vec4& color) {
  constexpr int kSegments = 20;
  constexpr float kTwoPi = 6.28318530717958647692f;
  shader.SetMat4("uMVP", viewProj);
  shader.SetVec4("uColor", color);
  for (int plane = 0; plane < 3; ++plane) {
    std::vector<glm::vec3> points;
    points.reserve(kSegments + 1);
    for (int i = 0; i <= kSegments; ++i) {
      const float angle = kTwoPi * static_cast<float>(i) / kSegments;
      const float c = std::cos(angle);
      const float s = std::sin(angle);
      const glm::vec3 local = plane == 0   ? glm::vec3(c, s, 0.0f)
                              : plane == 1 ? glm::vec3(0.0f, c, s)
                                           : glm::vec3(c, 0.0f, s);
      points.push_back(glm::vec3(model * glm::vec4(local, 1.0f)));
    }
    lines.SetPoints(points);
    lines.Draw();
  }
}

void DrawFigureWirePart(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                        const FigurePart& part, const glm::vec4& color) {
  if (part.primitive == FigurePrimitive::Rounded) {
    DrawWireRounded(shader, lines, viewProj, part.model, color);
  } else {
    DrawWireBox(shader, lines, viewProj, part.model, color);
  }
}

void DrawUnitWireframe(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                       const Unit& unit) {
  // Common highlight green for both teams.
  const glm::vec4 color(0.3f, 0.9f, 0.4f, 1.0f);
  for (const FigurePart& part : BuildFigureWireframe(unit)) {
    DrawFigureWirePart(shader, lines, viewProj, part, color);
  }
}

// Faded, team-colored wireframe of a remembered sighting plus a floor arrow
// along its movement direction (if it was moving).
void DrawSighting(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                  const Unit& sighted, const GameLogic::EnemySighting& s,
                  const std::vector<AABB>& sidewalks, float alpha,
                  const tactics::HeightField& terrain) {
  const glm::vec4 base = sighted.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                    : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  const glm::vec4 color(base.r, base.g, base.b, alpha);
  // Sidewalk slabs are raised but cosmetic (unit y stays 0), so lift the ghost
  // and arrow onto whatever slab the remembered position stands over, else
  // they render buried under it.
  glm::vec3 position = s.position;
  for (const AABB& slab : sidewalks) {
    if (position.x >= slab.min.x && position.x <= slab.max.x && position.z >= slab.min.z &&
        position.z <= slab.max.z) {
      position.y = std::max(position.y, slab.max.y);
    }
  }
  Unit ghost = sighted;
  ghost.position = position;
  ghost.facingYaw = s.facingYaw;
  ghost.walkPhase = s.walkPhase;
  ghost.walkBlend = s.walkBlend;
  ghost.idleElapsed = s.idleElapsed;
  ghost.knockdownElapsed = -1.0f;
  if (glm::length(s.moveDirection) > 0.0f) {
    const glm::vec3 d = s.moveDirection;
    const glm::vec3 side(-d.z, 0.0f, d.x);
    // Centre the arrow's overall length on the ghost.
    const float kLength = 1.2f;
    const glm::vec3 tail = position - d * (kLength * 0.5f);
    const glm::vec3 tip = tail + d * kLength;
    // Closed outline of a fat arrow (shaft + head), traced as a line strip.
    const float kShaftHalfWidth = 0.08f;
    const float kHeadHalfWidth = 0.22f;
    const float kHeadLength = 0.4f;
    const glm::vec3 headBase = tip - d * kHeadLength;
    // Each arrow vertex sits a hair above the terrain under it (a no-op on
    // flat maps), so the outline drapes over slopes instead of burying into
    // them.
    const auto onGround = [&](const glm::vec3& p) {
      if (terrain.Empty()) return p + glm::vec3(0.0f, 0.02f, 0.0f);
      return glm::vec3(p.x, terrain.HeightAt(p.x, p.z) + 0.02f, p.z);
    };
    const std::vector<glm::vec3> arrow = {
        onGround(tail + side * kShaftHalfWidth),
        onGround(headBase + side * kShaftHalfWidth),
        onGround(headBase + side * kHeadHalfWidth), onGround(tip),
        onGround(headBase - side * kHeadHalfWidth),
        onGround(headBase - side * kShaftHalfWidth),
        onGround(tail - side * kShaftHalfWidth),
        onGround(tail + side * kShaftHalfWidth)};
    lines.SetPoints(arrow);
    shader.SetMat4("uMVP", viewProj);
    shader.SetVec4("uColor", color);
    lines.Draw();
  }
  for (const FigurePart& part : BuildFigureWireframe(ghost)) {
    DrawFigureWirePart(shader, lines, viewProj, part, color);
  }
}

void DrawUnitDepth(const Shader& shader, const CubeMesh& cube, const SphereMesh& sphere,
                   const glm::mat4& lightSpaceMatrix, const Unit& unit) {
  for (const FigurePart& part : BuildFigure(unit)) {
    shader.SetMat4("uLightMVP", lightSpaceMatrix * part.model);
    if (part.primitive == FigurePrimitive::Rounded) {
      sphere.Draw();
    } else {
      cube.Draw();
    }
  }
}

// Flat ring (annulus) in the XZ plane, unit-sized and centred on the origin.
std::vector<glm::vec3> BuildRingPoints() {
  constexpr int kSegments = 32;
  constexpr float kOuter = 0.5f;
  constexpr float kInner = 0.35f;
  constexpr float kTwoPi = 6.28318530717958647692f;
  std::vector<glm::vec3> pts;
  pts.reserve(kSegments * 6);
  for (int i = 0; i < kSegments; ++i) {
    const float a0 = kTwoPi * static_cast<float>(i) / kSegments;
    const float a1 = kTwoPi * static_cast<float>(i + 1) / kSegments;
    const glm::vec3 o0(kOuter * std::cos(a0), 0.0f, kOuter * std::sin(a0));
    const glm::vec3 o1(kOuter * std::cos(a1), 0.0f, kOuter * std::sin(a1));
    const glm::vec3 i0(kInner * std::cos(a0), 0.0f, kInner * std::sin(a0));
    const glm::vec3 i1(kInner * std::cos(a1), 0.0f, kInner * std::sin(a1));
    // Counter-clockwise seen from above (+Y).
    pts.insert(pts.end(), {o0, o1, i0, i0, o1, i1});
  }
  return pts;
}

// `normal` is the unit surface normal the ring lies flat against (a ring on a
// ramp or hillside tilts with it instead of poking into the slope).
void DrawHighlight(const Shader& shader, const TriangleMesh& ring, const glm::mat4& viewProj,
                   const glm::vec3& position, const glm::vec4& color,
                   const glm::vec3& normal = glm::vec3(0.0f, 1.0f, 0.0f)) {
  glm::mat4 model = glm::translate(glm::mat4(1.0f), position + normal * 0.02f);
  const glm::vec3 axis = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), normal);
  if (glm::length(axis) > 1e-5f) {
    model = glm::rotate(model, std::acos(std::clamp(normal.y, -1.0f, 1.0f)), glm::normalize(axis));
  }
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetVec4("uColor", color);
  ring.Draw();
}

// 3x5 digit glyphs, one row per entry, MSB = leftmost column.
constexpr unsigned char kDigitGlyphs[10][5] = {
    {7, 5, 5, 5, 7}, {2, 6, 2, 2, 7}, {7, 1, 7, 4, 7}, {7, 1, 7, 1, 7}, {5, 5, 7, 1, 1},
    {7, 4, 7, 1, 7}, {7, 4, 7, 5, 7}, {7, 1, 1, 1, 1}, {7, 5, 7, 5, 7}, {7, 5, 7, 1, 7}};

// Snaps a ground-plane direction to the nearest world axis.
glm::vec3 SnapToAxis(const glm::vec3& v) {
  if (std::abs(v.x) >= std::abs(v.z)) return glm::vec3(v.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f);
  return glm::vec3(0.0f, 0.0f, v.z < 0.0f ? -1.0f : 1.0f);
}

// The thick-rim ring with the number `n` (1-99) drawn inside it as flat
// yellow digits. Digits are built from axis-aligned cells and oriented to
// read upright for `view`'s camera (snapped to the nearest world axis).
void DrawNumberedHighlight(const Shader& shader, const TriangleMesh& ring, const CubeMesh& cube,
                           const glm::mat4& viewProj, const glm::mat4& view,
                           const glm::vec3& position, const glm::vec4& color, int n,
                           const glm::vec3& normal) {
  DrawHighlight(shader, ring, viewProj, position, color, normal);
  n = std::clamp(n, 0, 99);
  const int digitCount = n >= 10 ? 2 : 1;
  const float cell = 0.07f;  // Two digits (7 cells wide) still fit inside the rim.
  const float textWidth = (digitCount * 3 + (digitCount - 1)) * cell;
  const glm::vec3 right = SnapToAxis(glm::vec3(view[0][0], 0.0f, view[2][0]));
  const glm::vec3 up = SnapToAxis(glm::vec3(-view[0][2], 0.0f, -view[2][2]));
  for (int d = 0; d < digitCount; ++d) {
    const int digit = digitCount == 2 ? (d == 0 ? n / 10 : n % 10) : n;
    for (int row = 0; row < 5; ++row) {
      for (int col = 0; col < 3; ++col) {
        if (!(kDigitGlyphs[digit][row] & (4 >> col))) continue;
        const float u0 = -textWidth / 2 + (d * 4 + col) * cell;
        const float v0 = 2.5f * cell - (row + 1) * cell;
        const glm::vec3 a = position + right * u0 + up * v0;
        const glm::vec3 b = position + right * (u0 + cell) + up * (v0 + cell);
        const glm::vec3 lo = glm::min(a, b);
        const glm::vec3 hi = glm::max(a, b);
        // Lift each cell onto the ring's plane so digits don't sink into slopes.
        const glm::vec3 mid = 0.5f * (lo + hi) - position;
        const float planeY = position.y - (normal.x * mid.x + normal.z * mid.z) / normal.y;
        DrawBox(shader, cube, viewProj, glm::vec3(lo.x, planeY + 0.01f, lo.z),
                glm::vec3(hi.x - lo.x, 0.04f, hi.z - lo.z), color);
      }
    }
  }
}

// Triangulated terrain heightfield with central-difference normals, plus a
// border skirt dropping below the lowest sample so the map edge reads as a
// solid block instead of a paper-thin sheet.
void BuildTerrainMesh(const tactics::HeightField& hf, LitTriangleMesh* mesh) {
  const int nx = hf.nx, nz = hf.nz;
  std::vector<LitTriangleMesh::Vertex> vertices;
  std::vector<GLuint> indices;
  vertices.reserve(static_cast<size_t>(nx) * nz);
  float skirtBottom = 0.0f;
  for (int iz = 0; iz < nz; ++iz) {
    for (int ix = 0; ix < nx; ++ix) {
      const float x = hf.minX + ix * hf.step;
      const float z = hf.minZ + iz * hf.step;
      const float y = hf.At(ix, iz);
      skirtBottom = std::min(skirtBottom, y);
      const float dx = (hf.At(ix + 1, iz) - hf.At(ix - 1, iz)) / (2.0f * hf.step);
      const float dz = (hf.At(ix, iz + 1) - hf.At(ix, iz - 1)) / (2.0f * hf.step);
      vertices.push_back({glm::vec3(x, y, z), glm::normalize(glm::vec3(-dx, 1.0f, -dz))});
    }
  }
  indices.reserve(static_cast<size_t>(nx - 1) * (nz - 1) * 6);
  for (int iz = 0; iz + 1 < nz; ++iz) {
    for (int ix = 0; ix + 1 < nx; ++ix) {
      const GLuint a = static_cast<GLuint>(iz * nx + ix);
      const GLuint b = a + 1;
      const GLuint c = a + nx;
      const GLuint d = c + 1;
      indices.insert(indices.end(), {a, b, d, a, d, c});
    }
  }
  skirtBottom -= 0.5f;
  // Skirt: one quad per border edge, with an outward horizontal normal.
  auto skirtQuad = [&](int ix0, int iz0, int ix1, int iz1, const glm::vec3& normal) {
    const GLuint base = static_cast<GLuint>(vertices.size());
    const glm::vec3 a = vertices[iz0 * nx + ix0].pos;
    const glm::vec3 b = vertices[iz1 * nx + ix1].pos;
    vertices.push_back({a, normal});
    vertices.push_back({b, normal});
    vertices.push_back({glm::vec3(b.x, skirtBottom, b.z), normal});
    vertices.push_back({glm::vec3(a.x, skirtBottom, a.z), normal});
    indices.insert(indices.end(), {base, base + 1, base + 2, base, base + 2, base + 3});
  };
  for (int ix = 0; ix + 1 < nx; ++ix) {
    skirtQuad(ix, 0, ix + 1, 0, glm::vec3(0.0f, 0.0f, -1.0f));
    skirtQuad(ix, nz - 1, ix + 1, nz - 1, glm::vec3(0.0f, 0.0f, 1.0f));
  }
  for (int iz = 0; iz + 1 < nz; ++iz) {
    skirtQuad(0, iz, 0, iz + 1, glm::vec3(-1.0f, 0.0f, 0.0f));
    skirtQuad(nx - 1, iz, nx - 1, iz + 1, glm::vec3(1.0f, 0.0f, 0.0f));
  }
  mesh->SetMesh(vertices, indices);
}

// Debug overlay of the navmesh's walkable-cell boundaries: each cell's
// rectangle drawn as line segments lifted a hair above its walkable surface
// -- terrain-following for ground cells (edges subdivided so they hug the
// slope), flat at the obstacle top for climb-top cells.
void DrawNavMeshDebug(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                      const tactics::NavMesh& navmesh, const tactics::HeightField& terrain) {
  // Lines can't use polygon offset in GLES, so they need a real lift above
  // the drawn terrain. The drawn mesh triangulates each heightfield cell,
  // which on a steep diagonal slope can bow above the bilinear HeightAt
  // sampled here, so the lift must clear that bow (plus depth precision at
  // distance), not just a z-fighting epsilon.
  constexpr float kLift = 0.2f;
  constexpr float kEdgeStep = 0.75f;  // Terrain-sampling interval along an edge.
  std::vector<glm::vec3> groundSegs, climbSegs;
  for (const tactics::NavCell& cell : navmesh.Cells()) {
    const glm::vec2 corners[4] = {{cell.xMin, cell.zMin},
                                  {cell.xMax, cell.zMin},
                                  {cell.xMax, cell.zMax},
                                  {cell.xMin, cell.zMax}};
    std::vector<glm::vec3>& out = cell.climbTop ? climbSegs : groundSegs;
    for (int e = 0; e < 4; ++e) {
      const glm::vec2 a = corners[e];
      const glm::vec2 b = corners[(e + 1) % 4];
      const int pieces =
          cell.climbTop || terrain.Empty()
              ? 1
              : std::max(1, static_cast<int>(std::ceil(glm::distance(a, b) / kEdgeStep)));
      for (int k = 0; k < pieces; ++k) {
        const glm::vec2 p = a + (b - a) * (static_cast<float>(k) / pieces);
        const glm::vec2 q = a + (b - a) * (static_cast<float>(k + 1) / pieces);
        const float py = cell.climbTop ? cell.elevation : terrain.HeightAt(p.x, p.y);
        const float qy = cell.climbTop ? cell.elevation : terrain.HeightAt(q.x, q.y);
        out.emplace_back(p.x, py + kLift, p.y);
        out.emplace_back(q.x, qy + kLift, q.y);
      }
    }
  }
  shader.SetMat4("uMVP", viewProj);
  shader.SetVec4("uColor", glm::vec4(0.25f, 0.9f, 0.95f, 1.0f));
  lines.SetPoints(groundSegs);
  lines.DrawSegments();
  if (!climbSegs.empty()) {
    shader.SetVec4("uColor", glm::vec4(1.0f, 0.6f, 0.1f, 1.0f));
    lines.SetPoints(climbSegs);
    lines.DrawSegments();
  }
  std::vector<glm::vec3> surfaceSegs;
  for (const tactics::WalkSurface& surface : navmesh.WalkSurfaces()) {
    for (size_t i = 0; i < surface.vertices.size(); ++i) {
      surfaceSegs.push_back(surface.vertices[i] + glm::vec3(0, 0.03f, 0));
      surfaceSegs.push_back(surface.vertices[(i + 1) % surface.vertices.size()] +
                            glm::vec3(0, 0.03f, 0));
    }
  }
  if (!surfaceSegs.empty()) {
    shader.SetVec4("uColor", glm::vec4(0.95f, 0.25f, 0.9f, 1.0f));
    lines.SetPoints(surfaceSegs);
    lines.DrawSegments();
  }
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

bool FootprintSpan(const glm::vec3& eye, const glm::vec2& dir,
                   const tactics::Obstacle& obstacle, float* outEnter, float* outExit) {
  const std::vector<glm::vec2> polygon = tactics::ObstacleFootprint(obstacle);
  float enter = 0.0f;
  float exit = std::numeric_limits<float>::max();
  const float orientation = tactics::PolygonSignedArea(polygon) >= 0.0f ? 1.0f : -1.0f;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i];
    const glm::vec2 edge = polygon[(i + 1) % polygon.size()] - a;
    const glm::vec2 rel(eye.x - a.x, eye.z - a.y);
    const float value = orientation * (edge.x * rel.y - edge.y * rel.x);
    const float rate = orientation * (edge.x * dir.y - edge.y * dir.x);
    if (std::fabs(rate) < 1e-8f) {
      if (value < 0.0f) return false;
      continue;
    }
    const float t = -value / rate;
    if (rate > 0.0f) enter = std::max(enter, t);
    else exit = std::min(exit, t);
    if (exit < enter) return false;
  }
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
bool GroundShadow(const glm::vec3& eye, const glm::vec2& dir,
                  const tactics::Obstacle& obstacle, float range,
                  GroundSpan* outShadow) {
  float enter = 0.0f;
  float exit = 0.0f;
  if (!FootprintSpan(eye, dir, obstacle, &enter, &exit)) return false;
  const AABB& box = obstacle.bounds;
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

// A walk surface as a thin occluder: its XZ footprint plus the slab beneath
// the top plane. Thickness matches the gameplay LOS check (LineOfSightClear).
struct DeckOccluder {
  const tactics::WalkSurface* surface = nullptr;
  tactics::Obstacle footprint;
  // The eye is above the deck's plane, so its top face is the one in view.
  bool topFacesEye = false;
};
constexpr float kDeckThickness = 0.45f;

// The ground shadow a deck casts along one sight ray and, when the eye looks
// down on it, the stretch of its top face the ray lands on. A sightline to
// ground distance T is blocked if it crosses the slab somewhere in the
// footprint span; the blocking distance T = s * eye.y / (eye.y - h) is
// monotonic in s, so the extremes sit at the span ends. Slab faces at or
// above eye level are never reached by (descending) ground sightlines.
void DeckSpans(const glm::vec3& eye, const glm::vec2& dir, const DeckOccluder& deck,
               float range, std::vector<GroundSpan>* shadows, GroundSpan* outTop,
               bool* hasTop) {
  *hasTop = false;
  float enter = 0.0f;
  float exit = 0.0f;
  if (!FootprintSpan(eye, dir, deck.footprint, &enter, &exit)) return;
  if (enter >= range) return;
  if (deck.topFacesEye) {
    *outTop = GroundSpan{enter, std::min(exit, range)};
    *hasTop = outTop->begin < outTop->end;
  }
  const auto landing = [&](float s, float h) {
    h = std::max(h, 0.0f);
    return h >= eye.y - 1e-4f ? std::numeric_limits<float>::infinity()
                              : s * eye.y / (eye.y - h);
  };
  float begin = std::numeric_limits<float>::infinity();
  float end = 0.0f;
  for (const float s : {enter, exit}) {
    const float top = tactics::SurfaceHeightAt(*deck.surface, eye.x + dir.x * s,
                                               eye.z + dir.y * s);
    begin = std::min(begin, landing(s, top - kDeckThickness));
    end = std::max(end, landing(s, top));
  }
  end = std::min(end, range);
  if (begin < end) shadows->push_back(GroundSpan{begin, end});
}

// The visible stretches of ground along one sight ray within [0, range]:
// the complement of the union of every obstacle's and deck's ground shadow.
// Decks also report, via `deckTops`, the visible stretch of their top face.
struct DeckTop {
  int deck = 0;
  GroundSpan span;
};
std::vector<GroundSpan> VisibleGroundSpans(const glm::vec3& eye, const glm::vec2& dir,
                                           const std::vector<tactics::Obstacle>& obstacles,
                                           const std::vector<DeckOccluder>& decks, float range,
                                           std::vector<DeckTop>* deckTops) {
  std::vector<GroundSpan> shadows;
  for (const auto& obstacle : obstacles) {
    GroundSpan shadow;
    if (GroundShadow(eye, dir, obstacle, range, &shadow)) shadows.push_back(shadow);
  }
  for (size_t i = 0; i < decks.size(); ++i) {
    GroundSpan top;
    bool hasTop = false;
    DeckSpans(eye, dir, decks[i], range, &shadows, &top, &hasTop);
    if (hasTop) deckTops->push_back(DeckTop{static_cast<int>(i), top});
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

// Clip the terrain's actual triangles to an overlay footprint, retaining
// their interpolated heights. Sampling only the footprint's vertices (even
// with radial subdivisions) cuts across ridges, and bilinear HeightAt does
// not match the rendered surface within a non-planar heightfield cell.
void AppendTerrainOverlay(const tactics::HeightField& terrain,
                          const std::array<glm::vec3, 3>& footprint,
                          std::vector<glm::vec3>* points) {
  constexpr float kTerrainLift = 0.08f;
  const auto crossXZ = [](const glm::vec3& a, const glm::vec3& b) {
    return a.x * b.z - a.z * b.x;
  };
  const float area = crossXZ(footprint[1] - footprint[0], footprint[2] - footprint[0]);
  if (std::abs(area) < 1e-8f) return;  // The cone's tip can make a degenerate triangle.
  const float winding = area > 0.0f ? 1.0f : -1.0f;
  const glm::vec3 lo = glm::min(footprint[0], glm::min(footprint[1], footprint[2]));
  const glm::vec3 hi = glm::max(footprint[0], glm::max(footprint[1], footprint[2]));
  const auto cellIndex = [&](float value, float origin, int count) {
    return std::clamp(static_cast<int>(std::floor((value - origin) / terrain.step)),
                      0, count - 2);
  };
  const int z0 = cellIndex(lo.z, terrain.minZ, terrain.nz);
  const int z1 = cellIndex(hi.z, terrain.minZ, terrain.nz);
  const auto vertex = [&](int ix, int iz) {
    return glm::vec3(terrain.minX + ix * terrain.step, terrain.At(ix, iz) + kTerrainLift,
                     terrain.minZ + iz * terrain.step);
  };
  for (int iz = z0; iz <= z1; ++iz) {
    // A narrow cone slice can have a huge bounding box. Restrict each grid
    // row to the slice's X interval so we only clip nearby terrain cells.
    const float rowMinZ = terrain.minZ + iz * terrain.step;
    const float rowMaxZ = rowMinZ + terrain.step;
    float minX = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    const auto includeX = [&](float x) {
      minX = std::min(minX, x);
      maxX = std::max(maxX, x);
    };
    for (int edge = 0; edge < 3; ++edge) {
      const glm::vec3& p = footprint[edge];
      const glm::vec3& q = footprint[(edge + 1) % 3];
      if (p.z >= rowMinZ && p.z <= rowMaxZ) includeX(p.x);
      if (p.z == q.z) continue;
      for (float z : {rowMinZ, rowMaxZ}) {
        if (z >= std::min(p.z, q.z) && z <= std::max(p.z, q.z)) {
          includeX(glm::mix(p.x, q.x, (z - p.z) / (q.z - p.z)));
        }
      }
    }
    if (minX > maxX) continue;
    const int x0 = cellIndex(minX, terrain.minX, terrain.nx);
    const int x1 = cellIndex(maxX, terrain.minX, terrain.nx);
    for (int ix = x0; ix <= x1; ++ix) {
      const glm::vec3 a = vertex(ix, iz), b = vertex(ix + 1, iz);
      const glm::vec3 c = vertex(ix, iz + 1), d = vertex(ix + 1, iz + 1);
      // Same diagonal as BuildTerrainMesh: a-b-d and a-d-c.
      for (const auto& triangle : {std::array{a, b, d}, std::array{a, d, c}}) {
        // A triangle clipped by three half-planes has at most six vertices.
        std::array<glm::vec3, 6> poly{}, clipped{};
        std::copy(triangle.begin(), triangle.end(), poly.begin());
        int count = 3;
        for (int edge = 0; edge < 3 && count > 0; ++edge) {
          const glm::vec3& origin = footprint[edge];
          const glm::vec3 direction = footprint[(edge + 1) % 3] - origin;
          const auto side = [&](const glm::vec3& p) {
            return winding * crossXZ(direction, p - origin);
          };
          int clippedCount = 0;
          glm::vec3 prev = poly[count - 1];
          float prevSide = side(prev);
          for (int v = 0; v < count; ++v) {
            const glm::vec3 cur = poly[v];
            const float curSide = side(cur);
            if ((prevSide >= 0.0f) != (curSide >= 0.0f)) {
              clipped[clippedCount++] = glm::mix(prev, cur, prevSide / (prevSide - curSide));
            }
            if (curSide >= 0.0f) clipped[clippedCount++] = cur;
            prev = cur;
            prevSide = curSide;
          }
          poly.swap(clipped);
          count = clippedCount;
        }
        for (int v = 1; v + 1 < count; ++v) {
          points->insert(points->end(), {poly[0], poly[v], poly[v + 1]});
        }
      }
    }
  }
}

// Builds a unit's ground-following FOV overlay spanning kShootHalfFovDegrees around
// FacingDirection(), capped at kFovConeVisualRange (bigger than the map
// diagonal) and clipped at the map boundary. Occlusion is 3D:
// the cone's tip is the unit's eye, so an obstacle below eye level only
// shadows the strip of ground it actually hides -- the overlay resumes where
// the sightline over its top edge lands, and only a target crouched at
// ground level right behind the obstacle stays hidden. Obstacles at or above
// eye level shadow everything behind them. Walk surfaces (decks/ramps) are
// thin slab occluders too, and the cone is laid on the top faces the viewer
// looks down on -- including the deck underfoot when the viewer stands on
// one, so the cone runs along the whole bridge chain and drops onto the
// ground beyond its edges. Top faces are not occluded by obstacles or by
// other decks (approximation).
std::vector<glm::vec3> BuildFovCone(const Unit& unit,
                                    const std::vector<tactics::Obstacle>& obstacles,
                                    const std::vector<AABB>& sidewalks, float mapHalfExtent,
                                    const tactics::HeightField& terrain,
                                    const std::vector<tactics::RoadSurface>& polygonSidewalks,
                                    const std::vector<tactics::WalkSurface>& walkSurfaces) {
  constexpr int kArcSegments = 24;
  constexpr float kGroundOffset = 0.015f;
  // Angular nudge to either side of an obstacle corner: one ray lands on the
  // occluding face right at the corner, the other shoots past it.
  constexpr float kCornerEpsilon = 1e-3f;
  const float halfFovRad = glm::radians(tactics::constants::kShootHalfFovDegrees);
  const float range = tactics::constants::kFovConeVisualRange;
  const glm::vec3 eye = unit.EyePosition();

  std::vector<DeckOccluder> decks;
  for (const tactics::WalkSurface& surface : walkSurfaces) {
    if (surface.vertices.size() < 3) continue;
    DeckOccluder deck;
    deck.surface = &surface;
    for (const glm::vec3& v : surface.vertices) deck.footprint.footprint.emplace_back(v.x, v.z);
    deck.topFacesEye = eye.y > tactics::SurfaceHeightAt(surface, eye.x, eye.z) + 1e-3f;
    decks.push_back(std::move(deck));
  }

  // Boundary ray angles as offsets from facingYaw in [-halfFov, +halfFov].
  // A uniform fan alone puts the occlusion edge on a chord between the two
  // samples straddling an obstacle corner, which reads as a skewed edge that
  // misses the corner; casting extra rays at each obstacle
  // corner (nudged to either side) pins the edge exactly onto the corner.
  std::vector<float> offsets;
  offsets.reserve(kArcSegments + 1 + (obstacles.size() + walkSurfaces.size()) * 12);
  for (int i = 0; i <= kArcSegments; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(kArcSegments);
    offsets.push_back(-halfFovRad + 2.0f * halfFovRad * t);
  }
  constexpr float kTwoPi = 6.28318530717958647692f;
  for (const auto& obstacle : obstacles) {
    for (const glm::vec2& p : tactics::ObstacleFootprint(obstacle)) {
      const float delta =
          std::remainder(std::atan2(p.y - eye.z, p.x - eye.x) - unit.facingYaw, kTwoPi);
      for (const float nudged : {delta - kCornerEpsilon, delta, delta + kCornerEpsilon}) {
        if (nudged >= -halfFovRad && nudged <= halfFovRad) offsets.push_back(nudged);
      }
    }
  }
  for (const DeckOccluder& deck : decks) {
    for (const glm::vec2& p : deck.footprint.footprint) {
      const float delta =
          std::remainder(std::atan2(p.y - eye.z, p.x - eye.x) - unit.facingYaw, kTwoPi);
      for (const float nudged : {delta - kCornerEpsilon, delta, delta + kCornerEpsilon}) {
        if (nudged >= -halfFovRad && nudged <= halfFovRad) offsets.push_back(nudged);
      }
    }
  }
  std::sort(offsets.begin(), offsets.end());

  std::vector<glm::vec2> dirs;
  std::vector<std::vector<GroundSpan>> spansPerRay;
  std::vector<std::vector<DeckTop>> deckTopsPerRay;
  deckTopsPerRay.reserve(offsets.size());
  dirs.reserve(offsets.size());
  spansPerRay.reserve(offsets.size());
  for (const float offset : offsets) {
    const float angle = unit.facingYaw + offset;
    const glm::vec2 dir(std::cos(angle), std::sin(angle));
    dirs.push_back(dir);
    const float clippedRange = ClipToMap(eye, dir, range, mapHalfExtent);
    deckTopsPerRay.emplace_back();
    spansPerRay.push_back(VisibleGroundSpans(eye, dir, obstacles, decks, clippedRange,
                                             &deckTopsPerRay.back()));
  }

  // Stitch adjacent rays into quads, one per matching visible span. Corner
  // rays keep the span structure identical across a slice except in the
  // epsilon-thin slivers at corners, where dropping unmatched spans is
  // invisible. Shadow boundaries of straight box edges are straight lines on
  // the ground, so the quads trace them exactly. Over terrain the footprint
  // is clipped to the rendered ground triangles and lifted slightly above
  // them; the occlusion spans are still computed against the flat ground plane.
  const auto groundPoint = [&](const glm::vec2& dir, float t) {
    const float x = eye.x + dir.x * t;
    const float z = eye.z + dir.y * t;
    float y = terrain.HeightAt(x, z);
    for (const tactics::RoadSurface& sidewalk : polygonSidewalks) {
      if (PatchContainsXZ(sidewalk, x, z)) {
        y = std::max(y, sidewalk.vertices.front().y);
      }
    }
    return glm::vec3(x, y + kGroundOffset, z);
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
      if (terrain.Empty()) {
        points.insert(points.end(), {l0, r0, r1, l0, r1, l1});
      } else {
        AppendTerrainOverlay(terrain, {l0, r0, r1}, &points);
        AppendTerrainOverlay(terrain, {l0, r1, l1}, &points);
      }
    }
  }
  const size_t groundPointCount = points.size();
  // Deck/ramp tops in view: stitch matching decks across adjacent rays and
  // lay the quad on the (planar) top face.
  const auto deckPoint = [&](int deck, const glm::vec2& dir, float t) {
    const float x = eye.x + dir.x * t;
    const float z = eye.z + dir.y * t;
    return glm::vec3(x, tactics::SurfaceHeightAt(*decks[deck].surface, x, z) + kGroundOffset, z);
  };
  for (size_t i = 0; i + 1 < offsets.size(); ++i) {
    for (const DeckTop& left : deckTopsPerRay[i]) {
      for (const DeckTop& right : deckTopsPerRay[i + 1]) {
        if (right.deck != left.deck) continue;
        const glm::vec3 l0 = deckPoint(left.deck, dirs[i], left.span.begin);
        const glm::vec3 l1 = deckPoint(left.deck, dirs[i], left.span.end);
        const glm::vec3 r0 = deckPoint(left.deck, dirs[i + 1], right.span.begin);
        const glm::vec3 r1 = deckPoint(left.deck, dirs[i + 1], right.span.end);
        for (const glm::vec3& p : {l0, r0, r1, l0, r1, l1}) points.push_back(p);
      }
    }
  }
  // Every walkable surface gets its own copy of the cone at its own height:
  // the ground-level cone above is buried under raised sidewalk slabs, so
  // clip the cone to each slab's footprint and lay that piece on its top.
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
  return points;
}

// Caller enables blending and caps overlapping cones with the stencil buffer.
// Alpha fades linearly from kFovAlpha at the unit to zero at the visual range.
void DrawFovCone(const Shader& shader, ColorTriangleMesh& mesh, const glm::mat4& viewProj,
                 const Unit& unit, const std::vector<glm::vec3>& points) {
  constexpr float kFovAlpha = 0.15f;
  const glm::vec3 eye = unit.EyePosition();
  const float range = tactics::constants::kFovConeVisualRange;
  const glm::vec3 baseColor = unit.team == Team::Blue ? glm::vec3(0.2f, 0.45f, 0.95f)
                                                      : glm::vec3(0.9f, 0.25f, 0.22f);
  std::vector<ColorTriangleMesh::Vertex> vertices;
  vertices.reserve(points.size());
  for (const glm::vec3& p : points) {
    const float dist = glm::length(glm::vec2(p.x - eye.x, p.z - eye.z));
    const float fade = std::clamp(1.0f - dist / range, 0.0f, 1.0f);
    vertices.push_back({p, glm::vec4(baseColor, kFovAlpha * fade)});
  }
  mesh.SetVertices(vertices);
  shader.SetMat4("uMVP", viewProj);
  mesh.Draw();
}

// Quartic falloff (matches the footprint shader) so the cone fades early.
inline float ShotConeFade(float u) {
  const float k = std::max(1.0f - u, 0.0f);
  return k * k * k * k;
}

// The selected figure's shot cone as a real 3D cone: apex at the gun tip, axis
// along the aim, fading out by the shot range. The side surface is depth
// tested, so ground and walls cut it off rather than receiving a projection;
// where it meets them, the footprint pass lights up exactly the part of each
// surface lying inside the cone volume, bright where the hit probability is
// high. Every figure standing in the cone (`litUnits`) catches the light,
// so a would-be target reads as highlighted before it is ever clicked.
// Caller is responsible for blending, polygon offset and depth state.
void DrawShotCone(const Shader& colorShader, const Shader& surfaceShader,
                  ColorTriangleMesh& mesh, const CubeMesh& cube, const SphereMesh& sphere,
                  const glm::mat4& viewProj, const Unit& unit,
                  const std::vector<const Unit*>& litUnits,
                  const std::vector<tactics::Obstacle>& obstacles,
                  const std::vector<AABB>& sidewalks, float mapHalfExtent) {
  constexpr int kSegments = 40;
  constexpr int kRings = 12;  // Along the axis, so the alpha fade interpolates smoothly.
  constexpr float kSurfaceAlpha = 0.18f;
  constexpr float kFootprintAlpha = 0.7f;
  constexpr float kTwoPi = 6.28318530717958647692f;
  const float tanHalf = std::tan(glm::radians(tactics::constants::kShotConeHalfAngleDegrees));
  const glm::vec3 apex = unit.MuzzlePosition();
  const glm::vec3 axis = unit.FacingDirection();
  const float fadeRange = tactics::kDefaultShotProfile.range;
  // The cone ends where its axis first meets an obstacle, so it never pokes
  // out the far side of a wall.
  float range = fadeRange;
  for (const auto& obstacle : obstacles) {
    float tEnter = 0.0f;
    float tExit = range;
    bool hit = true;
    for (int i = 0; i < 3 && hit; ++i) {
      if (std::abs(axis[i]) < 1e-6f) {
        hit = apex[i] >= obstacle.bounds.min[i] && apex[i] <= obstacle.bounds.max[i];
        continue;
      }
      float t0 = (obstacle.bounds.min[i] - apex[i]) / axis[i];
      float t1 = (obstacle.bounds.max[i] - apex[i]) / axis[i];
      if (t0 > t1) std::swap(t0, t1);
      tEnter = std::max(tEnter, t0);
      tExit = std::min(tExit, t1);
      hit = tEnter <= tExit;
    }
    if (hit && tEnter > 0.0f) range = tEnter;
  }
  const glm::vec3 right(-axis.z, 0.0f, axis.x);
  const glm::vec3 up(0.0f, 1.0f, 0.0f);
  const auto vertex = [&](int ring, int seg) {
    const float u = static_cast<float>(ring) / static_cast<float>(kRings);
    const float a = kTwoPi * static_cast<float>(seg) / static_cast<float>(kSegments);
    const float t = range * u;
    const glm::vec3 p =
        apex + axis * t + (right * std::cos(a) + up * std::sin(a)) * (t * tanHalf);
    return ColorTriangleMesh::Vertex{
        p, glm::vec4(kSetupColor, kSurfaceAlpha * ShotConeFade(t / fadeRange))};
  };
  std::vector<ColorTriangleMesh::Vertex> vertices;
  vertices.reserve(static_cast<size_t>(kRings * kSegments * 6));
  for (int ring = 0; ring < kRings; ++ring) {
    for (int seg = 0; seg < kSegments; ++seg) {
      vertices.push_back(vertex(ring, seg));
      vertices.push_back(vertex(ring + 1, seg));
      vertices.push_back(vertex(ring + 1, seg + 1));
      vertices.push_back(vertex(ring, seg));
      vertices.push_back(vertex(ring + 1, seg + 1));
      vertices.push_back(vertex(ring, seg + 1));
    }
  }
  colorShader.Use();
  mesh.SetVertices(vertices);
  colorShader.SetMat4("uMVP", viewProj);
  mesh.Draw();

  surfaceShader.Use();
  surfaceShader.SetVec3("uApex", apex);
  surfaceShader.SetVec3("uAxis", axis);
  surfaceShader.SetVec4("uCone",
                       glm::vec4(tanHalf, range + 0.05f, fadeRange, kFootprintAlpha));
  surfaceShader.SetVec3("uColor", kSetupColor);
  const auto drawModel = [&](const glm::mat4& model, const auto& modelMesh, bool requireFacing) {
    surfaceShader.SetMat4("uModel", model);
    surfaceShader.SetMat4("uMVP", viewProj * model);
    surfaceShader.SetInt("uRequireFacing", requireFacing ? 1 : 0);
    modelMesh.Draw();
  };
  const auto drawBox = [&](const glm::vec3& minCorner, const glm::vec3& size) {
    drawModel(glm::translate(glm::mat4(1.0f), minCorner) *
                  glm::scale(glm::mat4(1.0f), size),
              cube, false);
  };
  drawBox(glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
          glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f));
  for (const AABB& slab : sidewalks) drawBox(slab.min, slab.max - slab.min);
  for (const auto& obstacle : obstacles) {
    drawBox(obstacle.bounds.min, obstacle.bounds.max - obstacle.bounds.min);
  }
  for (const Unit* lit : litUnits) {
    for (const FigurePart& part : BuildFigure(*lit)) {
      if (part.primitive == FigurePrimitive::Rounded) {
        drawModel(part.model, sphere, true);
      } else {
        drawModel(part.model, cube, true);
      }
    }
  }
}

// Issue #136: everything `team`'s pane must keep visible this frame -- the
// team's own living figures, the movement-frontier overlay, movement
// indicators (preview/planned paths, destination ghosts) and remembered-
// sighting ghosts/arrows -- as world-space probe points. Any obstacle or
// deck slab crossing a camera->probe segment is drawn see-through.
std::vector<glm::vec3> CollectSeeThroughProbes(const GameLogic& game, Team team, bool fogActive,
                                               const PaneOverlays& overlays) {
  // Probes hover slightly above their surface so grazing contact with the
  // ground/deck they sit on never reads as occlusion.
  constexpr float kSurfaceLift = 0.3f;
  constexpr float kGhostLift = 0.9f;  // Mid-torso of a ghost/figure wireframe.
  std::vector<glm::vec3> probes;
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.team != team) continue;
    probes.push_back(unit.EyePosition());
    probes.push_back(unit.position + glm::vec3(0.0f, kSurfaceLift, 0.0f));
  }
  const auto addPolyline = [&](const std::vector<glm::vec3>& path) {
    for (size_t i = 0; i < path.size(); ++i) {
      probes.push_back(path[i] + glm::vec3(0.0f, kSurfaceLift, 0.0f));
      if (i + 1 == path.size()) continue;
      // Subdivide long chords so a stretch hidden mid-segment still probes.
      const int pieces = static_cast<int>(glm::distance(path[i], path[i + 1]) /
                                          kOcclusionProbeSpacing);
      for (int k = 1; k <= pieces; ++k) {
        probes.push_back(glm::mix(path[i], path[i + 1],
                                  static_cast<float>(k) / static_cast<float>(pieces + 1)) +
                         glm::vec3(0.0f, kSurfaceLift, 0.0f));
      }
    }
  };
  if (overlays.movePreviewPath) addPolyline(*overlays.movePreviewPath);
  if (overlays.moveFrontier && overlays.moveFrontier->nx > 0) {
    const tactics::ReachField& f = *overlays.moveFrontier;
    const int stride =
        std::max(1, static_cast<int>(std::round(kOcclusionProbeSpacing / f.step)));
    for (int iz = 0; iz < f.nz; iz += stride) {
      for (int ix = 0; ix < f.nx; ix += stride) {
        if (f.Reached(ix, iz)) {
          probes.push_back(f.Node(ix, iz) + glm::vec3(0.0f, kSurfaceLift, 0.0f));
        }
      }
    }
  }
  const bool planning =
      game.Mode() != InputMode::GameOver && game.Mode() != InputMode::Executing;
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || unit.team != team) continue;
      if (unit.plan.type != tactics::PlannedActionType::Move || unit.plan.movePath.size() < 2) {
        continue;
      }
      addPolyline(unit.plan.movePath);
      probes.push_back(unit.plan.movePath.back() +
                       glm::vec3(0.0f, kGhostLift, 0.0f));  // Destination ghost.
      for (const auto& leg : unit.plan.queuedLegs) addPolyline(leg);
    }
  }
  if (fogActive) {
    for (const Unit& unit : game.GetScene().units) {
      if (unit.team == team) continue;
      for (const auto& s : game.Sightings(team, unit.id)) {
        if (1.0f - s.ageRounds * tactics::constants::kSightingFadePerRound <= 0.0f) continue;
        probes.push_back(s.position + glm::vec3(0.0f, kGhostLift, 0.0f));  // Ghost + arrow.
      }
    }
  }
  return probes;
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
  if (!colorShader_.Compile(kColorVertexShaderSrc, kColorFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the vertex-color shader\n");
    return false;
  }
  if (!coneSurfaceShader_.Compile(kConeSurfaceVertexShaderSrc, kConeSurfaceFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the shot-cone surface shader\n");
    return false;
  }
  if (!fovMaskShader_.Compile(kFovMaskVertexShaderSrc, kFovMaskFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the FOV mask shader\n");
    return false;
  }
  cubeMesh_.Init();
  sphereMesh_.Init();
  frontierFill_.Init();
  frontierBorder_.Init();
  ziplineFill_.Init();
  ziplineBorder_.Init();
  pathLine_.Init();
  fovConeMesh_.Init();
  shotConeMesh_.Init();
  terrainMesh_.Init();
  geometryMesh_.Init();
  fovSceneMesh_.Init();
  highlightRing_.Init();
  highlightRing_.SetPoints(BuildRingPoints());

  // Issue #110: the two per-unit eye-space depth maps. Allocated up front
  // (16 MB of depth) even in CPU mode so flipping the mode at runtime needs
  // no re-init; they are only rendered to in ShadowMap mode.
  glGenFramebuffers(2, fovFbo_);
  glGenTextures(2, fovDepthTex_);
  for (int i = 0; i < 2; ++i) {
    glBindTexture(GL_TEXTURE_2D, fovDepthTex_[i]);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kFovMapWidth, kFovMapHeight, 0,
                 GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glBindFramebuffer(GL_FRAMEBUFFER, fovFbo_[i]);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, fovDepthTex_[i], 0);
    const GLenum noColorBuffer = GL_NONE;
    glDrawBuffers(1, &noColorBuffer);
    glReadBuffer(GL_NONE);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      std::fprintf(stderr, "FOV depth map framebuffer %d incomplete\n", i);
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
      return false;
    }
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Stage-C: a single directional light (simulating overhead factory
  // lighting) casting a PCF-filtered shadow map. The
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
  // scene's map size and camera framing and is recomputed per RenderPane.
  lightDir_ = glm::normalize(glm::vec3(0.35f, -1.0f, 0.25f));

  glEnable(GL_DEPTH_TEST);
  return true;
}

void SceneRenderer::Destroy() {
  frontierFill_.Destroy();
  frontierBorder_.Destroy();
  ziplineFill_.Destroy();
  ziplineBorder_.Destroy();
  pathLine_.Destroy();
  fovConeMesh_.Destroy();
  shotConeMesh_.Destroy();
  terrainFovCache_.clear();
  fovKeyObstacles_.clear();
  fovKeySidewalks_.clear();
  fovKeyMapHalfExtent_ = 0.0f;
  terrainMesh_.Destroy();
  geometryMesh_.Destroy();
  fovSceneMesh_.Destroy();
  fovSceneKey_ = 0;
  terrainKey_ = tactics::HeightField{};
  highlightRing_.Destroy();
  sphereMesh_.Destroy();
  cubeMesh_.Destroy();
  if (shadowDepthTex_) glDeleteTextures(1, &shadowDepthTex_);
  if (shadowFbo_) glDeleteFramebuffers(1, &shadowFbo_);
  shadowDepthTex_ = 0;
  shadowFbo_ = 0;
  if (fovDepthTex_[0]) glDeleteTextures(2, fovDepthTex_);
  if (fovFbo_[0]) glDeleteFramebuffers(2, fovFbo_);
  fovDepthTex_[0] = fovDepthTex_[1] = 0;
  fovFbo_[0] = fovFbo_[1] = 0;
}

void SceneRenderer::DrawFovShadowMask(const GameLogic& game, const Unit& unit,
                                      const glm::mat4& viewProj, bool drawTerrain,
                                      GLuint targetFramebuffer, int x, int y, int width,
                                      int height) {
  const tactics::Scene& scene = game.GetScene();
  const float mapHalfExtent = scene.mapHalfExtent;
  const glm::vec3 eye = unit.EyePosition();
  const float range = tactics::constants::kFovConeVisualRange;
  // Far plane: the cone's visual range, which already exceeds any map's
  // diagonal; the depth comparison is done in linearized depth so the
  // far-plane choice only affects precision, not correctness.
  const float far = range;
  const float aspect = std::tan(glm::radians(kFovMapHalfAzimuthDegrees)) /
                       std::tan(glm::radians(kFovMapHalfElevationDegrees));
  const glm::mat4 proj = glm::perspective(glm::radians(2.0f * kFovMapHalfElevationDegrees),
                                          aspect, kFovMapNear, far);
  glm::mat4 mapMatrix[2];
  for (int half = 0; half < 2; ++half) {
    const float yaw = unit.facingYaw + glm::radians(kFovMapHalfSplitDegrees) * (half == 0 ? -1.0f : 1.0f);
    const glm::vec3 dir(std::cos(yaw), 0.0f, std::sin(yaw));
    mapMatrix[half] = proj * glm::lookAt(eye, eye + dir, glm::vec3(0.0f, 1.0f, 0.0f));
  }

  // Depth passes: the static scene from the eye, into each half-map. The
  // ground/terrain is included so hills occlude; units are not (gameplay
  // LOS ignores them, and so does the CPU overlay).
  glDisable(GL_SCISSOR_TEST);
  glDisable(GL_STENCIL_TEST);
  glDisable(GL_BLEND);
  glDisable(GL_POLYGON_OFFSET_FILL);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_LESS);
  depthShader_.Use();
  for (int half = 0; half < 2; ++half) {
    glBindFramebuffer(GL_FRAMEBUFFER, fovFbo_[half]);
    glViewport(0, 0, kFovMapWidth, kFovMapHeight);
    glClear(GL_DEPTH_BUFFER_BIT);
    if (drawTerrain) {
      depthShader_.SetMat4("uLightMVP", mapMatrix[half]);
      terrainMesh_.Draw();
    } else {
      DrawBoxDepth(depthShader_, cubeMesh_, mapMatrix[half],
                   glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
                   glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f));
    }
    if (fovSceneMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", mapMatrix[half]);
      fovSceneMesh_.Draw();
    }
  }

  // Back to the pane: same blend/stencil/no-depth-write state as the CPU
  // overlay, with the mask drawn over the already-lit geometry (LEQUAL plus
  // a small offset so the identical surfaces pass their own depth).
  glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);
  glEnable(GL_SCISSOR_TEST);
  glViewport(x, y, width, height);
  glScissor(x, y, width, height);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_EQUAL, 0, 0xFF);
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(-1.0f, -2.0f);

  constexpr float kConeAlpha = 0.15f;
  const glm::vec4 baseColor = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                      : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  fovMaskShader_.Use();
  fovMaskShader_.SetVec3("uEye", eye);
  fovMaskShader_.SetVec3("uFacing", unit.FacingDirection());
  fovMaskShader_.SetFloat("uCosHalfFov",
                          std::cos(glm::radians(tactics::constants::kShootHalfFovDegrees)));
  fovMaskShader_.SetFloat("uRange", range);
  fovMaskShader_.SetMat4("uFovMatrix0", mapMatrix[0]);
  fovMaskShader_.SetMat4("uFovMatrix1", mapMatrix[1]);
  fovMaskShader_.SetFloat("uNear", kFovMapNear);
  fovMaskShader_.SetFloat("uFar", far);
  // The larger of the two texel footprints (vertical: the whole elevation
  // range over the map's height) per unit of map depth.
  fovMaskShader_.SetFloat("uTexelPerDepth",
                          std::max(2.0f * std::tan(glm::radians(kFovMapHalfElevationDegrees)) /
                                       kFovMapHeight,
                                   2.0f * std::tan(glm::radians(kFovMapHalfAzimuthDegrees)) /
                                       kFovMapWidth));
  fovMaskShader_.SetFloat("uProbeHeight", fovProbeHeight_);
  fovMaskShader_.SetVec4("uColor", glm::vec4(baseColor.r, baseColor.g, baseColor.b, kConeAlpha));
  fovMaskShader_.SetInt("uFovMap0", 1);
  fovMaskShader_.SetInt("uFovMap1", 2);
  glActiveTexture(GL_TEXTURE1);
  glBindTexture(GL_TEXTURE_2D, fovDepthTex_[0]);
  glActiveTexture(GL_TEXTURE2);
  glBindTexture(GL_TEXTURE_2D, fovDepthTex_[1]);
  glActiveTexture(GL_TEXTURE0);

  const auto drawReceiver = [&](const auto& mesh, const glm::mat4& model) {
    fovMaskShader_.SetMat4("uModel", model);
    fovMaskShader_.SetMat4("uMVP", viewProj * model);
    mesh.Draw();
  };
  if (drawTerrain) {
    drawReceiver(terrainMesh_, glm::mat4(1.0f));
  } else {
    drawReceiver(cubeMesh_, BoxModel(glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
                                     glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f)));
  }
  if (fovSceneMesh_.HasGeometry()) drawReceiver(fovSceneMesh_, glm::mat4(1.0f));
  glDepthFunc(GL_LESS);
}

void SceneRenderer::RenderPane(const GameLogic& game, Team team, bool fogActive,
                               const TeamVisibility& visibility, const OrbitCamera& camera, int x,
                               int y, int width, int height, const PaneOverlays& overlays,
                               GLuint targetFramebuffer, const RenderDebugOptions& debug) {
  const ScopedDrawStats drawStats(debug.stats);
  const auto& obstacles = game.GetScene().obstacles;
  const auto& sidewalks = game.GetScene().sidewalks;
  const float mapHalfExtent = game.GetScene().mapHalfExtent;
  const tactics::HeightField& terrain = game.GetScene().ground;
  // (Re)triangulate the terrain only when the scene's heightfield changes.
  const bool terrainChanged =
      terrainKey_.heights != terrain.heights || terrainKey_.nx != terrain.nx ||
      terrainKey_.nz != terrain.nz || terrainKey_.minX != terrain.minX ||
      terrainKey_.minZ != terrain.minZ || terrainKey_.step != terrain.step;
  if (terrainChanged) {
    if (!terrain.Empty()) BuildTerrainMesh(terrain, &terrainMesh_);
    terrainKey_ = terrain;
  }
  // Terrain clipping is more expensive than the flat fan. Reuse each unit's
  // geometry while it stands still, invalidating on map contents rather
  // than scene addresses (which can be reused between scenarios).
  const auto sameBox = [](const AABB& a, const AABB& b) {
    return a.min == b.min && a.max == b.max;
  };
  if (terrainChanged || mapHalfExtent != fovKeyMapHalfExtent_ ||
      !std::equal(obstacles.begin(), obstacles.end(), fovKeyObstacles_.begin(),
                  fovKeyObstacles_.end(), [&](const auto& a, const auto& b) {
                    return sameBox(a.bounds, b.bounds);
                  }) ||
      !std::equal(sidewalks.begin(), sidewalks.end(), fovKeySidewalks_.begin(),
                  fovKeySidewalks_.end(), sameBox)) {
    terrainFovCache_.clear();
    fovKeyObstacles_ = obstacles;
    fovKeySidewalks_ = sidewalks;
    fovKeyMapHalfExtent_ = mapHalfExtent;
  }
  const bool drawTerrain = !terrain.Empty() && terrainMesh_.HasGeometry();

  // Fit the light frustum to the camera's current zoom instead of spreading
  // the shadow map over the whole scene. Cap it at the previous whole-map
  // coverage so zoomed-out views retain the same bounds.
  const float cameraDistance = glm::distance(camera.Position(), camera.target);
  const float orthoHalfExtent = std::min(mapHalfExtent * 1.5f, cameraDistance);
  const float shadowTexelWorldSize = orthoHalfExtent * 2.0f / kShadowMapSize;

  // Keep the shadow texel grid fixed in world space while panning. Snap in
  // the light's X/Y basis (rather than world X/Z), since those are the axes
  // that map onto the shadow texture.
  const float lightDistance = mapHalfExtent * 3.0f;
  const glm::mat4 lightBasis = glm::lookAt(-lightDir_, glm::vec3(0.0f),
                                           glm::vec3(0.0f, 1.0f, 0.0f));
  glm::vec4 centerInLightSpace = lightBasis * glm::vec4(camera.target, 1.0f);
  centerInLightSpace.x = std::round(centerInLightSpace.x / shadowTexelWorldSize) *
                         shadowTexelWorldSize;
  centerInLightSpace.y = std::round(centerInLightSpace.y / shadowTexelWorldSize) *
                         shadowTexelWorldSize;
  const glm::vec3 snappedCenter = glm::vec3(glm::inverse(lightBasis) * centerInLightSpace);
  const glm::mat4 lightView =
      glm::lookAt(snappedCenter - lightDir_ * lightDistance, snappedCenter,
                  glm::vec3(0.0f, 1.0f, 0.0f));
  lightSpaceMatrix_ = glm::ortho(-orthoHalfExtent, orthoHalfExtent, -orthoHalfExtent,
                                 orthoHalfExtent, 0.1f, lightDistance * 2.0f) *
                      lightView;

  // Shadow pass: only casters this team can currently see.
  glDisable(GL_SCISSOR_TEST);
  if (!debug.disableShadows) {
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glViewport(0, 0, kShadowMapSize, kShadowMapSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    depthShader_.Use();
    // Hills cast shadows (into valleys, onto units); the flat ground never
    // could, so flat scenes skip the extra caster exactly as before.
    if (drawTerrain) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      terrainMesh_.Draw();
    }
    for (const auto& obstacle : obstacles) {
      const AABB& bounds = obstacle.bounds;
      if (obstacle.footprint.empty()) {
        DrawBoxDepth(depthShader_, cubeMesh_, lightSpaceMatrix_, bounds.min,
                     bounds.max - bounds.min);
      } else {
        BuildPolygonPrism(obstacle, &geometryMesh_);
        depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
        geometryMesh_.Draw();
      }
    }
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(kDeckShadowSlopeBias, kDeckShadowConstantBias);
    for (const tactics::WalkSurface& surface : game.GetScene().walkSurfaces) {
      const std::vector<char> hidden = SharedEdges(game.GetScene().walkSurfaces, surface);
      BuildSurfacePatch(surface.vertices, 0.45f, &geometryMesh_, &hidden);
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      geometryMesh_.Draw();
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    for (const Unit& unit : game.GetScene().units) {
      if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
      DrawUnitDepth(depthShader_, cubeMesh_, sphereMesh_, lightSpaceMatrix_, unit);
    }
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

  // Issue #136 camera-occlusion see-through: per frame, raycast from the
  // camera to every point the player needs to see (own figures, movement
  // frontier, move paths/ghosts, sighting ghosts) and mark each obstacle or
  // deck slab actually crossing a sightline. Marked occluders are skipped by
  // the opaque pass below and re-drawn translucent after the figures.
  const auto& walkSurfaces = game.GetScene().walkSurfaces;
  std::vector<char> fadeObstacle(obstacles.size(), 0);
  std::vector<char> fadeSurface(walkSurfaces.size(), 0);
  bool anyFaded = false;
  if (!debug.disableOcclusionFade) {
    const std::vector<glm::vec3> probes =
        CollectSeeThroughProbes(game, team, fogActive, overlays);
    const glm::vec3 cameraPos = camera.Position();
    struct ProbeRay {
      glm::vec3 direction;
      float length;
    };
    std::vector<ProbeRay> rays;
    rays.reserve(probes.size());
    for (const glm::vec3& probe : probes) {
      const glm::vec3 segment = probe - cameraPos;
      const float length = glm::length(segment);
      if (length > 1e-4f) rays.push_back({segment / length, length});
    }
    // Same endpoint handling as LineOfSightClear: a hit at (or essentially
    // at) the probe itself does not count, so a figure hugging a wall does
    // not fade that wall.
    constexpr float kEndpointEpsilon = 1e-2f;
    for (size_t i = 0; i < obstacles.size(); ++i) {
      for (const ProbeRay& ray : rays) {
        float t = 0.0f;
        if (tactics::RayIntersectsObstacle(cameraPos, ray.direction, obstacles[i], &t) &&
            t > kEndpointEpsilon && t < ray.length - kEndpointEpsilon) {
          fadeObstacle[i] = 1;
          anyFaded = true;
          break;
        }
      }
    }
    for (size_t i = 0; i < walkSurfaces.size(); ++i) {
      for (const ProbeRay& ray : rays) {
        float t = 0.0f;
        if (tactics::RayIntersectsWalkSurface(cameraPos, ray.direction, walkSurfaces[i],
                                              kDeckThickness, &t) &&
            t > kEndpointEpsilon && t < ray.length - kEndpointEpsilon) {
          fadeSurface[i] = 1;
          anyFaded = true;
          break;
        }
      }
    }
  }

  litShader_.Use();
  litShader_.SetVec3("uLightDir", lightDir_);
  litShader_.SetVec3("uViewPos", camera.Position());
  litShader_.SetInt("uShadowMap", 0);
  litShader_.SetInt("uDisableShadows", debug.disableShadows ? 1 : 0);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, shadowDepthTex_);

  if (drawTerrain) {
    // Earthy green-brown so the hills read as terrain rather than factory floor.
    DrawLitModel(litShader_, terrainMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(0.30f, 0.36f, 0.22f, 1.0f));
  } else {
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_,
               glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
               glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f),
               glm::vec4(0.16f, 0.18f, 0.20f, 1.0f));
  }

  for (const AABB& slab : game.GetScene().sidewalks) {
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, slab.min, slab.max - slab.min,
               glm::vec4(0.36f, 0.37f, 0.39f, 1.0f));
  }

  for (const tactics::RoadSurface& sidewalk : game.GetScene().sidewalkSurfaces) {
    BuildSurfacePatch(sidewalk.vertices, 0.10f, &geometryMesh_);
    DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(0.36f, 0.37f, 0.39f, 1.0f));
  }

  for (const tactics::RoadSurface& road : game.GetScene().roads) {
    BuildSurfacePatch(road.vertices, 0.0f, &geometryMesh_);
    DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(0.13f, 0.14f, 0.16f, 1.0f));
  }
  for (size_t i = 0; i < walkSurfaces.size(); ++i) {
    if (fadeSurface[i]) continue;  // Re-drawn translucent below.
    // Concrete, clearly lighter than the asphalt below: the elevated deck
    // and its ramps must read as a bridge, not as more ground road.
    const std::vector<char> hidden = SharedEdges(walkSurfaces, walkSurfaces[i]);
    BuildSurfacePatch(walkSurfaces[i].vertices, kDeckThickness, &geometryMesh_, &hidden);
    DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(kDeckColor, 1.0f));
  }

  for (size_t i = 0; i < obstacles.size(); ++i) {
    if (fadeObstacle[i]) continue;  // Re-drawn translucent below.
    const AABB& bounds = obstacles[i].bounds;
    const glm::vec4 color(kObstacleColor, 1.0f);
    if (obstacles[i].footprint.empty()) {
      DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, bounds.min,
                 bounds.max - bounds.min, color);
    } else {
      BuildPolygonPrism(obstacles[i], &geometryMesh_);
      DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f), color);
    }
  }

  // Zipline anchor posts (the cable itself is drawn with the line overlays).
  for (const tactics::Zipline& line : game.GetScene().ziplines) {
    for (const glm::vec3& foot : {line.a, line.b}) {
      constexpr float kPostHalf = 0.12f;
      DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_,
                 foot + glm::vec3(-kPostHalf, 0.0f, -kPostHalf),
                 glm::vec3(2.0f * kPostHalf, tactics::constants::kZiplinePostHeight,
                           2.0f * kPostHalf),
                 glm::vec4(0.42f, 0.30f, 0.18f, 1.0f));
    }
  }

  for (const Unit& unit : game.GetScene().units) {
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawUnit(litShader_, cubeMesh_, sphereMesh_, viewProj, lightSpaceMatrix_, unit);
  }

  // Issue #136: alpha-blended re-draw of the occluders marked above, sorted
  // front-to-back and stencil-capped so each pixel is blended exactly once
  // (the nearest occluder): stacked buildings read as one flat
  // kOccluderFadeOpacity instead of compounding toward opaque. Depth writes off -- the figures already drawn and
  // the depth-tested overlays drawn later (frontier fill, FOV cones, paths)
  // all show through them. Shadow casting and the FOV depth maps are left
  // untouched: a faded block is still physically there and still blocks
  // unit sightlines; only the camera gets to see through it.
  if (anyFaded) {
    struct FadedOccluder {
      float viewDistance;
      int obstacle;  // Index into obstacles, or -1 when `surface` is set.
      int surface;   // Index into walkSurfaces, or -1.
    };
    const glm::vec3 cameraPos = camera.Position();
    std::vector<FadedOccluder> faded;
    for (size_t i = 0; i < obstacles.size(); ++i) {
      if (fadeObstacle[i]) {
        faded.push_back({glm::distance(cameraPos, obstacles[i].bounds.Center()),
                         static_cast<int>(i), -1});
      }
    }
    for (size_t i = 0; i < walkSurfaces.size(); ++i) {
      if (fadeSurface[i]) {
        faded.push_back({glm::distance(cameraPos, tactics::SurfaceCenter(walkSurfaces[i])), -1,
                         static_cast<int>(i)});
      }
    }
    std::sort(faded.begin(), faded.end(), [](const FadedOccluder& a, const FadedOccluder& b) {
      return a.viewDistance < b.viewDistance;
    });
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_EQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
    for (const FadedOccluder& f : faded) {
      if (f.obstacle >= 0) {
        const tactics::Obstacle& obstacle = obstacles[f.obstacle];
        const glm::vec4 color(kObstacleColor, kOccluderFadeOpacity);
        if (obstacle.footprint.empty()) {
          DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, obstacle.bounds.min,
                     obstacle.bounds.max - obstacle.bounds.min, color);
        } else {
          BuildPolygonPrism(obstacle, &geometryMesh_);
          DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                       color);
        }
      } else {
        const std::vector<char> hidden = SharedEdges(walkSurfaces, walkSurfaces[f.surface]);
        BuildSurfacePatch(walkSurfaces[f.surface].vertices, kDeckThickness, &geometryMesh_,
                          &hidden);
        DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                     glm::vec4(kDeckColor, kOccluderFadeOpacity));
      }
    }
    glDisable(GL_STENCIL_TEST);
    glClear(GL_STENCIL_BUFFER_BIT);  // Later passes expect a clean stencil.
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  // Navmesh boundary debug overlay (opaque lines, depth-tested like the
  // scene geometry so cells hidden behind obstacles/hills read correctly).
  if (overlays.navMeshDebug) {
    unlitShader_.Use();
    DrawNavMeshDebug(unlitShader_, pathLine_, viewProj, *overlays.navMeshDebug, terrain);
  }

  // Rings sit on whatever flat slab (sidewalk) is under them, not inside it.
  const auto OnSurface = [&](glm::vec3 position) {
    for (const AABB& slab : game.GetScene().sidewalks) {
      if (position.x >= slab.min.x && position.x <= slab.max.x && position.z >= slab.min.z &&
          position.z <= slab.max.z) {
        position.y = std::max(position.y, slab.max.y);
      }
    }
    for (const tactics::RoadSurface& sidewalk : game.GetScene().sidewalkSurfaces) {
      if (PatchContainsXZ(sidewalk, position.x, position.z)) {
        position.y = std::max(position.y, sidewalk.vertices.front().y);
      }
    }
    return position;
  };
  // Unit normal of the walk surface under `position`: a ramp/deck plane, the
  // terrain slope, or straight up on flat slabs and ground.
  const auto SurfaceNormal = [&](const glm::vec3& position) {
    for (const tactics::WalkSurface& surface : game.GetScene().walkSurfaces) {
      if (surface.vertices.size() < 3) continue;
      if (!tactics::SurfaceContainsXZ(surface, position.x, position.z)) continue;
      if (std::fabs(tactics::SurfaceHeightAt(surface, position.x, position.z) - position.y) > 0.3f)
        continue;
      glm::vec3 n = glm::cross(surface.vertices[1] - surface.vertices[0],
                               surface.vertices[2] - surface.vertices[0]);
      if (glm::length(n) < 1e-6f) continue;
      n = glm::normalize(n);
      return n.y < 0.0f ? -n : n;
    }
    const tactics::HeightField& ground = game.GetScene().ground;
    if (!ground.Empty() && std::fabs(ground.HeightAt(position.x, position.z) - position.y) < 0.3f) {
      const float h = 0.5f * ground.step;
      const float dx = ground.HeightAt(position.x + h, position.z) - ground.HeightAt(position.x - h, position.z);
      const float dz = ground.HeightAt(position.x, position.z + h) - ground.HeightAt(position.x, position.z - h);
      return glm::normalize(glm::vec3(-dx / (2.0f * h), 1.0f, -dz / (2.0f * h)));
    }
    return glm::vec3(0.0f, 1.0f, 0.0f);
  };
  const auto DrawHighlightOnSurface = [&](const glm::vec3& position, const glm::vec4& color) {
    DrawHighlight(unlitShader_, highlightRing_, viewProj, OnSurface(position), color,
                  SurfaceNormal(position));
  };
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
  colorShader_.Use();
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
  if (fovOverlayMode_ == FovOverlayMode::ShadowMap) {
    // Issue #110 prototype: projective per-unit mask instead of the
    // analytic ground overlay. Static receivers/casters are cached per scene.
    const unsigned long long sceneKey = FovSceneFingerprint(game.GetScene());
    if (sceneKey != fovSceneKey_) {
      BuildFovSceneMesh(game.GetScene(), &fovSceneMesh_);
      fovSceneKey_ = sceneKey;
    }
    for (const Unit& unit : game.GetScene().units) {
      if (debug.disableFov || !unit.alive || unit.team != team) continue;
      DrawFovShadowMask(game, unit, viewProj, drawTerrain, targetFramebuffer, x, y, width, height);
    }
    colorShader_.Use();
  } else {
    const auto build = [&](const Unit& unit) {
      return BuildFovCone(unit, obstacles, sidewalks, mapHalfExtent, terrain,
                          game.GetScene().sidewalkSurfaces, game.GetScene().walkSurfaces);
    };
    for (const Unit& unit : game.GetScene().units) {
      if (debug.disableFov || !unit.alive || unit.team != team) continue;
      if (terrain.Empty()) {
        DrawFovCone(colorShader_, fovConeMesh_, viewProj, unit, build(unit));
        continue;
      }
      auto cached = std::find_if(terrainFovCache_.begin(), terrainFovCache_.end(),
                                 [&](const auto& entry) { return entry.unitId == unit.id; });
      const glm::vec3 eye = unit.EyePosition();
      if (cached == terrainFovCache_.end()) {
        terrainFovCache_.push_back({unit.id, eye, unit.facingYaw, build(unit)});
        cached = terrainFovCache_.end() - 1;
      } else if (cached->eye != eye || cached->facingYaw != unit.facingYaw) {
        cached->eye = eye;
        cached->facingYaw = unit.facingYaw;
        cached->points = build(unit);
      }
      DrawFovCone(colorShader_, fovConeMesh_, viewProj, unit, cached->points);
    }
  }
  glDisable(GL_STENCIL_TEST);
  // Shot probability cones are setup feedback, drawn over the FOV overlay:
  // the selected figure gets one while choosing a target, and every planned
  // shot keeps its cone until the round is committed (like a planned move's
  // path and destination ghost).
  const bool planning =
      game.Mode() != InputMode::GameOver && game.Mode() != InputMode::Executing;
  // Every rendered figure other than the shooter can catch the cone's light,
  // so anything standing in the beam is highlighted before it is targeted.
  const auto coneLitUnits = [&](const Unit& shooter) {
    std::vector<const Unit*> lit;
    for (const Unit& other : game.GetScene().units) {
      if (other.id == shooter.id) continue;
      if (!IsUnitVisibleForRender(other, team, fogActive, visibility)) continue;
      lit.push_back(&other);
    }
    return lit;
  };
  const Unit* aimingShooter = nullptr;
  if (overlays.selectionHighlight && overlays.showShotCone) {
    if (const Unit* selected = game.FindUnit(*game.SelectedUnitId())) {
      if (selected->alive) {
        // Depth-tested against the world so ground, slabs and walls cut the
        // cone off; LEQUAL lets the footprint pass land on the very surfaces
        // already in the depth buffer.
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        DrawShotCone(colorShader_, coneSurfaceShader_, shotConeMesh_, cubeMesh_, sphereMesh_,
                     viewProj, *selected, coneLitUnits(*selected), obstacles,
                     game.GetScene().sidewalks, game.GetScene().mapHalfExtent);
        glDepthFunc(GL_LESS);
        aimingShooter = selected;
      }
    }
  }
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || unit.team != team ||
          unit.plan.type != tactics::PlannedActionType::Shoot || &unit == aimingShooter) {
        continue;
      }
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LEQUAL);
      DrawShotCone(colorShader_, coneSurfaceShader_, shotConeMesh_, cubeMesh_, sphereMesh_,
                   viewProj, unit, coneLitUnits(unit), obstacles,
                   game.GetScene().sidewalks, game.GetScene().mapHalfExtent);
      glDepthFunc(GL_LESS);
    }
  }
  glDisable(GL_POLYGON_OFFSET_FILL);
  unlitShader_.Use();

  // Zipline cable: post top to post top, with a slight sag.
  const auto cablePoints = [](const tactics::Zipline& line) {
    constexpr int kPieces = 10;
    constexpr float kSag = 0.3f;
    const glm::vec3 up(0.0f, tactics::constants::kZiplinePostHeight, 0.0f);
    std::vector<glm::vec3> points;
    for (int i = 0; i <= kPieces; ++i) {
      const float t = static_cast<float>(i) / kPieces;
      points.push_back(glm::mix(line.a, line.b, t) + up -
                       glm::vec3(0.0f, kSag * 4.0f * t * (1.0f - t), 0.0f));
    }
    return points;
  };
  unlitShader_.SetMat4("uMVP", viewProj);
  unlitShader_.SetVec4("uColor", glm::vec4(0.12f, 0.12f, 0.12f, 1.0f));
  for (const tactics::Zipline& line : game.GetScene().ziplines) {
    pathLine_.SetPoints(cablePoints(line));
    pathLine_.Draw();
  }

  // Builds one reach field's glow + boundary geometry (see below) in `color`,
  // appending to `fill`/`border`. Shared by the walk region and the zipline
  // regions, which only differ in tint.
  const auto appendFrontier = [&](const tactics::ReachField& f, const glm::vec3& color,
                                  std::vector<ColorTriangleMesh::Vertex>& fill,
                                  std::vector<glm::vec3>& border) {
    constexpr float kY = 0.03f;
    constexpr float kSlabOffset = 0.015f;  // Above a sidewalk slab's top face.
    constexpr float kFadeWidth = 0.8f;   // World units from boundary to transparent.
    constexpr float kEdgeAlpha = 0.65f;  // Fill alpha right at the boundary.
    const int nx = f.nx, nz = f.nz;
    const float inf = std::numeric_limits<float>::infinity();

    // Nodes whose surface heights differ by more than a step belong to
    // different walk layers (deck vs. ground, or either side of a deck
    // edge). Nothing -- distance, blur, triangles -- may span them, or the
    // frontier hangs off the deck edge as vertical curtains.
    constexpr float kLayerStep = 0.6f;
    constexpr float kMaxDepth = 4.0f;  // Caps distance when no same-layer boundary exists.
    const auto sameLayer = [&](int ax, int az, int bx, int bz) {
      if (f.surfaceY.size() != static_cast<size_t>(nx) * nz) return true;
      return std::abs(f.surfaceY[az * nx + ax] - f.surfaceY[bz * nx + bx]) <= kLayerStep;
    };
    // A node on its layer's edge counts as touching the unreached class.
    const auto onLayerEdge = [&](int ix, int iz) {
      constexpr int kDx[4] = {-1, 1, 0, 0}, kDz[4] = {0, 0, -1, 1};
      for (int k = 0; k < 4; ++k) {
        const int jx = ix + kDx[k], jz = iz + kDz[k];
        if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) continue;
        if (!sameLayer(ix, iz, jx, jz)) return true;
      }
      return false;
    };

    // Chamfer distance (in world units) from each node to the nearest node
    // of the opposite reached/unreached class, within its own layer.
    auto chamfer = [&](bool target) {
      std::vector<float> d(static_cast<size_t>(nx) * nz, inf);
      for (int iz = 0; iz < nz; ++iz)
        for (int ix = 0; ix < nx; ++ix)
          if (f.Reached(ix, iz) == target ||
              (!target && f.Reached(ix, iz) && onLayerEdge(ix, iz)))
            d[iz * nx + ix] = 0.0f;
      const float s = f.step, sd = f.step * 1.41421356f;
      auto relax = [&](int ix, int iz, int dx, int dz, float w) {
        const int jx = ix + dx, jz = iz + dz;
        if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) return;
        if (!sameLayer(ix, iz, jx, jz)) return;
        float& v = d[iz * nx + ix];
        v = std::min(v, d[jz * nx + jx] + w);
      };
      for (int iz = 0; iz < nz; ++iz)
        for (int ix = 0; ix < nx; ++ix) {
          relax(ix, iz, -1, 0, s);
          relax(ix, iz, 0, -1, s);
          relax(ix, iz, -1, -1, sd);
          relax(ix, iz, 1, -1, sd);
        }
      for (int iz = nz - 1; iz >= 0; --iz)
        for (int ix = nx - 1; ix >= 0; --ix) {
          relax(ix, iz, 1, 0, s);
          relax(ix, iz, 0, 1, s);
          relax(ix, iz, 1, 1, sd);
          relax(ix, iz, -1, 1, sd);
        }
      return d;
    };
    const std::vector<float> dIn = chamfer(false);   // Distance to nearest unreached.
    const std::vector<float> dOut = chamfer(true);   // Distance to nearest reached.
    std::vector<float> g(dIn.size());
    for (int iz = 0; iz < nz; ++iz)
      for (int ix = 0; ix < nx; ++ix) {
        const int i = iz * nx + ix;
        // Signed depth: positive inside, zero contour half a step out.
        g[i] = f.Reached(ix, iz) ? std::min(dIn[i], kMaxDepth) - 0.5f * f.step
                                 : -(std::min(dOut[i], kMaxDepth) - 0.5f * f.step);
      }
    // Separable box blur (radius 2, two passes) rounds off the grid steps.
    std::vector<float> tmp(g.size());
    for (int pass = 0; pass < 2; ++pass) {
      for (int iz = 0; iz < nz; ++iz)
        for (int ix = 0; ix < nx; ++ix) {
          float sum = 0.0f;
          for (int k = -2; k <= 2; ++k) {
            const int jx = std::clamp(ix + k, 0, nx - 1);
            sum += sameLayer(ix, iz, jx, iz) ? g[iz * nx + jx] : g[iz * nx + ix];
          }
          tmp[iz * nx + ix] = sum / 5.0f;
        }
      for (int iz = 0; iz < nz; ++iz)
        for (int ix = 0; ix < nx; ++ix) {
          float sum = 0.0f;
          for (int k = -2; k <= 2; ++k) {
            const int jz = std::clamp(iz + k, 0, nz - 1);
            sum += sameLayer(ix, iz, ix, jz) ? tmp[jz * nx + ix] : tmp[iz * nx + ix];
          }
          g[iz * nx + ix] = sum / 5.0f;
        }
    }

    struct Pt {
      glm::vec3 p;
      float g;
    };
    const std::vector<AABB>& slabs = game.GetScene().sidewalks;
    // Clips a convex polygon to a slab's XZ footprint (Sutherland-Hodgman),
    // interpolating the field value, and lifts it onto the slab's top.
    auto clipToSlab = [&](const std::vector<Pt>& in, const AABB& slab) {
      std::vector<Pt> poly = in;
      for (int edge = 0; edge < 4 && !poly.empty(); ++edge) {
        const float bound = edge == 0 ? slab.min.x : edge == 1 ? slab.max.x
                            : edge == 2 ? slab.min.z : slab.max.z;
        const auto coord = [&](const Pt& q) { return edge < 2 ? q.p.x : q.p.z; };
        const auto inside = [&](const Pt& q) {
          return edge % 2 == 0 ? coord(q) >= bound : coord(q) <= bound;
        };
        std::vector<Pt> out;
        for (size_t v = 0; v < poly.size(); ++v) {
          const Pt& cur = poly[v];
          const Pt& prev = poly[(v + poly.size() - 1) % poly.size()];
          if (inside(cur) != inside(prev)) {
            const float t = (bound - coord(prev)) / (coord(cur) - coord(prev));
            out.push_back({prev.p + (cur.p - prev.p) * t, prev.g + (cur.g - prev.g) * t});
          }
          if (inside(cur)) out.push_back(cur);
        }
        poly = std::move(out);
      }
      for (Pt& q : poly) q.p.y = slab.max.y + kSlabOffset;
      return poly;
    };
    auto toVertex = [&](const Pt& q) {
      const float a = kEdgeAlpha * std::clamp(1.0f - q.g / kFadeWidth, 0.0f, 1.0f);
      return ColorTriangleMesh::Vertex{q.p, glm::vec4(color, a)};
    };
    for (int iz = 0; iz + 1 < nz; ++iz) {
      for (int ix = 0; ix + 1 < nx; ++ix) {
        const int cx[4] = {ix, ix + 1, ix + 1, ix};
        const int cz[4] = {iz, iz, iz + 1, iz + 1};
        Pt quad[4];
        bool anyIn = false;
        for (int k = 0; k < 4; ++k) {
          quad[k] = {f.Node(cx[k], cz[k]), g[cz[k] * nx + cx[k]]};
          // Each node rides the terrain under it (HeightAt is 0 on flat
          // maps); the 0.25-unit grid is fine enough that the linear
          // contour/edge interpolation below stays on the slope.
          for (const tactics::RoadSurface& sidewalk : game.GetScene().sidewalkSurfaces) {
            if (PatchContainsXZ(sidewalk, quad[k].p.x, quad[k].p.z)) {
              quad[k].p.y = std::max(quad[k].p.y, sidewalk.vertices.front().y);
            }
          }
          quad[k].p.y += kY;
          anyIn |= quad[k].g > 0.0f;
        }
        if (!anyIn) continue;
        // Never triangulate across a layer step (deck edge).
        if (!sameLayer(cx[0], cz[0], cx[1], cz[1]) || !sameLayer(cx[1], cz[1], cx[2], cz[2]) ||
            !sameLayer(cx[2], cz[2], cx[3], cz[3]) || !sameLayer(cx[3], cz[3], cx[0], cz[0]))
          continue;
        // Clip the cell to g >= 0 (Sutherland-Hodgman against the field).
        std::vector<Pt> poly;
        std::vector<Pt> cut;  // Contour crossings, in polygon order.
        for (int k = 0; k < 4; ++k) {
          const Pt& a = quad[k];
          const Pt& b = quad[(k + 1) % 4];
          const bool ain = a.g >= 0.0f, bin = b.g >= 0.0f;
          if (ain) poly.push_back(a);
          if (ain != bin) {
            const float t = a.g / (a.g - b.g);
            const Pt c{a.p + (b.p - a.p) * t, 0.0f};
            poly.push_back(c);
            cut.push_back(c);
          }
        }
        for (size_t k = 1; k + 1 < poly.size(); ++k) {
          fill.push_back(toVertex(poly[0]));
          fill.push_back(toVertex(poly[k]));
          fill.push_back(toVertex(poly[k + 1]));
        }
        if (cut.size() == 2) {
          border.push_back(cut[0].p);
          border.push_back(cut[1].p);
        }
        // The ground-level copy is buried under raised sidewalk slabs, so
        // lay a clipped copy on top of each slab the cell overlaps.
        for (const AABB& slab : slabs) {
          const std::vector<Pt> piece = clipToSlab(poly, slab);
          for (size_t k = 1; k + 1 < piece.size(); ++k) {
            fill.push_back(toVertex(piece[0]));
            fill.push_back(toVertex(piece[k]));
            fill.push_back(toVertex(piece[k + 1]));
          }
          if (cut.size() == 2) {
            // Liang-Barsky clip of the border segment to the slab footprint.
            const glm::vec3 a = cut[0].p, d = cut[1].p - cut[0].p;
            float t0 = 0.0f, t1 = 1.0f;
            const float lo[2] = {slab.min.x, slab.min.z}, hi[2] = {slab.max.x, slab.max.z};
            const float a2[2] = {a.x, a.z}, d2[2] = {d.x, d.z};
            bool vis = true;
            for (int ax = 0; ax < 2 && vis; ++ax) {
              if (std::abs(d2[ax]) < 1e-9f) {
                vis = a2[ax] >= lo[ax] && a2[ax] <= hi[ax];
                continue;
              }
              float ta = (lo[ax] - a2[ax]) / d2[ax], tb = (hi[ax] - a2[ax]) / d2[ax];
              if (ta > tb) std::swap(ta, tb);
              t0 = std::max(t0, ta);
              t1 = std::min(t1, tb);
              vis = t0 < t1;
            }
            if (vis) {
              const float y = slab.max.y + kSlabOffset;
              border.emplace_back(a.x + d.x * t0, y, a.z + d.z * t0);
              border.emplace_back(a.x + d.x * t1, y, a.z + d.z * t1);
            }
          }
        }
      }
    }
  };

  // Movement frontier: a glow hugging the reach boundary -- brightest at the
  // boundary, fading to fully transparent within kFadeWidth inside it. The
  // boundary is the zero contour of a blurred signed-distance field, so it
  // is smooth rather than following the sampling grid.
  if (overlays.moveFrontier && overlays.moveFrontier->nx > 0) {
    const tactics::ReachField& f = *overlays.moveFrontier;
    const glm::vec2 origin(f.minX, f.minZ);
    if (frontierKeyField_ != &f || frontierKeyOrigin_ != origin ||
        frontierKeyBudget_ != f.budget) {
      frontierKeyField_ = &f;
      frontierKeyOrigin_ = origin;
      frontierKeyBudget_ = f.budget;
      std::vector<ColorTriangleMesh::Vertex> fill;
      std::vector<glm::vec3> border;
      appendFrontier(f, kSetupColor, fill, border);
      frontierFill_.SetVertices(fill);
      frontierBorder_.SetPoints(border);
    }
    // Same depth bias as the FOV cone, against z-fighting at distance.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-2.0f, -4.0f);
    colorShader_.Use();
    colorShader_.SetMat4("uMVP", viewProj);
    frontierFill_.Draw();
    unlitShader_.Use();
    unlitShader_.SetMat4("uMVP", viewProj);
    unlitShader_.SetVec4("uColor", overlays.moveFrontierSubsequentLeg
                                       ? glm::vec4(1.0f, 0.9f, 0.15f, 1.0f)
                                       : glm::vec4(kSetupColor, 1.0f));
    frontierBorder_.DrawSegments();
    glDisable(GL_POLYGON_OFFSET_FILL);
  }
  // Zipline regions: where a walk -> ride -> walk plan can reach, in a second
  // (cyan) tint, with the usable line's cable highlighted.
  if (overlays.ziplineFrontiers && !overlays.ziplineFrontiers->empty()) {
    const glm::vec3 kZiplineColor(0.2f, 0.75f, 1.0f);
    const auto& frontiers = *overlays.ziplineFrontiers;
    std::vector<glm::vec4> key;
    for (const auto& fr : frontiers) {
      key.emplace_back(fr.field.minX, fr.field.minZ, fr.field.budget,
                       static_cast<float>(fr.zipline));
    }
    if (key != ziplineKey_) {
      ziplineKey_ = key;
      std::vector<ColorTriangleMesh::Vertex> fill;
      std::vector<glm::vec3> border;
      for (const auto& fr : frontiers) {
        if (fr.field.nx > 0) appendFrontier(fr.field, kZiplineColor, fill, border);
      }
      ziplineFill_.SetVertices(fill);
      ziplineBorder_.SetPoints(border);
    }
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-2.0f, -4.0f);
    colorShader_.Use();
    colorShader_.SetMat4("uMVP", viewProj);
    ziplineFill_.Draw();
    unlitShader_.Use();
    unlitShader_.SetMat4("uMVP", viewProj);
    unlitShader_.SetVec4("uColor", glm::vec4(kZiplineColor, 1.0f));
    ziplineBorder_.DrawSegments();
    glDisable(GL_POLYGON_OFFSET_FILL);
    for (const auto& fr : frontiers) {
      pathLine_.SetPoints(cablePoints(game.GetScene().ziplines[fr.zipline]));
      pathLine_.Draw();
      DrawHighlightOnSurface(fr.entry, glm::vec4(kZiplineColor, 1.0f));
    }
  }
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);

  // Enemy sighting memory: faint wireframe trail fading with age.
  if (fogActive) {
    constexpr float kSightingMaxAlpha = 0.35f;
    unlitShader_.Use();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const Unit& unit : game.GetScene().units) {
      if (unit.team == team) continue;
      for (const auto& s : game.Sightings(team, unit.id)) {
        const float life = 1.0f - s.ageRounds * tactics::constants::kSightingFadePerRound;
        if (life <= 0.0f) continue;
        DrawSighting(unlitShader_, pathLine_, viewProj, unit, s, game.GetScene().sidewalks,
                     life * kSightingMaxAlpha, terrain);
      }
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  // WEGO planning: both teams plan concurrently, so during the planning
  // phase every pane highlights its own team's living figures (dim white =
  // still needs a plan, green = plan set) -- this is squad-wide, not a
  // single actor.
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || unit.team != team) continue;
      if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
      const bool planned = unit.plan.type != tactics::PlannedActionType::None;
      const glm::vec4 color = planned ? glm::vec4(0.25f, 0.9f, 0.35f, 1.0f)
                                        : glm::vec4(1.0f, 1.0f, 1.0f, 0.6f);
      DrawHighlightOnSurface(unit.position, color);
    }
  }
  // Playbook indicator: magenta ring on every figure whose squad playbook
  // has any shoot reaction.
  for (const Unit& unit : game.GetScene().units) {
    const tactics::SquadPlaybook& pb = game.Playbook(unit.team);
    bool shoots = false;
    for (int m = 0; m < 2; ++m)
      for (int s = 0; s < 2; ++s) shoots |= tactics::ReactionShoots(pb.table[m][s]);
    if (!unit.alive || !shoots) continue;
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawHighlightOnSurface(unit.position, glm::vec4(0.85f, 0.1f, 0.85f, 1.0f));
  }
  // Selection/move-preview overlays belong to whichever pane the input
  // layer says is acting; callers pass them only for that pane.
  if (overlays.selectionHighlight) {
    DrawHighlightOnSurface(*overlays.selectionHighlight,
                  glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
  }
  // An executing round animates both teams' planned moves at once; each
  // pane highlights its own team's figures currently mid-move.
  if (game.Mode() == InputMode::Executing) {
    for (const Unit& unit : game.GetScene().units) {
      if (unit.team == team && game.IsUnitMoving(unit.id)) {
        DrawHighlightOnSurface(unit.position,
                      glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
      }
    }
  }
  // Move-path polylines carry terrain-height waypoints, but the straight
  // chord between two waypoints can still dip into the drawn surface (and
  // GLES lines get no polygon offset), so over terrain they ride slightly
  // above their sampled heights.
  const auto liftedPath = [&](const std::vector<glm::vec3>& points) {
    std::vector<glm::vec3> lifted = points;
    if (!terrain.Empty()) {
      constexpr float kPathLift = 0.2f;
      for (glm::vec3& p : lifted) p.y += kPathLift;
    }
    return lifted;
  };
  // A move path as walk -> ride -> walk: walked stretches in `walkColor`,
  // each zipline ride in the zipline tint.
  const auto drawMovePath = [&](const std::vector<glm::vec3>& path,
                                const std::vector<tactics::PathRide>& rides,
                                const glm::vec4& walkColor) {
    const glm::vec4 rideColor(0.2f, 0.75f, 1.0f, 1.0f);
    unlitShader_.SetMat4("uMVP", viewProj);
    const auto drawRange = [&](size_t first, size_t last, const glm::vec4& color) {
      if (last <= first) return;
      pathLine_.SetPoints(liftedPath(std::vector<glm::vec3>(path.begin() + first,
                                                            path.begin() + last + 1)));
      unlitShader_.SetVec4("uColor", color);
      pathLine_.Draw();
    };
    std::vector<tactics::PathRide> sorted = rides;
    std::sort(sorted.begin(), sorted.end(),
              [](const tactics::PathRide& a, const tactics::PathRide& b) {
                return a.segment < b.segment;
              });
    size_t begin = 0;
    for (const tactics::PathRide& ride : sorted) {
      const size_t seg = static_cast<size_t>(std::max(ride.segment, 0));
      if (seg + 1 >= path.size() || seg < begin) continue;
      drawRange(begin, seg, walkColor);
      drawRange(seg, seg + 1, rideColor);
      begin = seg + 1;
    }
    drawRange(begin, path.size() - 1, walkColor);
  };
  if (overlays.movePreviewPath && overlays.movePreviewPath->size() >= 2) {
    static const std::vector<tactics::PathRide> kNoRides;
    drawMovePath(*overlays.movePreviewPath,
                 overlays.movePreviewRides ? *overlays.movePreviewRides : kNoRides,
                 glm::vec4(1.0f, 0.85f, 0.2f, 1.0f));
    // Mark the final position with the same ring used for selection.
    DrawHighlightOnSurface(overlays.movePreviewPath->back(),
                  glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
  } else if (overlays.invalidHoverHighlight) {
    DrawHighlightOnSurface(*overlays.invalidHoverHighlight,
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
        drawMovePath(unit.plan.movePath, unit.plan.moveRides, glm::vec4(0.3f, 0.9f, 0.4f, 1.0f));
        // Wireframe stand-in at the destination, showing the planned final
        // facing (persists until the turn is committed).
        Unit ghost = unit;
        ghost.position = unit.plan.movePath.back();
        ghost.facingYaw = unit.plan.endFacingYaw;
        DrawUnitWireframe(unlitShader_, pathLine_, viewProj, ghost);
        // Chained legs for later rounds: yellow, with a marker at each leg
        // end so they read apart from the leg about to execute (green).
        const glm::vec4 yellow(1.0f, 0.85f, 0.2f, 1.0f);
        const bool chaining = game.Mode() == InputMode::AwaitingMoveDestination &&
                              game.SelectedUnitId() == unit.id;
        if (chaining || !unit.plan.queuedLegs.empty()) {
          DrawNumberedHighlight(unlitShader_, highlightRing_, cubeMesh_, viewProj, view,
                                OnSurface(unit.plan.movePath.back()), yellow, 1,
                                SurfaceNormal(unit.plan.movePath.back()));
        }
        int legNumber = 1;
        for (const auto& leg : unit.plan.queuedLegs) {
          ++legNumber;
          if (leg.size() < 2) continue;
          const size_t legIndex = static_cast<size_t>(legNumber - 2);
          static const std::vector<tactics::PathRide> kNoLegRides;
          drawMovePath(leg,
                       legIndex < unit.plan.queuedLegRides.size() ? unit.plan.queuedLegRides[legIndex]
                                                                  : kNoLegRides,
                       yellow);
          DrawNumberedHighlight(unlitShader_, highlightRing_, cubeMesh_, viewProj, view,
                                OnSurface(leg.back()), yellow, legNumber,
                                SurfaceNormal(leg.back()));
        }
      } else if (unit.plan.type == tactics::PlannedActionType::Shoot) {
        if (const Unit* shotTarget = game.FindUnit(unit.plan.shootTargetId)) {
          const std::vector<glm::vec3> shotLine = {unit.MuzzlePosition(), shotTarget->EyePosition()};
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
