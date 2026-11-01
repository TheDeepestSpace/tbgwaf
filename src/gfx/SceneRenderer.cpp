#include "gfx/SceneRenderer.h"

#include "gfx/Ragdoll.h"

#include "gfx/FigureRig.h"

#include <algorithm>
#include <functional>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

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
    // A later leg is drawn at 50% alpha; the first leg is solid green.
    overlays.moveFrontierSubsequentLeg = selected->plan.type == tactics::PlannedActionType::Move &&
                                         selected->plan.movePath.size() >= 2;
    if (game.MovePreviewValid()) {
      overlays.movePreviewPath = &game.MovePreviewPath();
    } else if (hoveredGroundPoint) {
      overlays.invalidHoverHighlight = hoveredGroundPoint;
    }
  }
  overlays.showShotCone = game.Mode() == tactics::InputMode::AwaitingShootTarget;
  if (overlays.showShotCone) {
    // Free-aim (issue #129): the aiming figure gets its 360-degree LOS
    // highlight, and a placed aim point shows as the "+" selector.
    overlays.aimShooter = selected;
    if (const auto& preview = game.GetAimPreview()) {
      overlays.aimMarker = preview->point;
    } else if (const auto& lock = game.GetLockPreview()) {
      // A staged lock-on shows the cone and "+" on the locked figure's
      // torso -- the exact point the burst will be aimed at, so the cone
      // footprint the shader paints on the figure is the real hit odds.
      if (const tactics::Unit* target = game.FindUnit(*lock)) {
        overlays.aimMarker =
            target->position + glm::vec3(0.0f, tactics::constants::kTorsoAimHeight, 0.0f);
      }
    }
    overlays.aimConeTarget = overlays.aimMarker ? overlays.aimMarker : hoveredGroundPoint;
  }
  return overlays;
}
namespace {

constexpr int kShadowMapSize = 2048;
// Large static meshes are split into this many cells per side (see
// LitTriangleMesh::SetChunkedMesh) so passes that see only part of the map
// -- each FOV depth map, the FOV mask, a zoomed-in camera -- skip the rest.
constexpr int kCullGridCells = 8;
// Deck/ramp slabs form one continuous ribbon but are submitted as separate
// shadow casters. A stronger-than-default slope bias keeps each shallow
// joint from shadowing the next slab while the deck still shadows geometry
// below its 0.45-unit thickness.
constexpr float kDeckShadowSlopeBias = 16.0f;
constexpr float kDeckShadowConstantBias = 64.0f;
const glm::vec3 kSetupColor(0.2f, 1.0f, 0.3f);
// The thermal HUD's "bright" green (Hud.cpp PushThermalHudStyle), used for
// move paths and reach frontiers in the AO view.
const glm::vec3 kThermalGreen(0.30f, 0.90f, 0.40f);
// Black-hot (light) view: the bright green washes out on the pale background,
// so overlays use a darker shade of it.
const glm::vec3 kThermalGreenDark(0.02f, 0.45f, 0.14f);
// Scene geometry base colors, shared by the opaque pass and the issue #136
// see-through pass so a faded block keeps its normal tint.
const glm::vec3 kObstacleColor(0.55f, 0.55f, 0.6f);
// Facade openings (issue #152): window glazing, door leafs, and the
// frame/sill/mullion trim extruded from (or lining a recess into) the walls.
const glm::vec3 kDeckColor(0.44f, 0.45f, 0.48f);

// Apparent temperatures are art-directed to the aerial black-hot reference,
// independent of visible-light paint colors. Figures are the exception: they
// keep their team color so sides stay readable (PR #178 review). Keep the
// shader IDs in sync.
enum class ThermalMaterial { Ground = 0, Concrete, Asphalt, Building, Metal, Figure };
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
precision highp int;
in vec3 vWorldPos;
in vec3 vWorldNormal;
in vec4 vLightSpacePos;
uniform vec4 uColor;
uniform vec3 uLightDir;  // Direction the light travels; surfaces face -uLightDir.
uniform vec3 uViewPos;
uniform sampler2D uShadowMap;
uniform int uDisableShadows;
uniform int uMonochrome;
uniform int uThermal;
uniform int uThermalBlackHot;  // 1 = hot reads dark (the review reference), 0 = white-hot.
uniform int uThermalMaterial;  // ThermalMaterial; only figures use uColor.
layout(location = 0) out vec4 FragColor;
// Thermal G-buffer normal. Alpha 1 = grayscale heat surface; alpha 0 = a
// surface drawn in its own color (figures), which the AO composite leaves
// chromatic.
layout(location = 1) out vec4 WorldNormal;

// Integer hash avoids sin()-dependent grain differences across GL drivers.
float ThermalHash(vec2 p) {
  uvec2 q = uvec2(ivec2(floor(p)));
  uint h = (q.x * 1664525u) ^ (q.y * 1013904223u);
  h = (h ^ (h >> 16u)) * 2246822519u;
  return float(h & 65535u) / 65535.0;
}

float ThermalNoise(vec2 p) {
  vec2 i = floor(p);
  vec2 f = fract(p);
  vec2 u = f * f * (3.0 - 2.0 * f);
  return mix(mix(ThermalHash(i), ThermalHash(i + vec2(1.0, 0.0)), u.x),
             mix(ThermalHash(i + vec2(0.0, 1.0)), ThermalHash(i + vec2(1.0, 1.0)), u.x), u.y);
}

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
  WorldNormal = vec4(normal * 0.5 + 0.5, 1.0);

  float diffuse = max(dot(normal, -uLightDir), 0.0);
  float shadow = ComputeShadow(normal);
  const float kAmbient = 0.35;
  float lit = kAmbient + (1.0 - shadow) * diffuse * 0.65;
  if (uThermal != 0) {
    // Pale clay-like surroundings from the AO reference, with subtle
    // material differences. Contact shading is applied in screen space
    // after the opaque pass.
    if (uThermalMaterial == 5) {
      // Figures keep their team color in both polarities so sides stay
      // readable; a small view-dependent falloff rounds limbs without
      // sun lighting.
      float shade = 0.82 + 0.18 * max(dot(normal, viewDir), 0.0);
      WorldNormal.a = 0.0;
      FragColor = vec4(uColor.rgb * shade, uColor.a);
      return;
    }
    float heat;
    float skyFacing = clamp(normal.y, 0.0, 1.0);
    // Project onto the dominant face: walls get 2D texture too, instead
    // of stretching ground noise into vertical stripes. Fine detail fades
    // out when smaller than a pixel so distant roofs do not shimmer.
    vec3 axis = abs(normal);
    vec2 p = axis.y >= max(axis.x, axis.z) ? vWorldPos.xz :
             (axis.x > axis.z ? vWorldPos.zy : vWorldPos.xy);
    float fineWeight = 1.0 - smoothstep(0.15, 0.6, length(fwidth(p)));
    float textureHeat = (ThermalNoise(p * 0.45) - 0.5) * 0.45 +
                        (ThermalNoise(p * 2.5) - 0.5) * 0.35 +
                        (ThermalNoise(p * 9.0) - 0.5) * 0.20 * fineWeight;
    if (uThermalMaterial == 3) {
      heat = mix(0.15, 0.018, skyFacing) +
             textureHeat * mix(0.035, 0.015, skyFacing);
      // Undersides retain a darker tone; never use visible-light shadows.
      heat += max(-normal.y, 0.0) * 0.15;
    } else if (uThermalMaterial == 2) {
      heat = 0.20 + textureHeat * 0.045;
    } else if (uThermalMaterial == 1) {
      heat = mix(0.17, 0.07, skyFacing) + textureHeat * 0.03;
    } else if (uThermalMaterial == 4) {
      heat = 0.13 + textureHeat * 0.025;
    } else {
      heat = 0.09 + textureHeat * 0.05;
    }
    heat += (1.0 - max(dot(normal, viewDir), 0.0)) * 0.018;
    float gray = uThermalBlackHot != 0 ? 1.0 - heat : heat;
    FragColor = vec4(vec3(clamp(gray, 0.0, 1.0)), uColor.a);
    return;
  }
  vec3 base = uColor.rgb;
  if (uMonochrome != 0) base = vec3(dot(base, vec3(0.299, 0.587, 0.114)));
  FragColor = vec4(base * lit, uColor.a);
}
)";

// Optics act on the world image before symbology is drawn. A small blur
// removes razor-sharp polygon edges; subtle detector grain follows it.
// No temporal noise: paused frames and regression captures stay stable.
const char* kThermalSensorVertexShaderSrc = R"(#version 300 es
out vec2 vUV;
void main() {
  vUV = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
  gl_Position = vec4(vUV * 2.0 - 1.0, 0.0, 1.0);
}
)";

const char* kThermalSensorFragmentShaderSrc = R"(#version 300 es
precision highp float;
precision highp int;
in vec2 vUV;
uniform sampler2D uScene;
out vec4 FragColor;
void main() {
  vec2 size = vec2(textureSize(uScene, 0));
  vec2 texel = 1.0 / size;
  // Per channel: the world is gray, but figures carry their team color.
  vec3 rgb = texture(uScene, vUV).rgb * 0.40;
  rgb += texture(uScene, vUV + vec2(texel.x, 0.0)).rgb * 0.15;
  rgb += texture(uScene, vUV - vec2(texel.x, 0.0)).rgb * 0.15;
  rgb += texture(uScene, vUV + vec2(0.0, texel.y)).rgb * 0.15;
  rgb += texture(uScene, vUV - vec2(0.0, texel.y)).rgb * 0.15;
  uvec2 p = uvec2(vUV * size);
  uint h = (p.x * 1664525u) ^ (p.y * 1013904223u);
  h = (h ^ (h >> 16u)) * 2246822519u;
  rgb += (float(h & 65535u) / 65535.0 - 0.5) * 0.012;
  FragColor = vec4(clamp(rgb, 0.0, 1.0), 1.0);
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
// a fragment is tinted when, for any of the pass's figures, it faces that
// figure's eye, lies inside its horizontal cone (analytic, so the cone's
// angular edges stay crisp), and its eye sightline passes a depth test
// against one of the figure's two eye-space depth maps. Flat tint, no
// attenuation or N.L -- this is a mask, not lighting. Everything else is
// discarded, which also keeps the stencil untouched so overlapping masks of
// a team with more figures than one pass takes don't stack (see RenderPane).
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
// Figures one mask pass tests (two depth maps each, on texture units 1..8).
// Unrolled in the shader below: GLSL ES 3.00 only indexes sampler arrays
// with constant expressions.
constexpr int kFovMasksPerPass = 4;
// A figure's cached depth maps are freed after this many RenderPane calls
// without being used (it died, stopped aiming, or the scene changed).
constexpr unsigned long long kFovMapIdlePanes = 120;

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
uniform int uCount;            // Figures in this pass, at most 4.
uniform vec3 uEye[4];
uniform vec3 uFacing[4];       // Unit forward, horizontal, normalized.
uniform float uCosHalfFov[4];
uniform mat4 uFovMatrix0[4];
uniform mat4 uFovMatrix1[4];
// highp is essential: GLSL ES defaults fragment samplers to lowp, and at
// least Mesa llvmpipe then returns depth reads at half-float precision
// (~11 bits), which after linearization is a 0.5-unit sawtooth at 15 units
// out -- far more than any bias can hide. (Diagnosed on this prototype;
// see docs/fov-shadow-map.md.)
uniform highp sampler2D uFovMap0;
uniform highp sampler2D uFovMap1;
uniform highp sampler2D uFovMap2;
uniform highp sampler2D uFovMap3;
uniform highp sampler2D uFovMap4;
uniform highp sampler2D uFovMap5;
uniform highp sampler2D uFovMap6;
uniform highp sampler2D uFovMap7;
uniform float uRange;
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
  // mainly by the normal offset applied in Tinted() (it pushes the sample
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

// Whether figure i (depth maps map0/map1) sees this fragment.
bool Tinted(int i, highp sampler2D map0, highp sampler2D map1, vec3 n) {
  vec3 toEye = uEye[i] - vWorldPos;
  float dist = length(toEye);
  if (dist > uRange) return false;
  vec3 dir = toEye / dist;
  // A face turned away from the eye is the back of something; the eye sees
  // its front, which is drawn (and tested) separately.
  float ndotl = dot(n, dir);
  if (ndotl <= 0.0) return false;
  // Analytic azimuth test: the fragment's bearing from the eye must be
  // within the half-FOV of the unit's facing.
  vec2 bearing = vec2(-dir.x, -dir.z);
  float bearingLen = length(bearing);
  if (bearingLen > 1e-5 && dot(bearing / bearingLen, uFacing[i].xz) < uCosHalfFov[i]) {
    return false;
  }
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
  float lit = SampleMap(map0, uFovMatrix0[i], p, slope);
  if (lit < 0.0) lit = SampleMap(map1, uFovMatrix1[i], p, slope);
  // Binary edge: a partially lit fragment would claim the stencil with a
  // faint alpha and block a teammate with a clearer view of the same pixel.
  return lit >= 0.5;
}

void main() {
  vec3 n = normalize(vWorldNormal);
  bool tinted = Tinted(0, uFovMap0, uFovMap1, n) ||
                (uCount > 1 && Tinted(1, uFovMap2, uFovMap3, n)) ||
                (uCount > 2 && Tinted(2, uFovMap4, uFovMap5, n)) ||
                (uCount > 3 && Tinted(3, uFovMap6, uFovMap7, n));
  if (!tinted) discard;
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

// DrawLitModel for a static world-space mesh, drawing only the chunks
// `visible` accepts, minus the triangle groups `skipGroup` rejects.
void DrawLitVisible(const Shader& shader, const LitTriangleMesh& mesh, const glm::mat4& viewProj,
                    const glm::mat4& lightSpaceMatrix, const glm::vec4& color,
                    const ChunkFilter& visible,
                    const std::function<bool(int group)>& skipGroup = nullptr) {
  shader.SetMat4("uModel", glm::mat4(1.0f));
  shader.SetMat4("uMVP", viewProj);
  shader.SetMat4("uLightSpaceMatrix", lightSpaceMatrix);
  shader.SetVec4("uColor", color);
  mesh.DrawVisible(visible, skipGroup);
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

struct StreetPropPart {
  glm::mat4 model{1.0f};
  size_t material = 0;
};

enum class StreetPropMaterial : size_t {
  Pole,
  TrafficHousing,
  TrafficRed,
  TrafficYellow,
  TrafficGreen,
  StopRed,
  YieldWhite,
  YieldRed,
  StreetName,
  LampHousing,
  LampGlow,
  Count,
};

// Every street fixture shares the building color so props read as part of
// the same urban fabric.
const glm::vec4& StreetPropColor(StreetPropMaterial) {
  static const glm::vec4 kColor(kObstacleColor, 1.0f);
  return kColor;
}

struct StreetPropParts {
  std::array<StreetPropPart, 8> values{};
  size_t count = 0;

  void Add(const glm::mat4& model, StreetPropMaterial material) {
    values[count++] = {model, static_cast<size_t>(material)};
  }
  const StreetPropPart* begin() const { return values.data(); }
  const StreetPropPart* end() const { return values.data() + count; }
};

// One geometry description feeds the lit, lighting-shadow, and shadow-map
// FOV passes, so fixtures cannot drift between those views of the scene.
StreetPropParts BuildStreetPropParts(const tactics::StreetProp& prop) {
  StreetPropParts parts;
  const auto base = [&](float yaw) {
    // StreetProp yaw follows the gameplay convention: (cos(yaw), 0,
    // sin(yaw)). A positive GLM rotation about +Y sends local +X toward -Z,
    // so use -Y to keep the rendered fixture aligned with its stored yaw.
    return glm::rotate(glm::translate(glm::mat4(1.0f), prop.position), yaw,
                       glm::vec3(0.0f, -1.0f, 0.0f));
  };
  auto box = [&](float yaw, float cx, float y0, float cz, float sx, float sy, float sz,
                 StreetPropMaterial material) {
    const glm::mat4 model =
        base(yaw) *
        glm::translate(glm::mat4(1.0f), glm::vec3(cx - sx * 0.5f, y0, cz - sz * 0.5f)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(sx, sy, sz));
    parts.Add(model, material);
  };
  // A square beam between two local XY points, used to approximate a curved
  // lamp arm with short straight segments.
  auto beam = [&](glm::vec2 a, glm::vec2 b, float thickness, StreetPropMaterial material) {
    const glm::vec2 delta = b - a;
    const float length = glm::length(delta);
    const float angle = std::atan2(delta.y, delta.x);
    const glm::mat4 model =
        base(prop.yaw) * glm::translate(glm::mat4(1.0f), glm::vec3(a.x, a.y, 0.0f)) *
        glm::rotate(glm::mat4(1.0f), angle, glm::vec3(0.0f, 0.0f, 1.0f)) *
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -thickness * 0.5f,
                                                  -thickness * 0.5f)) *
        glm::scale(glm::mat4(1.0f), glm::vec3(length, thickness, thickness));
    parts.Add(model, material);
  };

  switch (prop.kind) {
    case tactics::StreetPropKind::TrafficLight:
      box(prop.yaw, 0.0f, 0.0f, 0.0f, 0.14f, 3.1f, 0.14f, StreetPropMaterial::Pole);
      // +X is both the controlled approach's facing and the head/lamp face.
      box(prop.yaw, 0.24f, 2.35f, 0.0f, 0.30f, 0.80f, 0.26f,
          StreetPropMaterial::TrafficHousing);
      box(prop.yaw, 0.40f, 2.88f, 0.0f, 0.06f, 0.16f, 0.16f,
          StreetPropMaterial::TrafficRed);
      box(prop.yaw, 0.40f, 2.67f, 0.0f, 0.06f, 0.16f, 0.16f,
          StreetPropMaterial::TrafficYellow);
      box(prop.yaw, 0.40f, 2.46f, 0.0f, 0.06f, 0.16f, 0.16f,
          StreetPropMaterial::TrafficGreen);
      break;
    case tactics::StreetPropKind::StopSign:
      box(prop.yaw, 0.0f, 0.0f, 0.0f, 0.09f, 2.2f, 0.09f, StreetPropMaterial::Pole);
      box(prop.yaw, 0.07f, 1.62f, 0.0f, 0.05f, 0.52f, 0.52f, StreetPropMaterial::StopRed);
      break;
    case tactics::StreetPropKind::YieldSign:
      box(prop.yaw, 0.0f, 0.0f, 0.0f, 0.09f, 2.2f, 0.09f, StreetPropMaterial::Pole);
      box(prop.yaw, 0.07f, 1.62f, 0.0f, 0.05f, 0.52f, 0.52f,
          StreetPropMaterial::YieldWhite);
      box(prop.yaw, 0.10f, 1.74f, 0.0f, 0.03f, 0.28f, 0.28f,
          StreetPropMaterial::YieldRed);
      break;
    case tactics::StreetPropKind::StreetNameSign:
      box(prop.yaw, 0.0f, 0.0f, 0.0f, 0.08f, 2.5f, 0.08f, StreetPropMaterial::Pole);
      // Each blade follows its actual incident street; secondaryYaw is not
      // assumed perpendicular, so skewed junctions remain skewed here.
      box(prop.yaw, 0.0f, 2.28f, 0.0f, 0.80f, 0.14f, 0.04f,
          StreetPropMaterial::StreetName);
      box(prop.secondaryYaw, 0.0f, 2.10f, 0.0f, 0.80f, 0.14f, 0.04f,
          StreetPropMaterial::StreetName);
      break;
    case tactics::StreetPropKind::LampPost:
      // The nested obstacle remains the collision/LOS pole; this slightly
      // wider visible pole hides its plain obstacle-box rendering.
      box(prop.yaw, 0.0f, 0.0f, 0.0f, 0.18f, 5.40f, 0.18f, StreetPropMaterial::Pole);
      beam({0.0f, 5.20f}, {0.32f, 5.52f}, 0.16f, StreetPropMaterial::Pole);
      beam({0.32f, 5.52f}, {0.82f, 5.72f}, 0.16f, StreetPropMaterial::Pole);
      beam({0.82f, 5.72f}, {1.48f, 5.72f}, 0.16f, StreetPropMaterial::Pole);
      beam({1.48f, 5.72f}, {1.66f, 5.54f}, 0.16f, StreetPropMaterial::Pole);
      box(prop.yaw, 1.66f, 5.38f, 0.0f, 0.54f, 0.18f, 0.32f,
          StreetPropMaterial::LampHousing);
      box(prop.yaw, 1.66f, 5.34f, 0.0f, 0.38f, 0.05f, 0.24f,
          StreetPropMaterial::LampGlow);
      break;
  }
  return parts;
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

// A quad from corner `a` spanned by `right` and `up`, facing up-cross-right
// (the clockwise-outward winding the scene meshes use throughout).
void AppendQuad(glm::vec3 a, glm::vec3 right, glm::vec3 up, LitVertices* vertices,
                std::vector<GLuint>* indices) {
  const glm::vec3 n = glm::normalize(glm::cross(up, right));
  AppendTriangle(a, a + right, a + right + up, n, vertices, indices);
  AppendTriangle(a, a + right + up, a + up, n, vertices, indices);
}

// One wall face of a prism (edge a->b, heights lo..hi) minus the `holes`
// rectangles (u0, y0, u1, y1 along the edge): recessed facade openings
// (doors, factory windows) show their niche instead of a flat wall. The
// face is split into vertical strips at every hole edge; each strip emits
// the wall pieces between the holes stacked inside it.
void AppendWallFaceWithHoles(glm::vec2 a, glm::vec2 b, float lo, float hi,
                             const std::vector<glm::vec4>& holes, LitVertices* vertices,
                             std::vector<GLuint>* indices) {
  const float len = glm::distance(a, b);
  if (len < 1e-4f) return;
  const glm::vec2 dir = (b - a) / len;
  const glm::vec3 right(dir.x, 0.0f, dir.y);
  constexpr float kEps = 1e-3f;
  std::vector<float> cuts{0.0f, len};
  for (const glm::vec4& h : holes) {
    cuts.push_back(glm::clamp(h.x, 0.0f, len));
    cuts.push_back(glm::clamp(h.z, 0.0f, len));
  }
  std::sort(cuts.begin(), cuts.end());
  const auto rect = [&](float u0, float u1, float y0, float y1) {
    if (u1 - u0 < kEps || y1 - y0 < kEps) return;
    const glm::vec2 p = a + dir * u0;
    AppendQuad(glm::vec3(p.x, y0, p.y), right * (u1 - u0), glm::vec3(0.0f, y1 - y0, 0.0f),
               vertices, indices);
  };
  for (size_t i = 0; i + 1 < cuts.size(); ++i) {
    const float u0 = cuts[i], u1 = cuts[i + 1];
    if (u1 - u0 < kEps) continue;
    std::vector<glm::vec2> spans;  // y-ranges the holes cover in this strip.
    for (const glm::vec4& h : holes) {
      if (h.x <= u0 + kEps && h.z >= u1 - kEps) {
        spans.emplace_back(glm::clamp(h.y, lo, hi), glm::clamp(h.w, lo, hi));
      }
    }
    std::sort(spans.begin(), spans.end(),
              [](const glm::vec2& p, const glm::vec2& q) { return p.x < q.x; });
    float y = lo;
    for (const glm::vec2& s : spans) {
      rect(u0, u1, y, s.x);
      y = std::max(y, s.y);
    }
    rect(u0, u1, y, hi);
  }
}

// `holesPerEdge` (optional, indexed like the footprint's edges) cuts the
// listed opening rectangles out of the wall faces; see
// AppendWallFaceWithHoles. Box-bounds obstacles never carry openings.
void AppendPolygonPrism(const tactics::Obstacle& obstacle, LitVertices* vertices,
                        std::vector<GLuint>* indices,
                        const std::vector<std::vector<glm::vec4>>* holesPerEdge = nullptr) {
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
    if (holesPerEdge && i < holesPerEdge->size() && !(*holesPerEdge)[i].empty() &&
        !obstacle.footprint.empty()) {
      AppendWallFaceWithHoles(a, b, lo, hi, (*holesPerEdge)[i], vertices, indices);
      continue;
    }
    const glm::vec2 edge = glm::normalize(b - a);
    const glm::vec3 normal(edge.y, 0.0f, -edge.x);
    const glm::vec3 a0(a.x, lo, a.y), a1(a.x, hi, a.y);
    const glm::vec3 b0(b.x, lo, b.y), b1(b.x, hi, b.y);
    triangle(a0, b0, b1, normal);
    triangle(a0, b1, a1, normal);
  }
}

void BuildPolygonPrism(const tactics::Obstacle& obstacle, LitTriangleMesh* mesh,
                       const std::vector<std::vector<glm::vec4>>* holesPerEdge = nullptr) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  AppendPolygonPrism(obstacle, &vertices, &indices, holesPerEdge);
  mesh->SetMesh(vertices, indices);
}

// --- Facade openings as pane extrusions (issue #152 follow-up) ---------------
//
// Windows carry a rectangular frame extruded out of the wall (plus an
// exterior sill on some residential buildings); doors and factory-style
// commercial windows are pushed into the building body instead: the wall
// face gets a real hole (see AppendWallFaceWithHoles) lined with reveal
// faces, the leaf/glazing sits at the back of the niche, and factory
// windows add a grid of mullion bars. Offices keep flatter, thinner frames
// over their full-floor ribbon glazing. All of it stays visual-only: the
// gameplay Obstacle prism is untouched.

// The opening's frame on its wall: `base` is the world point at (u=0, y=0),
// `right` runs along the edge, `normal` points out of the wall.
struct OpeningBasis {
  glm::vec3 base{0.0f};
  glm::vec3 right{0.0f};
  glm::vec3 normal{0.0f};
  bool valid = false;
};

OpeningBasis OpeningBasisFor(const tactics::Scene& scene,
                             const tactics::FacadeOpening& opening) {
  OpeningBasis basis;
  if (opening.obstacle < 0 ||
      opening.obstacle >= static_cast<int>(scene.obstacles.size())) {
    return basis;
  }
  const std::vector<glm::vec2> fp =
      tactics::ObstacleFootprint(scene.obstacles[opening.obstacle]);
  if (fp.size() < 3 || opening.wallEdge < 0 ||
      opening.wallEdge >= static_cast<int>(fp.size()) || opening.u1 <= opening.u0 ||
      opening.y1 <= opening.y0) {
    return basis;
  }
  const glm::vec2 a = fp[opening.wallEdge];
  const glm::vec2 b = fp[(opening.wallEdge + 1) % fp.size()];
  const float len = glm::length(b - a);
  if (len < 1e-4f) return basis;
  const glm::vec2 dir = (b - a) / len;
  basis.base = glm::vec3(a.x, 0.0f, a.y);
  basis.right = glm::vec3(dir.x, 0.0f, dir.y);
  basis.normal = glm::vec3(dir.y, 0.0f, -dir.x);  // Outward, as AppendPolygonPrism.
  basis.valid = true;
  return basis;
}

// How the opening is built, decided from its kind and building archetype.
struct PaneStyle {
  bool recessed = false;  // Pushed into the wall instead of extruded from it.
  float depth = 0.0f;     // Extrusion out of / recess into the wall.
  float bar = 0.0f;       // Frame bar width (0: no frame, doors).
  bool sill = false;      // Exterior sill under the window.
  int gridU = 1, gridV = 1;  // Mullion grid panes (factory windows).
};

PaneStyle PaneStyleFor(const tactics::Scene& scene, const tactics::FacadeOpening& opening) {
  if (opening.kind == tactics::FacadeOpening::Kind::Door) return {true, 0.22f, 0.0f, false, 1, 1};
  tactics::BuildingKind kind = tactics::BuildingKind::Residential;
  if (opening.building >= 0 && opening.building < static_cast<int>(scene.buildings.size())) {
    kind = scene.buildings[opening.building].kind;
  }
  // Stable per-building draw for the touches that only some buildings get
  // (residential sills, a factory window's mullion grid).
  uint32_t h = static_cast<uint32_t>(opening.building) * 2654435761u;
  h ^= h >> 16;
  switch (kind) {
    case tactics::BuildingKind::Residential:
      return {false, 0.09f, 0.08f, (h % 100u) < 55u, 1, 1};
    case tactics::BuildingKind::MixedUse:
    case tactics::BuildingKind::Civic:
      return {false, 0.09f, 0.08f, false, 1, 1};
    case tactics::BuildingKind::Office:  // Flatter, thinner ribbon framing.
    case tactics::BuildingKind::Garage:
      return {false, 0.03f, 0.05f, false, 1, 1};
    case tactics::BuildingKind::Commercial: {  // Factory-style grid panes.
      constexpr int kGrids[4][2] = {{3, 3}, {4, 4}, {3, 4}, {4, 3}};
      const int* grid = kGrids[h % 4u];
      return {true, 0.16f, 0.06f, false, grid[0], grid[1]};
    }
  }
  return {false, 0.09f, 0.08f, false, 1, 1};
}

bool OpeningRecessed(const tactics::Scene& scene, const tactics::FacadeOpening& opening) {
  return PaneStyleFor(scene, opening).recessed;
}

// A box in the opening's wall frame: u along the edge, y up, d along the
// outward normal (negative reaches into the wall). `skipBack` leaves out the
// -normal face when it rests flush against the wall.
void AppendOpeningBox(const OpeningBasis& f, float u0, float u1, float y0, float y1, float d0,
                      float d1, bool skipBack, LitVertices* vertices,
                      std::vector<GLuint>* indices) {
  if (u1 - u0 < 1e-4f || y1 - y0 < 1e-4f || d1 - d0 < 1e-4f) return;
  const auto at = [&](float u, float y, float d) {
    return f.base + f.right * u + glm::vec3(0.0f, y, 0.0f) + f.normal * d;
  };
  const glm::vec3 U = f.right * (u1 - u0);
  const glm::vec3 Y(0.0f, y1 - y0, 0.0f);
  const glm::vec3 D = f.normal * (d1 - d0);
  AppendQuad(at(u0, y0, d1), U, Y, vertices, indices);                    // Front.
  if (!skipBack) AppendQuad(at(u1, y0, d0), -U, Y, vertices, indices);    // Back.
  AppendQuad(at(u0, y0, d0), D, Y, vertices, indices);                    // Left.
  AppendQuad(at(u1, y0, d1), -D, Y, vertices, indices);                   // Right.
  AppendQuad(at(u0, y1, d1), U, -D, vertices, indices);                   // Top.
  AppendQuad(at(u0, y0, d0), U, D, vertices, indices);                    // Bottom.
}

// One facade opening's geometry for `part`; see the section comment above
// for the shapes per kind.
void AppendFacadeOpening(const tactics::Scene& scene, const tactics::FacadeOpening& opening,
                         LitVertices* vertices, std::vector<GLuint>* indices) {
  const OpeningBasis f = OpeningBasisFor(scene, opening);
  if (!f.valid) return;
  const PaneStyle style = PaneStyleFor(scene, opening);
  const float u0 = opening.u0, u1 = opening.u1, y0 = opening.y0, y1 = opening.y1;
  const auto quad = [&](float qu0, float qu1, float qy0, float qy1, float d) {
    const glm::vec3 a = f.base + f.right * qu0 + glm::vec3(0.0f, qy0, 0.0f) + f.normal * d;
    AppendQuad(a, f.right * (qu1 - qu0), glm::vec3(0.0f, qy1 - qy0, 0.0f), vertices, indices);
  };
  if (style.recessed) {
    const float d = style.depth;
    {
      // Reveal faces lining the niche, wound to face into it.
      const glm::vec3 Y(0.0f, y1 - y0, 0.0f);
      const glm::vec3 U = f.right * (u1 - u0);
      const glm::vec3 D = f.normal * d;
      AppendQuad(f.base + f.right * u0 + glm::vec3(0.0f, y0, 0.0f), -D, Y, vertices, indices);
      AppendQuad(f.base + f.right * u1 + glm::vec3(0.0f, y0, 0.0f) - D, D, Y, vertices, indices);
      AppendQuad(f.base + f.right * u0 + glm::vec3(0.0f, y1, 0.0f) - D, U, D, vertices, indices);
      AppendQuad(f.base + f.right * u0 + glm::vec3(0.0f, y0, 0.0f), U, -D, vertices, indices);
      // Factory windows: mullion bars gridding the glazing at the niche back.
      const float bar = style.bar;
      for (int i = 1; i < style.gridU; ++i) {
        const float u = u0 + (u1 - u0) * static_cast<float>(i) / style.gridU;
        AppendOpeningBox(f, u - 0.5f * bar, u + 0.5f * bar, y0, y1, -d, -d + 0.10f, true,
                         vertices, indices);
      }
      for (int i = 1; i < style.gridV; ++i) {
        const float y = y0 + (y1 - y0) * static_cast<float>(i) / style.gridV;
        AppendOpeningBox(f, u0, u1, y - 0.5f * bar, y + 0.5f * bar, -d, -d + 0.10f, true,
                         vertices, indices);
      }
    }
    quad(u0, u1, y0, y1, -d);  // Back of the niche.
    return;
  }
  // Extruded frame: four bars ringing the rect, emitted as one ring (its
  // front, outer and inner faces) rather than four boxes, whose butted
  // faces would only be hidden inside it. Frames are most of a city's
  // facade triangles, so the 24 instead of 40 per window add up.
  const float bar = std::min({style.bar, 0.25f * (y1 - y0), 0.25f * (u1 - u0)});
  if (bar > 1e-3f) {
    const float d = style.depth;
    const auto at = [&](float u, float y, float depth) {
      return f.base + f.right * u + glm::vec3(0.0f, y, 0.0f) + f.normal * depth;
    };
    const glm::vec3 D = f.normal * d;
    const glm::vec3 U = f.right * (u1 - u0), innerU = f.right * (u1 - u0 - 2.0f * bar);
    const glm::vec3 Y(0.0f, y1 - y0, 0.0f), innerY(0.0f, y1 - y0 - 2.0f * bar, 0.0f);
    const glm::vec3 barU = f.right * bar, barY(0.0f, bar, 0.0f);
    AppendQuad(at(u0, y0, d), U, barY, vertices, indices);                    // Front: bottom bar,
    AppendQuad(at(u0, y1 - bar, d), U, barY, vertices, indices);              // top bar,
    AppendQuad(at(u0, y0 + bar, d), barU, innerY, vertices, indices);         // left bar,
    AppendQuad(at(u1 - bar, y0 + bar, d), barU, innerY, vertices, indices);   // right bar.
    AppendQuad(at(u0, y0, 0.0f), U, D, vertices, indices);                    // Outer: bottom,
    AppendQuad(at(u0, y1, d), U, -D, vertices, indices);                      // top,
    AppendQuad(at(u0, y0, 0.0f), D, Y, vertices, indices);                    // left,
    AppendQuad(at(u1, y0, d), -D, Y, vertices, indices);                      // right.
    AppendQuad(at(u0 + bar, y0 + bar, d), innerU, -D, vertices, indices);     // Inner: bottom,
    AppendQuad(at(u0 + bar, y1 - bar, 0.0f), innerU, D, vertices, indices);   // top,
    AppendQuad(at(u0 + bar, y0 + bar, d), -D, innerY, vertices, indices);     // left,
    AppendQuad(at(u1 - bar, y0 + bar, 0.0f), D, innerY, vertices, indices);   // right.
    if (style.sill) {
      AppendOpeningBox(f, u0 - 0.08f, u1 + 0.08f, y0 - 0.08f, y0, 0.0f, 0.14f, true, vertices,
                       indices);
    }
  }
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

void AppendMassingDetail(const tactics::MassingDetail& detail, LitVertices* vertices,
                         std::vector<GLuint>* indices) {
  if (detail.top.size() == detail.footprint.size() && detail.top.size() >= 3 &&
      detail.depth > 0.0f) {
    AppendSurfacePatch(detail.top, detail.depth, vertices, indices);
    return;
  }
  if (detail.footprint.size() < 3 || detail.y1 <= detail.y0) return;
  tactics::Obstacle prism;
  prism.footprint = detail.footprint;
  prism.bounds.min.y = detail.y0;
  prism.bounds.max.y = detail.y1;
  AppendPolygonPrism(prism, vertices, indices);
}

void BuildMassingDetail(const tactics::MassingDetail& detail, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  AppendMassingDetail(detail, &vertices, &indices);
  mesh->SetMesh(vertices, indices);
}

// A terrain stair as one concrete wedge: the sloped flight and the flat
// landing on top, each extruded down to the stair's base, sides outward.
void AppendTerrainStair(const tactics::TerrainStair& stair, LitVertices* vertices,
                        std::vector<GLuint>* indices) {
  const float baseY = stair.base - 0.05f;
  for (const std::vector<glm::vec3>* top : {&stair.flight, &stair.landing}) {
    if (top->size() < 3) continue;
    glm::vec3 center(0.0f);
    for (const glm::vec3& v : *top) center += v;
    center /= static_cast<float>(top->size());
    glm::vec3 topNormal = glm::cross((*top)[1] - (*top)[0], (*top)[2] - (*top)[0]);
    if (glm::length(topNormal) < 1e-7f) continue;
    topNormal = glm::normalize(topNormal);
    if (topNormal.y < 0.0f) topNormal = -topNormal;
    for (size_t i = 1; i + 1 < top->size(); ++i) {
      AppendTriangle((*top)[0], (*top)[i], (*top)[i + 1], topNormal, vertices, indices);
    }
    for (size_t i = 0; i < top->size(); ++i) {
      const glm::vec3 a = (*top)[i], b = (*top)[(i + 1) % top->size()];
      const glm::vec3 a0(a.x, baseY, a.z), b0(b.x, baseY, b.z);
      glm::vec3 normal = glm::cross(b - a, a0 - a);
      if (glm::length(normal) < 1e-7f) continue;
      normal = glm::normalize(normal);
      if (glm::dot(normal, center - a) > 0.0f) normal = -normal;  // Outward.
      AppendTriangle(a, b0, b, normal, vertices, indices);
      AppendTriangle(a, a0, b0, normal, vertices, indices);
    }
  }
}

void BuildTerrainStairMesh(const std::vector<tactics::TerrainStair>& stairs,
                           LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  for (const tactics::TerrainStair& stair : stairs) AppendTerrainStair(stair, &vertices, &indices);
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

// Unit cube transformed exactly like CubeMesh. Used to batch street
// fixtures into the single static mesh consumed by the FOV depth/receiver
// passes.
void AppendBoxModel(const glm::mat4& model, LitVertices* vertices,
                    std::vector<GLuint>* indices) {
  const glm::mat3 normalMatrix = glm::transpose(glm::inverse(glm::mat3(model)));
  const auto p = [&](float x, float y, float z) {
    return glm::vec3(model * glm::vec4(x, y, z, 1.0f));
  };
  const auto n = [&](float x, float y, float z) {
    return glm::normalize(normalMatrix * glm::vec3(x, y, z));
  };
  auto quad = [&](glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 d, glm::vec3 normal) {
    AppendTriangle(a, b, c, normal, vertices, indices);
    AppendTriangle(a, c, d, normal, vertices, indices);
  };
  quad(p(0, 1, 0), p(0, 1, 1), p(1, 1, 1), p(1, 1, 0), n(0, 1, 0));
  quad(p(0, 0, 0), p(1, 0, 0), p(1, 0, 1), p(0, 0, 1), n(0, -1, 0));
  quad(p(0, 0, 0), p(0, 1, 0), p(1, 1, 0), p(1, 0, 0), n(0, 0, -1));
  quad(p(0, 0, 1), p(1, 0, 1), p(1, 1, 1), p(0, 1, 1), n(0, 0, 1));
  quad(p(0, 0, 0), p(0, 0, 1), p(0, 1, 1), p(0, 1, 0), n(-1, 0, 0));
  quad(p(1, 0, 0), p(1, 1, 0), p(1, 1, 1), p(1, 0, 1), n(1, 0, 0));
}

// Terrain overlays (FOV cones over hills) use the same triangles as the
// ground, clipped to each footprint triangle.
void AppendTerrainOverlay(const tactics::HeightField& terrain,
                          const std::array<glm::vec3, 3>& footprint,
                          std::vector<glm::vec3>* points, float lift = 0.08f);

// Streets and sidewalks are planar patches at their own heights (a city's
// levels are planar too), batched into one mesh per material.
void BuildPavementMesh(const std::vector<tactics::RoadSurface>& patches, float thickness,
                       LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  for (const auto& patch : patches) {
    AppendSurfacePatch(patch.vertices, thickness, &vertices, &indices);
  }
  mesh->SetChunkedMesh(vertices, indices, kCullGridCells);
}

void BuildStreetFixtureMeshes(
    const std::vector<tactics::Obstacle>& obstacles,
    const std::vector<tactics::StreetProp>& props, LitTriangleMesh* furnitureMesh,
    LitTriangleMesh* depthMesh,
    std::array<LitTriangleMesh, static_cast<size_t>(StreetPropMaterial::Count)>* materialMeshes) {
  LitVertices furnitureVertices;
  std::vector<GLuint> furnitureIndices;
  for (const tactics::Obstacle& obstacle : obstacles) {
    if (obstacle.kind != tactics::ObstacleKind::StreetFurniture) continue;
    AppendBox(obstacle.bounds, &furnitureVertices, &furnitureIndices);
  }
  furnitureMesh->SetChunkedMesh(furnitureVertices, furnitureIndices, kCullGridCells);

  LitVertices depthVertices;
  std::vector<GLuint> depthIndices;
  std::array<LitVertices, static_cast<size_t>(StreetPropMaterial::Count)> vertices;
  std::array<std::vector<GLuint>, static_cast<size_t>(StreetPropMaterial::Count)> indices;
  for (const tactics::StreetProp& prop : props) {
    for (const StreetPropPart& part : BuildStreetPropParts(prop)) {
      AppendBoxModel(part.model, &depthVertices, &depthIndices);
      AppendBoxModel(part.model, &vertices[part.material], &indices[part.material]);
    }
  }
  depthMesh->SetChunkedMesh(depthVertices, depthIndices, kCullGridCells);
  for (size_t i = 0; i < materialMeshes->size(); ++i) {
    (*materialMeshes)[i].SetChunkedMesh(vertices[i], indices[i], kCullGridCells);
  }
}

// Everything static the shadow-map FOV prototype (issue #110) casts from
// and receives on, apart from the ground itself: obstacles (walls + roofs),
// decks/ramps as 0.45-thick slabs (matching the gameplay LOS slab in
// LineOfSightClear and the lit pass), street fixtures, sidewalk slabs and
// patches, and roads. Built once per scene so each per-unit pass is one draw.
// Facade openings are left out: they only receive the tint (drawn from the
// facade mesh, see DrawFovMasks). As casters the recessed parts sit
// inside the solid wall and the frames stand a few centimetres proud of it,
// yet they were most of a city's triangles in every depth map.
void BuildFovSceneMesh(const tactics::Scene& scene, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  for (const tactics::Obstacle& obstacle : scene.obstacles) {
    if (obstacle.navOnly) continue;  // A cliff edge: the terrain mesh is the caster.
    AppendPolygonPrism(obstacle, &vertices, &indices);
  }
  for (const tactics::StreetProp& prop : scene.props) {
    for (const StreetPropPart& part : BuildStreetPropParts(prop)) {
      AppendBoxModel(part.model, &vertices, &indices);
    }
  }
  // Parapets, roof furniture and awnings occlude like the gameplay LOS.
  for (const tactics::MassingDetail& d : scene.massing) {
    AppendMassingDetail(d, &vertices, &indices);
  }
  for (const tactics::WalkSurface& surface : scene.walkSurfaces) {
    AppendSurfacePatch(surface.vertices, 0.45f, &vertices, &indices);
  }
  for (const tactics::TerrainStair& stair : scene.terrainStairs) {
    AppendTerrainStair(stair, &vertices, &indices);
  }
  for (const AABB& slab : scene.sidewalks) AppendBox(slab, &vertices, &indices);
  for (const tactics::RoadSurface& sidewalk : scene.sidewalkSurfaces) {
    AppendSurfacePatch(sidewalk.vertices, 0.10f, &vertices, &indices);
  }
  for (const tactics::RoadSurface& road : scene.roads) {
    AppendSurfacePatch(road.vertices, 0.0f, &vertices, &indices);
  }
  mesh->SetChunkedMesh(vertices, indices, kCullGridCells);
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
  for (const tactics::StreetProp& prop : scene.props) {
    mix(&prop.kind, sizeof(prop.kind));
    mix(&prop.position, sizeof(prop.position));
    mix(&prop.yaw, sizeof(prop.yaw));
    mix(&prop.secondaryYaw, sizeof(prop.secondaryYaw));
  }
  for (const tactics::MassingDetail& d : scene.massing) {
    mix(d.footprint.data(), d.footprint.size() * sizeof(glm::vec2));
    mix(&d.y0, sizeof(d.y0));
    mix(&d.y1, sizeof(d.y1));
    mix(d.top.data(), d.top.size() * sizeof(glm::vec3));
    mix(&d.depth, sizeof(d.depth));
  }
  for (const tactics::FacadeOpening& o : scene.facadeOpenings) {
    mix(&o.obstacle, sizeof(o.obstacle));
    mix(&o.wallEdge, sizeof(o.wallEdge));
    mix(&o.u0, sizeof(o.u0));
    mix(&o.u1, sizeof(o.u1));
    mix(&o.y0, sizeof(o.y0));
    mix(&o.y1, sizeof(o.y1));
  }
  for (const tactics::WalkSurface& s : scene.walkSurfaces) mixVec3s(s.vertices);
  for (const tactics::TerrainStair& s : scene.terrainStairs) {
    mixVec3s(s.flight);
    mixVec3s(s.landing);
  }
  mix(scene.sidewalks.data(), scene.sidewalks.size() * sizeof(AABB));
  for (const tactics::RoadSurface& s : scene.sidewalkSurfaces) mixVec3s(s.vertices);
  for (const tactics::RoadSurface& s : scene.roads) mixVec3s(s.vertices);
  return h;
}

// Content hash of Scene::massing, so the massing meshes rebuild only when
// the dressing itself changes.
unsigned long long MassingFingerprint(const tactics::Scene& scene) {
  unsigned long long h = 1469598103934665603ULL ^ scene.massing.size();
  auto mix = [&](const void* data, size_t bytes) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
      h ^= p[i];
      h *= 1099511628211ULL;
    }
  };
  for (const tactics::MassingDetail& d : scene.massing) {
    mix(&d.kind, sizeof(d.kind));
    mix(&d.y0, sizeof(d.y0));
    mix(&d.y1, sizeof(d.y1));
    mix(&d.building, sizeof(d.building));
    mix(d.footprint.data(), d.footprint.size() * sizeof(glm::vec2));
    mix(d.top.data(), d.top.size() * sizeof(glm::vec3));
    mix(&d.depth, sizeof(d.depth));
  }
  return h;
}

// Content hash of Scene::facadeOpenings, so the decal meshes rebuild only
// when the openings themselves change.
unsigned long long FacadeFingerprint(const tactics::Scene& scene) {
  unsigned long long h = 1469598103934665603ULL ^ scene.facadeOpenings.size();
  auto mix = [&](const void* data, size_t bytes) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
      h ^= p[i];
      h *= 1099511628211ULL;
    }
  };
  for (const tactics::FacadeOpening& o : scene.facadeOpenings) {
    mix(&o.kind, sizeof(o.kind));
    mix(&o.building, sizeof(o.building));  // The pane style reads the building's archetype.
    mix(&o.obstacle, sizeof(o.obstacle));
    mix(&o.wallEdge, sizeof(o.wallEdge));
    mix(&o.u0, sizeof(o.u0));
    mix(&o.u1, sizeof(o.u1));
    mix(&o.y0, sizeof(o.y0));
    mix(&o.y1, sizeof(o.y1));
  }
  return h;
}

// All facade opening geometry merged into a single mesh, its triangles
// grouped by building so camera-faded buildings' openings can be skipped.
void BuildFacadeMesh(const tactics::Scene& scene, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  std::vector<int> groups;
  for (const tactics::FacadeOpening& opening : scene.facadeOpenings) {
    AppendFacadeOpening(scene, opening, &vertices, &indices);
    groups.resize(indices.size() / 3, opening.building);
  }
  mesh->SetChunkedMesh(vertices, indices, kCullGridCells, &groups);
}

// Opening rectangles each obstacle's wall faces must cut out (recessed
// doors and factory windows), keyed by obstacle index, per footprint edge:
// (u0, y0, u1, y1) along the edge, as AppendWallFaceWithHoles reads them.
std::unordered_map<int, std::vector<std::vector<glm::vec4>>> RecessHolesByObstacle(
    const tactics::Scene& scene) {
  std::unordered_map<int, std::vector<std::vector<glm::vec4>>> holes;
  for (const tactics::FacadeOpening& o : scene.facadeOpenings) {
    if (!OpeningRecessed(scene, o)) continue;
    if (o.obstacle < 0 || o.obstacle >= static_cast<int>(scene.obstacles.size())) continue;
    const tactics::Obstacle& wall = scene.obstacles[o.obstacle];
    if (wall.footprint.empty() || o.wallEdge < 0 ||
        o.wallEdge >= static_cast<int>(wall.footprint.size())) {
      continue;
    }
    auto& perEdge = holes[o.obstacle];
    perEdge.resize(wall.footprint.size());
    perEdge[o.wallEdge].emplace_back(o.u0, o.y0, o.u1, o.y1);
  }
  return holes;
}

// Content hash of Scene::walkSurfaces (vertices and neighbour links, which
// SharedEdges reads), so the batched deck mesh rebuilds only on scene change.
unsigned long long DeckFingerprint(const tactics::Scene& scene) {
  unsigned long long h = 1469598103934665603ULL ^ scene.walkSurfaces.size();
  auto mix = [&](const void* data, size_t bytes) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
      h ^= p[i];
      h *= 1099511628211ULL;
    }
  };
  for (const tactics::WalkSurface& s : scene.walkSurfaces) {
    mix(s.vertices.data(), s.vertices.size() * sizeof(glm::vec3));
    mix(s.neighbors.data(), s.neighbors.size() * sizeof(int));
  }
  return h;
}

// Every walk surface as one static mesh (0.45-thick slabs, shared chain
// edges hidden): decks, ramps, roof slabs, stoops and rear platforms are
// scene-static, so re-tessellating them per frame (and per pane) is wasted
// work once facade stoops multiply their count (issue #152).
void BuildDeckMesh(const tactics::Scene& scene, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  std::vector<int> groups;
  for (size_t i = 0; i < scene.walkSurfaces.size(); ++i) {
    const std::vector<char> hidden = SharedEdges(scene.walkSurfaces, scene.walkSurfaces[i]);
    AppendSurfacePatch(scene.walkSurfaces[i].vertices, 0.45f, &vertices, &indices, &hidden);
    groups.resize(indices.size() / 3, static_cast<int>(i));
  }
  mesh->SetChunkedMesh(vertices, indices, kCullGridCells, &groups);
}

// One prism per massing detail, merged into a single mesh, grouped by
// building like the facade mesh.
void BuildMassingMesh(const tactics::Scene& scene, LitTriangleMesh* mesh) {
  LitVertices vertices;
  std::vector<GLuint> indices;
  std::vector<int> groups;
  for (const tactics::MassingDetail& d : scene.massing) {
    AppendMassingDetail(d, &vertices, &indices);
    groups.resize(indices.size() / 3, static_cast<int>(&d - scene.massing.data()));
  }
  mesh->SetChunkedMesh(vertices, indices, kCullGridCells, &groups);
}

// For each walk surface, the building whose roof slab it is (all its
// vertices on one building obstacle's top), or -1.
std::vector<int> RoofBuildings(const tactics::Scene& scene) {
  std::vector<int> buildings(scene.walkSurfaces.size(), -1);
  for (size_t i = 0; i < scene.walkSurfaces.size(); ++i) {
    const tactics::WalkSurface& surface = scene.walkSurfaces[i];
    for (const tactics::Obstacle& o : scene.obstacles) {
      if (o.building < 0) continue;
      bool inside = !surface.vertices.empty();
      for (const glm::vec3& v : surface.vertices) {
        inside &= v.x >= o.bounds.min.x - 0.05f && v.x <= o.bounds.max.x + 0.05f &&
                  v.z >= o.bounds.min.z - 0.05f && v.z <= o.bounds.max.z + 0.05f &&
                  std::fabs(v.y - o.bounds.max.y) < 0.1f;
      }
      if (inside) {
        buildings[i] = o.building;
        break;
      }
    }
  }
  return buildings;
}

// Content hash of what BuildObstacleMeshes reads from Scene::obstacles.
unsigned long long ObstacleFingerprint(const tactics::Scene& scene) {
  unsigned long long h = 1469598103934665603ULL ^ scene.obstacles.size();
  auto mix = [&](const void* data, size_t bytes) {
    const unsigned char* p = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < bytes; ++i) {
      h ^= p[i];
      h *= 1099511628211ULL;
    }
  };
  for (const tactics::Obstacle& o : scene.obstacles) {
    mix(&o.bounds, sizeof(o.bounds));
    mix(&o.kind, sizeof(o.kind));
    mix(&o.navOnly, sizeof(o.navOnly));
    mix(&o.building, sizeof(o.building));
    mix(o.footprint.data(), o.footprint.size() * sizeof(glm::vec2));
  }
  return h;
}

// Every drawn obstacle (not nav-only cliff edges or street furniture, which
// has its own batch) as one static mesh, grouped by obstacle index so the
// camera-faded ones can be skipped: `drawn` with the recessed facade
// openings cut into its walls, `caster` solid for the shadow map.
void BuildObstacleMeshes(const tactics::Scene& scene,
                         const std::unordered_map<int, std::vector<std::vector<glm::vec4>>>& holes,
                         LitTriangleMesh* drawn, LitTriangleMesh* caster) {
  LitVertices drawnVertices, casterVertices;
  std::vector<GLuint> drawnIndices, casterIndices;
  std::vector<int> groups;
  for (size_t i = 0; i < scene.obstacles.size(); ++i) {
    const tactics::Obstacle& obstacle = scene.obstacles[i];
    if (obstacle.navOnly || obstacle.kind == tactics::ObstacleKind::StreetFurniture) continue;
    if (obstacle.footprint.empty()) {
      AppendBox(obstacle.bounds, &drawnVertices, &drawnIndices);
      AppendBox(obstacle.bounds, &casterVertices, &casterIndices);
    } else {
      const auto obstacleHoles = holes.find(static_cast<int>(i));
      AppendPolygonPrism(obstacle, &drawnVertices, &drawnIndices,
                         obstacleHoles == holes.end() ? nullptr : &obstacleHoles->second);
      AppendPolygonPrism(obstacle, &casterVertices, &casterIndices);
    }
    groups.resize(drawnIndices.size() / 3, static_cast<int>(i));
  }
  drawn->SetChunkedMesh(drawnVertices, drawnIndices, kCullGridCells, &groups);
  caster->SetChunkedMesh(casterVertices, casterIndices, kCullGridCells);
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

// The neutral flag at rest / dropped (a carried one is part of its carrier's
// figure). Visible to both teams regardless of fog: it is a neutral objective.
bool FlagPlantedOnGround(const GameLogic& game) {
  return game.Flag().enabled && game.Flag().carrierId < 0;
}

void DrawPlantedFlag(const Shader& shader, const CubeMesh& cube, const SphereMesh& sphere,
                     const glm::mat4& viewProj, const glm::mat4& lightSpaceMatrix,
                     const GameLogic& game) {
  if (!FlagPlantedOnGround(game)) return;
  for (const FigurePart& part :
       BuildPlantedFlag(game.Flag().position, game.Flag().dropElapsed)) {
    if (part.primitive == FigurePrimitive::Rounded) {
      DrawLitModel(shader, sphere, viewProj, lightSpaceMatrix, part.model, part.color);
    } else {
      DrawLitModel(shader, cube, viewProj, lightSpaceMatrix, part.model, part.color);
    }
  }
}

void DrawPlantedFlagDepth(const Shader& shader, const CubeMesh& cube, const SphereMesh& sphere,
                          const glm::mat4& lightSpaceMatrix, const GameLogic& game) {
  if (!FlagPlantedOnGround(game)) return;
  for (const FigurePart& part :
       BuildPlantedFlag(game.Flag().position, game.Flag().dropElapsed)) {
    shader.SetMat4("uLightMVP", lightSpaceMatrix * part.model);
    if (part.primitive == FigurePrimitive::Rounded) {
      sphere.Draw();
    } else {
      cube.Draw();
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
                       const Unit& unit, const glm::vec4& color) {
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
  ghost.turnLag = 0.0f;
  ghost.walkPhase = s.walkPhase;
  ghost.walkBlend = s.walkBlend;
  ghost.idleElapsed = s.idleElapsed;
  ghost.alive = s.alive;
  ghost.knockdownElapsed = s.knockdownElapsed;
  ghost.knockdownAxis = s.knockdownAxis;
  // Ghosts are frozen snapshots: drop every live one-shot animation overlay.
  ghost.shootElapsed = -1.0f;
  ghost.rideTravel = -1.0f;
  ghost.grabElapsed = -1.0f;
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

// Tessellate flat pads and gentle grades coarsely, retaining the full grid
// at cliffs. The maximum geometric error is 1 cm (below the pavement lift).
// Terrain, pavement and FOV overlays all use the same tile decomposition.
constexpr int kTerrainTileCells = 8;
bool MergeableTerrainTile(const tactics::HeightField& hf, int x0, int z0, int x1, int z1) {
  const float a = hf.At(x0, z0), b = hf.At(x1, z0), c = hf.At(x0, z1), d = hf.At(x1, z1);
  for (int z = z0; z <= z1; ++z) {
    for (int x = x0; x <= x1; ++x) {
      const float u = static_cast<float>(x - x0) / (x1 - x0);
      const float v = static_cast<float>(z - z0) / (z1 - z0);
      const float height = u >= v ? a + (b - a) * u + (d - b) * v
                                 : a + (d - c) * u + (c - a) * v;
      if (std::fabs(hf.At(x, z) - height) > 0.01f) return false;
    }
  }
  return true;
}

template <typename Emit>
void TerrainPatches(const tactics::HeightField& hf, int x0, int z0, int x1, int z1, Emit emit) {
  if ((x1 - x0 == 1 && z1 - z0 == 1) || MergeableTerrainTile(hf, x0, z0, x1, z1)) {
    emit(x0, z0, x1, z1);
    return;
  }
  if (x1 - x0 >= z1 - z0) {
    const int mid = (x0 + x1) / 2;
    TerrainPatches(hf, x0, z0, mid, z1, emit);
    TerrainPatches(hf, mid, z0, x1, z1, emit);
  } else {
    const int mid = (z0 + z1) / 2;
    TerrainPatches(hf, x0, z0, x1, mid, emit);
    TerrainPatches(hf, x0, mid, x1, z1, emit);
  }
}

// A flat-topped convex prism from `bottom` up to `top`, like a building:
// the top face and one vertical quad per edge, normals outward.
void AppendPrism(const std::vector<glm::vec2>& polygon, float bottom, float top, bool topFace,
                 LitVertices* vertices, std::vector<GLuint>* indices) {
  if (polygon.size() < 3) return;
  glm::vec2 center(0.0f);
  for (glm::vec2 p : polygon) center += p;
  center /= static_cast<float>(polygon.size());
  if (topFace) {
    const glm::vec3 a(polygon[0].x, top, polygon[0].y);
    for (size_t i = 1; i + 1 < polygon.size(); ++i) {
      AppendTriangle(a, {polygon[i].x, top, polygon[i].y},
                     {polygon[i + 1].x, top, polygon[i + 1].y}, {0.0f, 1.0f, 0.0f}, vertices,
                     indices);
    }
  }
  if (top - bottom < 1e-4f) return;
  for (size_t i = 0; i < polygon.size(); ++i) {
    const glm::vec2 a = polygon[i], b = polygon[(i + 1) % polygon.size()];
    if (glm::distance(a, b) < 1e-4f) continue;
    glm::vec2 n(-(b - a).y, (b - a).x);
    if (glm::dot(n, center - a) > 0.0f) n = -n;  // Outward.
    const glm::vec3 normal(glm::normalize(glm::vec3(n.x, 0.0f, n.y)));
    const glm::vec3 a0(a.x, bottom, a.y), a1(a.x, top, a.y), b0(b.x, bottom, b.y), b1(b.x, top, b.y);
    AppendTriangle(a0, b0, b1, normal, vertices, indices);
    AppendTriangle(a0, b1, a1, normal, vertices, indices);
  }
}

// Urban levels are deliberately geometric, like the buildings: each tier is
// a convex polygon prism (the map square split by every step line, minus
// the ramp corridors), flat on top and sheer on its sides down to the base
// plane. Where a lower tier meets a higher one, the higher prism's exposed
// side is the cliff; the rest of that side is hidden inside the lower
// prism. The sampled heightfield stays the movement model and never shows.
void BuildUrbanTierMeshes(const tactics::HeightField& hf,
                          const std::vector<tactics::TerrainWall>& walls,
                          const std::vector<std::vector<glm::vec2>>& cutouts,
                          LitTriangleMesh* base, LitTriangleMesh* raised) {
  LitVertices baseVertices, raisedVertices;
  std::vector<GLuint> baseIndices, raisedIndices;
  const float minX = hf.minX, minZ = hf.minZ;
  const float maxX = hf.minX + (hf.nx - 1) * hf.step;
  const float maxZ = hf.minZ + (hf.nz - 1) * hf.step;
  const std::vector<glm::vec2> square = {{minX, minZ}, {maxX, minZ}, {maxX, maxZ}, {minX, maxZ}};

  // The base plane and its skirt, as the flat city's ground slab.
  constexpr float kSkirt = 0.5f;
  AppendPrism(square, -kSkirt, 0.0f, /*topFace=*/true, &baseVertices, &baseIndices);

  // Tier polygons: the map square split by each step line, minus the ramp
  // corridors (shared with the tests, so what they check is what is drawn).
  for (const tactics::TerrainTier& tier : tactics::TerrainTiers(walls, cutouts, square)) {
    if (tier.height < 0.01f) continue;  // The base plane already shows here.
    AppendPrism(tier.polygon, 0.0f, tier.height, /*topFace=*/true, &raisedVertices,
                &raisedIndices);
  }
  base->SetChunkedMesh(baseVertices, baseIndices, kCullGridCells);
  raised->SetChunkedMesh(raisedVertices, raisedIndices, kCullGridCells);
}

// Triangulated terrain heightfield with central-difference normals, plus a
// border skirt dropping below the lowest sample so the map edge reads as a
// solid block instead of a paper-thin sheet. A city with levels is drawn
// geometrically instead (BuildUrbanTierMeshes); `raised` is then the
// concrete tiers and `base` the ground plane. Hills use the heightfield.
void BuildTerrainMesh(const tactics::HeightField& hf,
                      const std::vector<tactics::TerrainWall>& walls,
                      const std::vector<std::vector<glm::vec2>>& cutouts, LitTriangleMesh* base,
                      LitTriangleMesh* raised) {
  if (!walls.empty()) {
    BuildUrbanTierMeshes(hf, walls, cutouts, base, raised);
    return;
  }
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
  const auto quad = [&](int x0, int z0, int x1, int z1) {
    const GLuint a = static_cast<GLuint>(z0 * nx + x0), b = z0 * nx + x1;
    const GLuint c = z1 * nx + x0, d = z1 * nx + x1;
    indices.insert(indices.end(), {a, b, d, a, d, c});
  };
  for (int z = 0; z + 1 < nz; z += kTerrainTileCells) {
    for (int x = 0; x + 1 < nx; x += kTerrainTileCells) {
      const int x1 = std::min(x + kTerrainTileCells, nx - 1);
      const int z1 = std::min(z + kTerrainTileCells, nz - 1);
      TerrainPatches(hf, x, z, x1, z1, quad);
    }
  }
  skirtBottom -= 0.5f;
  // Skirt: one quad per border edge, with an outward horizontal normal.
  auto skirtQuad = [&](int ix0, int iz0, int ix1, int iz1, const glm::vec3& normal) {
    const GLuint start = static_cast<GLuint>(vertices.size());
    const glm::vec3 a = vertices[iz0 * nx + ix0].pos;
    const glm::vec3 b = vertices[iz1 * nx + ix1].pos;
    vertices.push_back({a, normal});
    vertices.push_back({b, normal});
    vertices.push_back({glm::vec3(b.x, skirtBottom, b.z), normal});
    vertices.push_back({glm::vec3(a.x, skirtBottom, a.z), normal});
    indices.insert(indices.end(), {start, start + 1, start + 2, start, start + 2, start + 3});
  };
  for (int ix = 0; ix + 1 < nx; ++ix) {
    skirtQuad(ix, 0, ix + 1, 0, glm::vec3(0.0f, 0.0f, -1.0f));
    skirtQuad(ix, nz - 1, ix + 1, nz - 1, glm::vec3(0.0f, 0.0f, 1.0f));
  }
  for (int iz = 0; iz + 1 < nz; ++iz) {
    skirtQuad(0, iz, 0, iz + 1, glm::vec3(-1.0f, 0.0f, 0.0f));
    skirtQuad(nx - 1, iz, nx - 1, iz + 1, glm::vec3(1.0f, 0.0f, 0.0f));
  }
  base->SetChunkedMesh(vertices, indices, kCullGridCells);
  raised->SetMesh({}, {});
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

// Clip the rendered terrain tiles to an overlay footprint, retaining
// their interpolated heights. Sampling only the footprint's vertices (even
// with radial subdivisions) cuts across ridges, and bilinear HeightAt does
// not match the rendered surface within a non-planar heightfield cell.
void AppendTerrainOverlay(const tactics::HeightField& terrain,
                          const std::array<glm::vec3, 3>& footprint,
                          std::vector<glm::vec3>* points, float lift) {
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
    return glm::vec3(terrain.minX + ix * terrain.step, terrain.At(ix, iz) + lift,
                     terrain.minZ + iz * terrain.step);
  };
  const auto clipTriangle = [&](const std::array<glm::vec3, 3>& triangle) {
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
  };
  const auto quad = [&](int x0, int z0, int x1, int z1) {
    const glm::vec3 a = vertex(x0, z0), b = vertex(x1, z0);
    const glm::vec3 c = vertex(x0, z1), d = vertex(x1, z1);
    clipTriangle({a, b, d});
    clipTriangle({a, d, c});
  };
  const int tile = kTerrainTileCells;
  for (int iz = (z0 / tile) * tile; iz <= z1; iz += tile) {
    const int endZ = std::min(iz + tile, terrain.nz - 1);
    // Restrict each tile row to the triangle's X interval. Cone slices can
    // have large bounding boxes while covering very little of each row.
    const float rowMinZ = terrain.minZ + iz * terrain.step;
    const float rowMaxZ = terrain.minZ + endZ * terrain.step;
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
    for (int ix = (x0 / tile) * tile; ix <= x1; ix += tile) {
      const int endX = std::min(ix + tile, terrain.nx - 1);
      TerrainPatches(terrain, ix, iz, endX, endZ, quad);
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
    // Knee-high slabs (facade stoops and rear-door platforms, issue #152)
    // cast no meaningful eye-level shadow; keeping them out of the occluder
    // set keeps the per-ray cost at the deck/ramp/roof count, not the far
    // larger stoop count. Gameplay LOS stays exact in Raycast.
    float clearance = 0.0f;
    for (const glm::vec3& v : surface.vertices) {
      clearance = std::max(clearance, v.y - terrain.HeightAt(v.x, v.z));
    }
    if (clearance < 0.6f) continue;
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
          std::remainder(std::atan2(p.y - eye.z, p.x - eye.x) - unit.VisualYaw(), kTwoPi);
      for (const float nudged : {delta - kCornerEpsilon, delta, delta + kCornerEpsilon}) {
        if (nudged >= -halfFovRad && nudged <= halfFovRad) offsets.push_back(nudged);
      }
    }
  }
  for (const DeckOccluder& deck : decks) {
    for (const glm::vec2& p : deck.footprint.footprint) {
      const float delta =
          std::remainder(std::atan2(p.y - eye.z, p.x - eye.x) - unit.VisualYaw(), kTwoPi);
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
    const float angle = unit.VisualYaw() + offset;
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
        y = std::max(y, terrain.Empty() ? sidewalk.vertices.front().y
                                        : terrain.MeshHeightAt(x, z) + 0.12f);
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
                 const Unit& unit, const std::vector<glm::vec3>& points,
                 const glm::vec3* colorOverride = nullptr, float alpha = 0.15f) {
  const float kFovAlpha = alpha;
  const glm::vec3 eye = unit.EyePosition();
  const float range = tactics::constants::kFovConeVisualRange;
  const glm::vec3 baseColor = colorOverride ? *colorOverride
                              : unit.team == Team::Blue ? glm::vec3(0.2f, 0.45f, 0.95f)
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
                  const std::vector<AABB>& sidewalks, float mapHalfExtent,
                  const glm::vec3* aimPoint = nullptr) {
  constexpr int kSegments = 40;
  constexpr int kRings = 12;  // Along the axis, so the alpha fade interpolates smoothly.
  constexpr float kSurfaceAlpha = 0.18f;
  constexpr float kFootprintAlpha = 0.7f;
  constexpr float kTwoPi = 6.28318530717958647692f;
  // Keep the setup preview identical to the trajectory scatter used when
  // the action executes.  In particular, the sniper and Deagle must not
  // inherit the AR's 3-degree cone.
  const float tanHalf =
      std::tan(glm::radians(tactics::StatsOf(unit.weapon).scatterHalfAngleDegrees));
  // Free-aim (issue #129): the cone leaves the muzzle of the figure as it
  // will stand when it fires -- turned toward the aim point -- and its axis
  // may pitch up/down (sky or elevated aim).
  Unit aimed = unit;
  if (aimPoint) {
    const glm::vec2 flat(aimPoint->x - unit.position.x, aimPoint->z - unit.position.z);
    if (glm::length(flat) > 1e-4f) aimed.facingYaw = std::atan2(flat.y, flat.x);
  }
  const glm::vec3 apex = aimed.MuzzlePosition();
  glm::vec3 axis = aimed.FacingDirection();
  if (aimPoint && glm::distance(*aimPoint, apex) > 1e-4f) {
    axis = glm::normalize(*aimPoint - apex);
  }
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
  // Basis perpendicular to the (possibly pitched) axis.
  glm::vec3 right = glm::cross(glm::vec3(0.0f, 1.0f, 0.0f), axis);
  right = glm::length(right) < 1e-5f ? glm::vec3(1.0f, 0.0f, 0.0f) : glm::normalize(right);
  const glm::vec3 up = glm::normalize(glm::cross(axis, right));
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

// Acid-green overlays for the free-aim flow (issue #129).
const glm::vec3 kAimColor(0.55f, 1.0f, 0.1f);

// The "+" aim selector: a 3D plus sign (three thin axis-aligned bars) so a
// point placed on a surface or in the sky reads from any camera angle.
void DrawAimMarker(const Shader& colorShader, ColorTriangleMesh& mesh,
                   const glm::mat4& viewProj, const glm::vec3& point) {
  constexpr float kArm = 0.55f;
  constexpr float kThick = 0.06f;
  constexpr float kAlpha = 0.95f;
  std::vector<ColorTriangleMesh::Vertex> vertices;
  const glm::vec4 color(kAimColor, kAlpha);
  const auto appendBox = [&](const glm::vec3& halfExtents) {
    const glm::vec3 mn = point - halfExtents;
    const glm::vec3 mx = point + halfExtents;
    const glm::vec3 corners[8] = {
        {mn.x, mn.y, mn.z}, {mx.x, mn.y, mn.z}, {mx.x, mx.y, mn.z}, {mn.x, mx.y, mn.z},
        {mn.x, mn.y, mx.z}, {mx.x, mn.y, mx.z}, {mx.x, mx.y, mx.z}, {mn.x, mx.y, mx.z}};
    static const int faces[6][4] = {{0, 1, 2, 3}, {5, 4, 7, 6}, {4, 0, 3, 7},
                                    {1, 5, 6, 2}, {3, 2, 6, 7}, {4, 5, 1, 0}};
    for (const auto& face : faces) {
      const int quad[6] = {face[0], face[1], face[2], face[0], face[2], face[3]};
      for (int index : quad) vertices.push_back({corners[index], color});
    }
  };
  appendBox(glm::vec3(kArm, kThick, kThick));
  appendBox(glm::vec3(kThick, kArm, kThick));
  appendBox(glm::vec3(kThick, kThick, kArm));
  colorShader.Use();
  mesh.SetVertices(vertices);
  colorShader.SetMat4("uMVP", viewProj);
  mesh.Draw();
}

// Issue #136: everything `team`'s pane must keep visible this frame -- the
// team's own living figures, the movement-frontier overlay, movement
// indicators (preview/planned paths, destination ghosts) and remembered-
// sighting ghosts/arrows -- as world-space probe points. Any obstacle or
// deck slab crossing a camera->probe segment is drawn see-through.
std::vector<glm::vec3> CollectSeeThroughProbes(const GameLogic& game, Team team, bool fogActive,
                                               const PaneOverlays& overlays) {
  // The omniscient spectator keeps both teams' figures and plans visible.
  const bool allTeams = overlays.spectatorFov;
  // Probes hover slightly above their surface so grazing contact with the
  // ground/deck they sit on never reads as occlusion.
  constexpr float kSurfaceLift = 0.3f;
  constexpr float kGhostLift = 0.9f;  // Mid-torso of a ghost/figure wireframe.
  std::vector<glm::vec3> probes;
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || (!allTeams && unit.team != team)) continue;
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
      if (!unit.alive || (!allTeams && unit.team != team)) continue;
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
        if (game.GhostAlpha(s.at) <= 0.0f) continue;
        probes.push_back(s.position + glm::vec3(0.0f, kGhostLift, 0.0f));  // Ghost + arrow.
      }
    }
  }
  return probes;
}

// Issue #256: identity of the occluder geometry feeding the see-through fade
// (everything the raycasts below depend on besides camera and probes).
// Content-hashed (FNV-1a), not address-keyed: successive scenes can reuse the
// same storage addresses.
uint64_t FadeGeometryKey(const tactics::Scene& scene) {
  uint64_t hash = 1469598103934665603ull;
  const auto mix = [&hash](const void* data, size_t size) {
    const unsigned char* bytes = static_cast<const unsigned char*>(data);
    for (size_t i = 0; i < size; ++i) {
      hash ^= bytes[i];
      hash *= 1099511628211ull;
    }
  };
  for (const tactics::Obstacle& o : scene.obstacles) {
    mix(&o.bounds, sizeof(o.bounds));
    mix(o.footprint.data(), o.footprint.size() * sizeof(glm::vec2));
    mix(&o.building, sizeof(o.building));
    const int kind = static_cast<int>(o.kind);
    mix(&kind, sizeof(kind));
    const char navOnly = o.navOnly ? 1 : 0;
    mix(&navOnly, sizeof(navOnly));
  }
  for (const tactics::WalkSurface& s : scene.walkSurfaces) {
    mix(s.vertices.data(), s.vertices.size() * sizeof(glm::vec3));
    const char overpass = s.overpass ? 1 : 0;
    mix(&overpass, sizeof(overpass));
  }
  for (const tactics::MassingDetail& d : scene.massing) {
    mix(d.footprint.data(), d.footprint.size() * sizeof(glm::vec2));
    mix(&d.y0, sizeof(d.y0));
    mix(&d.y1, sizeof(d.y1));
    mix(&d.building, sizeof(d.building));
    const int kind = static_cast<int>(d.kind);
    mix(&kind, sizeof(kind));
  }
  const size_t buildings = scene.buildings.size();
  mix(&buildings, sizeof(buildings));
  return hash;
}

}  // namespace

bool IsUnitVisibleForRender(const Unit& unit, Team viewingTeam, bool fogActive,
                            const TeamVisibility& visibility, const tactics::Scene& scene) {
  if (unit.team == viewingTeam) return true;
  if (!fogActive) return true;
  if (RagdollActive(unit)) {
    // The body has left the spot the unit was shot at: see it only if some
    // part of it is in the viewing team's FOV with clear line of sight.
    const RagdollPose pose = SampleRagdoll(unit);
    for (const RagdollJoint joint : {kRagHead, kRagPelvis, kRagFootL, kRagFootR}) {
      if (IsPointVisibleToTeam(viewingTeam, pose[joint], scene.units, scene.obstacles,
                               scene.walkSurfaces, scene.ground)) {
        return true;
      }
    }
    return false;
  }
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
  if (!thermalSensorShader_.Compile(kThermalSensorVertexShaderSrc,
                                    kThermalSensorFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the thermal sensor shader\n");
    return false;
  }
  if (!thermalOcclusion_.Init()) {
    std::fprintf(stderr, "Failed to compile the thermal ambient occlusion shaders\n");
    return false;
  }
  cubeMesh_.Init();
  sphereMesh_.Init();
  if (!trees_.Init()) {
    std::fprintf(stderr, "Failed to compile the tree shaders\n");
    return false;
  }
  frontierFill_.Init();
  frontierBorder_.Init();
  ziplineFill_.Init();
  ziplineBorder_.Init();
  pathLine_.Init();
  fovConeMesh_.Init();
  shotConeMesh_.Init();
  aimOverlayMesh_.Init();
  terrainMesh_.Init();
  terrainRaisedMesh_.Init();
  terrainRoadMesh_.Init();
  terrainSidewalkMesh_.Init();
  terrainStairMesh_.Init();
  geometryMesh_.Init();
  streetFurnitureMesh_.Init();
  streetPropDepthMesh_.Init();
  for (LitTriangleMesh& mesh : streetPropMeshes_) mesh.Init();
  fovSceneMesh_.Init();
  massingMesh_.Init();
  facadeMesh_.Init();
  deckMesh_.Init();
  obstacleMesh_.Init();
  obstacleCasterMesh_.Init();
  highlightRing_.Init();
  highlightRing_.SetPoints(BuildRingPoints());

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
  aimOverlayMesh_.Destroy();
  terrainFovCache_.clear();
  fovKeyObstacles_.clear();
  fovKeySidewalks_.clear();
  fovKeyMapHalfExtent_ = 0.0f;
  terrainMesh_.Destroy();
  terrainRaisedMesh_.Destroy();
  terrainRoadMesh_.Destroy();
  terrainSidewalkMesh_.Destroy();
  terrainStairMesh_.Destroy();
  pavementKey_ = 0;
  geometryMesh_.Destroy();
  streetFurnitureMesh_.Destroy();
  streetPropDepthMesh_.Destroy();
  for (LitTriangleMesh& mesh : streetPropMeshes_) mesh.Destroy();
  streetFurnitureKey_.clear();
  streetPropKey_.clear();
  fovSceneMesh_.Destroy();
  fovSceneKey_ = 0;
  massingMesh_.Destroy();
  massingKey_ = 0;
  facadeMesh_.Destroy();
  facadeRecessHoles_.clear();
  facadeKey_ = 0;
  deckMesh_.Destroy();
  obstacleMesh_.Destroy();
  obstacleCasterMesh_.Destroy();
  obstacleKey_ = 0;
  surfaceBuilding_.clear();
  surfaceBuildingKey_ = 0;
  deckKey_ = 0;
  terrainKey_ = tactics::HeightField{};
  highlightRing_.Destroy();
  sphereMesh_.Destroy();
  trees_.Destroy();
  cubeMesh_.Destroy();
  if (shadowDepthTex_) glDeleteTextures(1, &shadowDepthTex_);
  if (shadowFbo_) glDeleteFramebuffers(1, &shadowFbo_);
  shadowDepthTex_ = 0;
  shadowFbo_ = 0;
  for (FovMapSlot& slot : fovMaps_) DestroyFovMaps(&slot);
  fovMaps_.clear();
  if (thermalSceneTex_) glDeleteTextures(1, &thermalSceneTex_);
  if (thermalSensorVao_) glDeleteVertexArrays(1, &thermalSensorVao_);
  thermalSceneTex_ = thermalSensorVao_ = 0;
  thermalSceneWidth_ = thermalSceneHeight_ = 0;
  thermalOcclusion_.Destroy();
}

void SceneRenderer::ApplyThermalSensor(int x, int y, int width, int height) {
  // Copy only this pane, including on non-default caller framebuffers. Its
  // depth/stencil stay in place for the overlays that follow. Clamp sampling
  // to the pane edges so the other team's image can never bleed into it.
  glActiveTexture(GL_TEXTURE0);
  if (!thermalSceneTex_) {
    glGenTextures(1, &thermalSceneTex_);
    glGenVertexArrays(1, &thermalSensorVao_);
    glBindTexture(GL_TEXTURE_2D, thermalSceneTex_);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  }
  glBindTexture(GL_TEXTURE_2D, thermalSceneTex_);
  if (width != thermalSceneWidth_ || height != thermalSceneHeight_) {
    // RGB, not RGBA: CopyTexSubImage2D may only copy into a texture whose
    // components the read buffer has, and the WebGL2 canvas has no alpha.
    // An RGBA copy fails there with GL_INVALID_OPERATION and the sensor
    // pass then shows an uninitialized (black) image. Only .r is sampled.
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB,
                 GL_UNSIGNED_BYTE, nullptr);
    thermalSceneWidth_ = width;
    thermalSceneHeight_ = height;
  }
  glCopyTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, x, y, width, height);
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  thermalSensorShader_.Use();
  thermalSensorShader_.SetInt("uScene", 0);
  glBindVertexArray(thermalSensorVao_);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  RecordDraw(3, 1);
  glBindVertexArray(0);
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);
}

void SceneRenderer::DestroyFovMaps(FovMapSlot* slot) {
  if (slot->tex[0]) glDeleteTextures(2, slot->tex);
  if (slot->fbo[0]) glDeleteFramebuffers(2, slot->fbo);
  slot->tex[0] = slot->tex[1] = 0;
  slot->fbo[0] = slot->fbo[1] = 0;
}

int SceneRenderer::AcquireFovMaps(const Unit& unit, int sector, float facingYaw, bool drawTerrain,
                                  float mapHalfExtent) {
  const glm::vec3 eye = unit.EyePosition();
  int index = -1;
  for (size_t i = 0; i < fovMaps_.size(); ++i) {
    if (fovMaps_[i].unitId == unit.id && fovMaps_[i].sector == sector) index = static_cast<int>(i);
  }
  if (index < 0) {
    // A new pair of maps (16 MB of depth), allocated on first use.
    FovMapSlot slot;
    slot.unitId = unit.id;
    slot.sector = sector;
    glGenFramebuffers(2, slot.fbo);
    glGenTextures(2, slot.tex);
    bool complete = true;
    for (int half = 0; half < 2; ++half) {
      glBindTexture(GL_TEXTURE_2D, slot.tex[half]);
      glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kFovMapWidth, kFovMapHeight, 0,
                   GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
      glBindFramebuffer(GL_FRAMEBUFFER, slot.fbo[half]);
      glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, slot.tex[half],
                             0);
      const GLenum noColorBuffer = GL_NONE;
      glDrawBuffers(1, &noColorBuffer);
      glReadBuffer(GL_NONE);
      complete &= glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    }
    glBindTexture(GL_TEXTURE_2D, 0);
    if (!complete) {
      std::fprintf(stderr, "FOV depth map framebuffer incomplete\n");
      DestroyFovMaps(&slot);
      return -1;
    }
    fovMaps_.push_back(slot);
    index = static_cast<int>(fovMaps_.size()) - 1;
  }
  FovMapSlot& slot = fovMaps_[index];
  slot.lastUsed = fovMapClock_;
  if (slot.generation == fovMapGeneration_ && slot.eye == eye && slot.yaw == facingYaw &&
      slot.drawTerrain == drawTerrain && slot.mapHalfExtent == mapHalfExtent) {
    return index;  // Still current: nothing to render.
  }
  slot.eye = eye;
  slot.yaw = facingYaw;
  slot.drawTerrain = drawTerrain;
  slot.mapHalfExtent = mapHalfExtent;
  slot.generation = fovMapGeneration_;

  // Far plane: the cone's visual range, which already exceeds any map's
  // diagonal; the depth comparison is done in linearized depth so the
  // far-plane choice only affects precision, not correctness.
  const float far = tactics::constants::kFovConeVisualRange;
  const float aspect = std::tan(glm::radians(kFovMapHalfAzimuthDegrees)) /
                       std::tan(glm::radians(kFovMapHalfElevationDegrees));
  const glm::mat4 proj = glm::perspective(glm::radians(2.0f * kFovMapHalfElevationDegrees),
                                          aspect, kFovMapNear, far);
  for (int half = 0; half < 2; ++half) {
    const float yaw = facingYaw + glm::radians(kFovMapHalfSplitDegrees) * (half == 0 ? -1.0f : 1.0f);
    const glm::vec3 dir(std::cos(yaw), 0.0f, std::sin(yaw));
    slot.matrix[half] = proj * glm::lookAt(eye, eye + dir, glm::vec3(0.0f, 1.0f, 0.0f));
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
  // Each map sees only the wedge of the map in front of the eye, so the
  // chunked static meshes are culled to it (and the mask to the camera as
  // well): on a large city most of the scene is skipped.
  depthShader_.Use();
  for (int half = 0; half < 2; ++half) {
    const Frustum mapFrustum(slot.matrix[half]);
    const auto inMap = [&](const glm::vec3& min, const glm::vec3& max) {
      return mapFrustum.Intersects(min, max);
    };
    glBindFramebuffer(GL_FRAMEBUFFER, slot.fbo[half]);
    glViewport(0, 0, kFovMapWidth, kFovMapHeight);
    glClear(GL_DEPTH_BUFFER_BIT);
    if (drawTerrain) {
      depthShader_.SetMat4("uLightMVP", slot.matrix[half]);
      terrainMesh_.DrawVisible(inMap);
      terrainRaisedMesh_.DrawVisible(inMap);
      terrainStairMesh_.Draw();
    } else {
      DrawBoxDepth(depthShader_, cubeMesh_, slot.matrix[half],
                   glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
                   glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f));
    }
    if (fovSceneMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", slot.matrix[half]);
      fovSceneMesh_.DrawVisible(inMap);
    }
  }
  return index;
}

void SceneRenderer::DrawFovMasks(const std::vector<FovMask>& masks, const glm::vec4& color,
                                 const glm::mat4& viewProj, bool drawTerrain,
                                 float mapHalfExtent) {
  std::vector<FovMask> live;
  for (const FovMask& mask : masks) {
    if (mask.slot >= 0) live.push_back(mask);
  }
  if (live.empty()) return;
  // Same blend/stencil/no-depth-write state as the CPU overlay, with the
  // mask drawn over the already-lit geometry (LEQUAL plus a small offset so
  // the identical surfaces pass their own depth).
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDepthMask(GL_FALSE);
  glDepthFunc(GL_LEQUAL);
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_EQUAL, 0, 0xFF);
  glStencilOp(GL_KEEP, GL_KEEP, GL_INCR);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(-1.0f, -2.0f);

  fovMaskShader_.Use();
  fovMaskShader_.SetFloat("uRange", tactics::constants::kFovConeVisualRange);
  fovMaskShader_.SetFloat("uNear", kFovMapNear);
  fovMaskShader_.SetFloat("uFar", tactics::constants::kFovConeVisualRange);
  // The larger of the two texel footprints (vertical: the whole elevation
  // range over the map's height) per unit of map depth.
  fovMaskShader_.SetFloat("uTexelPerDepth",
                          std::max(2.0f * std::tan(glm::radians(kFovMapHalfElevationDegrees)) /
                                       kFovMapHeight,
                                   2.0f * std::tan(glm::radians(kFovMapHalfAzimuthDegrees)) /
                                       kFovMapWidth));
  fovMaskShader_.SetFloat("uProbeHeight", fovProbeHeight_);
  fovMaskShader_.SetVec4("uColor", color);
  const Frustum cameraFrustum(viewProj);
  // Up to kFovMasksPerPass figures per pass; a larger team takes several
  // passes, kept from stacking by the stencil cap.
  for (size_t first = 0; first < live.size(); first += kFovMasksPerPass) {
    const int count =
        static_cast<int>(std::min<size_t>(kFovMasksPerPass, live.size() - first));
    std::vector<Frustum> mapFrustums;
    for (int i = 0; i < kFovMasksPerPass; ++i) {
      // Unused figure slots still get valid depth textures bound (WebGL
      // validates every sampler); the shader never reads them.
      const FovMask& mask = live[first + std::min(i, count - 1)];
      const FovMapSlot& slot = fovMaps_[mask.slot];
      const std::string index = "[" + std::to_string(i) + "]";
      fovMaskShader_.SetVec3(("uEye" + index).c_str(), slot.eye);
      fovMaskShader_.SetVec3(("uFacing" + index).c_str(),
                             glm::vec3(std::cos(slot.yaw), 0.0f, std::sin(slot.yaw)));
      fovMaskShader_.SetFloat(("uCosHalfFov" + index).c_str(),
                              std::cos(glm::radians(mask.halfFovDegrees)));
      fovMaskShader_.SetMat4(("uFovMatrix0" + index).c_str(), slot.matrix[0]);
      fovMaskShader_.SetMat4(("uFovMatrix1" + index).c_str(), slot.matrix[1]);
      for (int half = 0; half < 2; ++half) {
        const int unit = 1 + 2 * i + half;
        fovMaskShader_.SetInt(("uFovMap" + std::to_string(2 * i + half)).c_str(), unit);
        glActiveTexture(GL_TEXTURE0 + unit);
        glBindTexture(GL_TEXTURE_2D, slot.tex[half]);
        if (i < count) mapFrustums.emplace_back(slot.matrix[half]);
      }
    }
    glActiveTexture(GL_TEXTURE0);
    fovMaskShader_.SetInt("uCount", count);

    // A fragment outside every figure's maps is always discarded, so only
    // chunks on screen and inside some map can be tinted.
    const auto receives = [&](const glm::vec3& min, const glm::vec3& max) {
      if (!cameraFrustum.Intersects(min, max)) return false;
      for (const Frustum& map : mapFrustums) {
        if (map.Intersects(min, max)) return true;
      }
      return false;
    };
    const auto drawReceiver = [&](const LitTriangleMesh& mesh) {
      fovMaskShader_.SetMat4("uModel", glm::mat4(1.0f));
      fovMaskShader_.SetMat4("uMVP", viewProj);
      mesh.DrawVisible(receives);
    };
    if (drawTerrain) {
      drawReceiver(terrainMesh_);
      drawReceiver(terrainRaisedMesh_);
    } else {
      const glm::mat4 model =
          BoxModel(glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
                   glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f));
      fovMaskShader_.SetMat4("uModel", model);
      fovMaskShader_.SetMat4("uMVP", viewProj * model);
      cubeMesh_.Draw();
    }
    if (fovSceneMesh_.HasGeometry()) drawReceiver(fovSceneMesh_);
    // Facade pane extrusions receive the tint like the walls they sit on: the
    // extruded frames stand proud of the wall and need their own coverage.
    if (facadeMesh_.HasGeometry()) drawReceiver(facadeMesh_);
  }
  glDepthFunc(GL_LESS);
}

void SceneRenderer::RenderPane(const GameLogic& game, Team team, bool fogActive,
                               const TeamVisibility& visibility, const OrbitCamera& camera, int x,
                               int y, int width, int height, const PaneOverlays& overlays,
                               GLuint targetFramebuffer, const RenderDebugOptions& debug) {
  const ScopedDrawStats drawStats(debug.stats);
  if (!debug.disableTrees) trees_.SetTrees(game.GetScene().trees);
  const auto& obstacles = game.GetScene().obstacles;
  SetRagdollObstacles(obstacles);
  SetRagdollGround(game.GetScene().walkSurfaces, game.GetScene().ground);
  const auto& sidewalks = game.GetScene().sidewalks;
  const auto& streetProps = game.GetScene().props;
  const float mapHalfExtent = game.GetScene().mapHalfExtent;
  const tactics::HeightField& terrain = game.GetScene().ground;
  // (Re)triangulate the terrain only when the scene's heightfield changes.
  const bool terrainChanged =
      terrainKey_.heights != terrain.heights || terrainKey_.nx != terrain.nx ||
      terrainKey_.nz != terrain.nz || terrainKey_.minX != terrain.minX ||
      terrainKey_.minZ != terrain.minZ || terrainKey_.step != terrain.step;
  if (terrainChanged) {
    if (!terrain.Empty()) {
      BuildTerrainMesh(terrain, game.GetScene().terrainWalls, game.GetScene().terrainCutouts,
                       &terrainMesh_, &terrainRaisedMesh_);
    }
    terrainKey_ = terrain;
    fovSceneKey_ = 0;
    ++fovMapGeneration_;
  }
  if (!terrain.Empty()) {
    const auto key = FovSceneFingerprint(game.GetScene());
    if (terrainChanged || pavementKey_ != key) {
      BuildPavementMesh(game.GetScene().roads, 0.0f, &terrainRoadMesh_);
      BuildPavementMesh(game.GetScene().sidewalkSurfaces, 0.10f, &terrainSidewalkMesh_);
      BuildTerrainStairMesh(game.GetScene().terrainStairs, &terrainStairMesh_);
      pavementKey_ = key;
    }
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

  // Street fixtures are static and consist of hundreds of tiny cubes in a
  // typical city. Rebuild only when the scene changes, then submit one
  // furniture mesh and one prop mesh per material instead of one draw per
  // cube.
  size_t furnitureIndex = 0;
  bool sameStreetFurniture = true;
  for (const tactics::Obstacle& obstacle : obstacles) {
    if (obstacle.kind == tactics::ObstacleKind::StreetFurniture) {
      sameStreetFurniture &= furnitureIndex < streetFurnitureKey_.size() &&
                             obstacle.bounds.min == streetFurnitureKey_[furnitureIndex].min &&
                             obstacle.bounds.max == streetFurnitureKey_[furnitureIndex].max;
      ++furnitureIndex;
    }
  }
  sameStreetFurniture &= furnitureIndex == streetFurnitureKey_.size();
  const bool sameStreetProps =
      streetProps.size() == streetPropKey_.size() &&
      std::equal(streetProps.begin(), streetProps.end(), streetPropKey_.begin(),
                 [](const tactics::StreetProp& a, const tactics::StreetProp& b) {
                   return a.kind == b.kind && a.position == b.position && a.yaw == b.yaw &&
                          a.secondaryYaw == b.secondaryYaw;
                 });
  if (!sameStreetFurniture || !sameStreetProps) {
    BuildStreetFixtureMeshes(obstacles, streetProps, &streetFurnitureMesh_,
                             &streetPropDepthMesh_, &streetPropMeshes_);
    streetFurnitureKey_.clear();
    streetFurnitureKey_.reserve(furnitureIndex);
    for (const tactics::Obstacle& obstacle : obstacles) {
      if (obstacle.kind == tactics::ObstacleKind::StreetFurniture) {
        streetFurnitureKey_.push_back(obstacle.bounds);
      }
    }
    streetPropKey_ = streetProps;
  }

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

  // Building massing (visual only), rebuilt when the scene's records change.
  {
    const unsigned long long massingKey = MassingFingerprint(game.GetScene());
    if (massingKey != massingKey_) {
      BuildMassingMesh(game.GetScene(), &massingMesh_);
      massingKey_ = massingKey;
    }
  }
  // Facade decals (issue #152), rebuilt when the opening records change.
  {
    const unsigned long long facadeKey = FacadeFingerprint(game.GetScene());
    if (facadeKey != facadeKey_) {
      BuildFacadeMesh(game.GetScene(), &facadeMesh_);
      facadeRecessHoles_ = RecessHolesByObstacle(game.GetScene());
      facadeKey_ = facadeKey;
      obstacleKey_ = 0;
    }
  }
  // Building prisms and other obstacles, rebuilt when they (or the facade
  // holes cut into them) change.
  {
    const unsigned long long obstacleKey = ObstacleFingerprint(game.GetScene());
    if (obstacleKey != obstacleKey_) {
      BuildObstacleMeshes(game.GetScene(), facadeRecessHoles_, &obstacleMesh_,
                          &obstacleCasterMesh_);
      obstacleKey_ = obstacleKey;
    }
  }
  // Batched walk-surface slabs, rebuilt when the surfaces change.
  {
    const unsigned long long deckKey = DeckFingerprint(game.GetScene());
    if (deckKey != deckKey_) {
      BuildDeckMesh(game.GetScene(), &deckMesh_);
      deckKey_ = deckKey;
    }
  }
  // The building whose roof each walk surface is (-1: none), for fading a
  // building as a whole; it depends on both the surfaces and the obstacles.
  if (surfaceBuildingKey_ != (deckKey_ ^ obstacleKey_)) {
    surfaceBuilding_ = RoofBuildings(game.GetScene());
    surfaceBuildingKey_ = deckKey_ ^ obstacleKey_;
  }

  // Shadow pass: only casters this team can currently see.
  glDisable(GL_SCISSOR_TEST);
  if (!debug.disableShadows && !debug.thermal) {
    const Frustum lightFrustum(lightSpaceMatrix_);
    const auto inShadowMap = [&](const glm::vec3& min, const glm::vec3& max) {
      return lightFrustum.Intersects(min, max);
    };
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
    glViewport(0, 0, kShadowMapSize, kShadowMapSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    depthShader_.Use();
    // Hills cast shadows (into valleys, onto units); the flat ground never
    // could, so flat scenes skip the extra caster exactly as before.
    if (drawTerrain) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      terrainMesh_.DrawVisible(inShadowMap);
      terrainRaisedMesh_.DrawVisible(inShadowMap);
      terrainStairMesh_.Draw();
    }
    if (obstacleCasterMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      obstacleCasterMesh_.DrawVisible(inShadowMap);
    }
    if (streetFurnitureMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      streetFurnitureMesh_.DrawVisible(inShadowMap);
    }
    if (streetPropDepthMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      streetPropDepthMesh_.DrawVisible(inShadowMap);
    }
    if (massingMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      massingMesh_.DrawVisible(inShadowMap);
    }
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(kDeckShadowSlopeBias, kDeckShadowConstantBias);
    if (deckMesh_.HasGeometry()) {
      depthShader_.SetMat4("uLightMVP", lightSpaceMatrix_);
      deckMesh_.DrawVisible(inShadowMap);
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    for (const Unit& unit : game.GetScene().units) {
      if (!IsUnitVisibleForRender(unit, team, fogActive, visibility, game.GetScene())) continue;
      DrawUnitDepth(depthShader_, cubeMesh_, sphereMesh_, lightSpaceMatrix_, unit);
    }
    DrawPlantedFlagDepth(depthShader_, cubeMesh_, sphereMesh_, lightSpaceMatrix_, game);
    if (!debug.disableTrees) trees_.DrawDepth(lightSpaceMatrix_);
  }

  // Shadow-map FOV masks: this pane's figures' depth maps (and the free-aim
  // sweep's), re-rendered only where stale, all before the main pass so the
  // pane framebuffer is not switched away from once it is in use.
  ++fovMapClock_;
  std::vector<FovMask> teamFovMasks, otherFovMasks, aimFovMasks;
  const bool shadowMapFov = fovOverlayMode_ == FovOverlayMode::ShadowMap && !debug.disableFov;
  const bool aimSweep = overlays.aimShooter && overlays.aimShooter->alive && !debug.disableFov;
  if (shadowMapFov || aimSweep) {
    // Static receivers/casters for the shadow-map mask, cached per scene.
    const unsigned long long sceneKey = FovSceneFingerprint(game.GetScene());
    if (sceneKey != fovSceneKey_) {
      BuildFovSceneMesh(game.GetScene(), &fovSceneMesh_);
      fovSceneKey_ = sceneKey;
      ++fovMapGeneration_;
    }
    // Free the maps of figures no pane has asked for in a while.
    for (FovMapSlot& slot : fovMaps_) {
      if (slot.lastUsed + kFovMapIdlePanes < fovMapClock_) DestroyFovMaps(&slot);
    }
    fovMaps_.erase(std::remove_if(fovMaps_.begin(), fovMaps_.end(),
                                  [](const FovMapSlot& slot) { return slot.fbo[0] == 0; }),
                   fovMaps_.end());
    if (shadowMapFov) {
      const Team otherTeam = team == Team::Blue ? Team::Red : Team::Blue;
      for (const Unit& unit : game.GetScene().units) {
        if (!unit.alive) continue;
        std::vector<FovMask>* masks = unit.team == team ? &teamFovMasks
                                      : overlays.spectatorFov && unit.team == otherTeam
                                          ? &otherFovMasks
                                          : nullptr;
        if (!masks) continue;
        masks->push_back({AcquireFovMaps(unit, -1, unit.VisualYaw(), drawTerrain, mapHalfExtent),
                          tactics::constants::kShootHalfFovDegrees});
      }
    }
    // Free-aim (issue #129): three 120-degree sectors (each within the two
    // depth maps' reach) tile the full circle around the aiming figure.
    if (aimSweep) {
      const Unit& shooter = *overlays.aimShooter;
      for (int sector = 0; sector < 3; ++sector) {
        aimFovMasks.push_back(
            {AcquireFovMaps(shooter, sector, shooter.facingYaw + glm::radians(120.0f * sector),
                            drawTerrain, mapHalfExtent),
             61.0f});
      }
    }
  }
  glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);

  // Scissor, not just viewport, is required here: glClear() would
  // otherwise clear the whole framebuffer (including other panes'
  // already-drawn rects) regardless of the glViewport rect.
  glEnable(GL_SCISSOR_TEST);
  glViewport(x, y, width, height);
  glScissor(x, y, width, height);
  if (debug.thermal) {
    // Off-map void matches the scene's overall tone: washed-out bright for
    // black-hot (the review's reference footage), near-black for white-hot.
    const float voidGray = debug.thermalBlackHot ? 0.86f : 0.14f;
    glClearColor(voidGray, voidGray, voidGray, 1.0f);
  } else {
    glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
  }
  glClearStencil(0);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  const glm::mat4 view = camera.ViewMatrix();
  const glm::mat4 proj =
      camera.ProjectionMatrix(static_cast<float>(width) / static_cast<float>(height));
  const glm::mat4 viewProj = proj * view;
  // The static scene meshes are chunked; skip the chunks off screen.
  const Frustum cameraFrustum(viewProj);
  const auto onScreen = [&](const glm::vec3& min, const glm::vec3& max) {
    return cameraFrustum.Intersects(min, max);
  };

  // Only opaque, visible geometry contributes to AO. Faded buildings are
  // blended afterward, so they cannot leave opaque contact shadows over
  // the figures and paths the see-through pass is meant to reveal.
  const bool thermalAo = debug.thermal &&
      thermalOcclusion_.Begin(width, height, debug.thermalBlackHot ? 0.86f : 0.14f);
  if (debug.thermal && !thermalAo) {
    glBindFramebuffer(GL_FRAMEBUFFER, targetFramebuffer);
    glViewport(x, y, width, height);
    glScissor(x, y, width, height);
  }

  // Issue #136 camera-occlusion see-through: per frame, raycast from the
  // camera to every point the player needs to see (own figures, movement
  // frontier, move paths/ghosts, sighting ghosts) and mark each obstacle,
  // deck slab or massing detail actually crossing a sightline. Marked
  // occluders are skipped by the opaque pass below and re-drawn translucent
  // after the figures.
  const auto& walkSurfaces = game.GetScene().walkSurfaces;
  const auto& massing = game.GetScene().massing;
  std::vector<char> fadeObstacle(obstacles.size(), 0);
  std::vector<char> fadeSurface(walkSurfaces.size(), 0);
  std::vector<char> fadeMassing(massing.size(), 0);
  std::vector<char> fadeBuilding;
  bool anyFaded = false;
  if (!debug.disableOcclusionFade) {
    std::vector<glm::vec3> probes = CollectSeeThroughProbes(game, team, fogActive, overlays);
    const glm::vec3 cameraPos = camera.Position();
    const uint64_t geometryKey = FadeGeometryKey(game.GetScene());
    FadeCache& fadeCache = fadeCache_[team == tactics::Team::Blue ? 0 : 1];
    if (fadeCache.valid && fadeCache.cameraPos == cameraPos &&
        fadeCache.geometryKey == geometryKey && fadeCache.probes == probes) {
      fadeObstacle = fadeCache.fadeObstacle;
      fadeSurface = fadeCache.fadeSurface;
      fadeMassing = fadeCache.fadeMassing;
      fadeBuilding = fadeCache.fadeBuilding;
      anyFaded = fadeCache.anyFaded;
    } else {
      struct ProbeRay {
        glm::vec3 direction;
        float length;
        float azimuth;  // atan2(direction.z, direction.x).
      };
      std::vector<ProbeRay> rays;
      rays.reserve(probes.size());
      for (const glm::vec3& probe : probes) {
        const glm::vec3 segment = probe - cameraPos;
        const float length = glm::length(segment);
        if (length > 1e-4f) {
          const glm::vec3 direction = segment / length;
          rays.push_back({direction, length, std::atan2(direction.z, direction.x)});
        }
      }
      // Angular broadphase (issue #256): with the frontier open there are
      // hundreds of probes, and testing every occluder against every ray is
      // O(rays x occluders). Sorted by azimuth, an occluder only tests the
      // rays inside its XZ angular span as seen from the camera; a crossing
      // ray's hit lies in the occluder's bounding circle, hence inside that
      // span, so the pruning cannot change any result.
      std::sort(rays.begin(), rays.end(),
                [](const ProbeRay& a, const ProbeRay& b) { return a.azimuth < b.azimuth; });
      std::vector<float> rayAzimuths;
      rayAzimuths.reserve(rays.size());
      for (const ProbeRay& ray : rays) rayAzimuths.push_back(ray.azimuth);
      const glm::vec2 cameraXZ(cameraPos.x, cameraPos.z);
      const auto anyRayHits = [&](const glm::vec2& center, float radius, const auto& test) {
        const glm::vec2 toCenter = center - cameraXZ;
        const float distance = glm::length(toCenter);
        // Padding absorbs float error at the span ends; camera over the
        // occluder tests every ray (its span is the whole circle).
        const float padded = radius + 0.05f;
        if (distance <= padded) {
          for (const ProbeRay& ray : rays) {
            if (test(ray)) return true;
          }
          return false;
        }
        const float halfSpan = std::asin(std::min(1.0f, padded / distance));
        const float centerAzimuth = std::atan2(toCenter.y, toCenter.x);
        const auto scan = [&](float lo, float hi) {
          const size_t begin =
              std::lower_bound(rayAzimuths.begin(), rayAzimuths.end(), lo) - rayAzimuths.begin();
          const size_t end =
              std::upper_bound(rayAzimuths.begin(), rayAzimuths.end(), hi) - rayAzimuths.begin();
          for (size_t r = begin; r < end; ++r) {
            if (test(rays[r])) return true;
          }
          return false;
        };
        constexpr float kPi = 3.14159265358979f;
        const float lo = centerAzimuth - halfSpan;
        const float hi = centerAzimuth + halfSpan;
        if (lo < -kPi) return scan(lo + 2.0f * kPi, kPi) || scan(-kPi, hi);
        if (hi > kPi) return scan(lo, kPi) || scan(-kPi, hi - 2.0f * kPi);
        return scan(lo, hi);
      };
      // Same endpoint handling as LineOfSightClear: a hit at (or essentially
      // at) the probe itself does not count, so a figure hugging a wall does
      // not fade that wall.
      constexpr float kEndpointEpsilon = 1e-2f;
      const auto crossesProbeRay = [&](const tactics::Obstacle& obstacle) {
        // Bounding circle over both the footprint and the bounds: hits are
        // clipped by the footprint in XZ, but fall back to bounds corners.
        glm::vec2 lo(obstacle.bounds.min.x, obstacle.bounds.min.z);
        glm::vec2 hi(obstacle.bounds.max.x, obstacle.bounds.max.z);
        for (const glm::vec2& p : obstacle.footprint) {
          lo = glm::min(lo, p);
          hi = glm::max(hi, p);
        }
        return anyRayHits(0.5f * (lo + hi), 0.5f * glm::length(hi - lo), [&](const ProbeRay& ray) {
          float t = 0.0f;
          return tactics::RayIntersectsObstacle(cameraPos, ray.direction, obstacle, &t) &&
                 t > kEndpointEpsilon && t < ray.length - kEndpointEpsilon;
        });
      };
      for (size_t i = 0; i < obstacles.size(); ++i) {
        // Post-sized furniture is too small to hide a useful target and is kept
        // in one static render batch below.
        if (obstacles[i].navOnly || obstacles[i].kind == tactics::ObstacleKind::StreetFurniture) {
          continue;
        }
        if (crossesProbeRay(obstacles[i])) {
          fadeObstacle[i] = 1;
          anyFaded = true;
        }
      }
      for (size_t i = 0; i < walkSurfaces.size(); ++i) {
        const tactics::WalkSurface& surface = walkSurfaces[i];
        if (surface.vertices.size() < 3) continue;  // Never intersected.
        glm::vec2 lo(surface.vertices[0].x, surface.vertices[0].z);
        glm::vec2 hi = lo;
        for (const glm::vec3& v : surface.vertices) {
          lo = glm::min(lo, glm::vec2(v.x, v.z));
          hi = glm::max(hi, glm::vec2(v.x, v.z));
        }
        const bool crossed =
            anyRayHits(0.5f * (lo + hi), 0.5f * glm::length(hi - lo), [&](const ProbeRay& ray) {
              float t = 0.0f;
              return tactics::RayIntersectsWalkSurface(cameraPos, ray.direction, surface,
                                                       kDeckThickness, &t) &&
                     t > kEndpointEpsilon && t < ray.length - kEndpointEpsilon;
            });
        if (crossed) {
          fadeSurface[i] = 1;
          anyFaded = true;
        }
      }
      // Rails, pier caps and building massing can occlude a probe even when
      // the slab/prism they belong to does not. Test the same conservative
      // sight prisms used by gameplay LOS so those hits trigger group fading.
      for (size_t i = 0; i < massing.size(); ++i) {
        const tactics::MassingDetail& d = massing[i];
        if (d.footprint.size() < 3 || d.y1 <= d.y0) continue;
        tactics::Obstacle prism;
        prism.footprint = d.footprint;
        prism.bounds.min = glm::vec3(std::numeric_limits<float>::infinity(), d.y0,
                                    std::numeric_limits<float>::infinity());
        prism.bounds.max = glm::vec3(-std::numeric_limits<float>::infinity(), d.y1,
                                    -std::numeric_limits<float>::infinity());
        for (const glm::vec2& p : d.footprint) {
          prism.bounds.min.x = std::min(prism.bounds.min.x, p.x);
          prism.bounds.max.x = std::max(prism.bounds.max.x, p.x);
          prism.bounds.min.z = std::min(prism.bounds.min.z, p.y);
          prism.bounds.max.z = std::max(prism.bounds.max.z, p.y);
        }
        if (crossesProbeRay(prism)) {
          fadeMassing[i] = 1;
          anyFaded = true;
        }
      }
      // A building fades as a whole: if any part of it (either prism, its roof
      // slab, massing detail) blocks a sightline, all of it goes see-through
      // together.
      const std::vector<int>& surfaceBuilding = surfaceBuilding_;
      fadeBuilding.assign(game.GetScene().buildings.size(), 0);
      const auto markBuilding = [&](int b) {
        if (b >= 0 && b < static_cast<int>(fadeBuilding.size())) fadeBuilding[b] = 1;
      };
      for (size_t i = 0; i < obstacles.size(); ++i) {
        if (fadeObstacle[i]) markBuilding(obstacles[i].building);
      }
      for (size_t i = 0; i < walkSurfaces.size(); ++i) {
        if (fadeSurface[i]) markBuilding(surfaceBuilding[i]);
      }
      for (size_t i = 0; i < massing.size(); ++i) {
        if (fadeMassing[i]) markBuilding(massing[i].building);
      }
      for (size_t i = 0; i < obstacles.size(); ++i) {
        const int b = obstacles[i].building;
        if (b >= 0 && b < static_cast<int>(fadeBuilding.size()) && fadeBuilding[b]) {
          fadeObstacle[i] = 1;
          anyFaded = true;
        }
      }
      for (size_t i = 0; i < walkSurfaces.size(); ++i) {
        const int b = surfaceBuilding[i];
        if (b >= 0 && b < static_cast<int>(fadeBuilding.size()) && fadeBuilding[b]) {
          fadeSurface[i] = 1;
          anyFaded = true;
        }
      }
      for (size_t i = 0; i < massing.size(); ++i) {
        const int b = massing[i].building;
        if (b >= 0 && b < static_cast<int>(fadeBuilding.size()) && fadeBuilding[b]) {
          fadeMassing[i] = 1;
          anyFaded = true;
        }
      }

      // The highway is one visual structure. A hit on any slab, deck rail,
      // pier cap, column or footing fades every part, like a building group.
      bool fadeOverpass = false;
      for (size_t i = 0; i < obstacles.size(); ++i) {
        fadeOverpass |= fadeObstacle[i] && obstacles[i].kind == tactics::ObstacleKind::Pier;
      }
      for (size_t i = 0; i < walkSurfaces.size(); ++i) {
        fadeOverpass |= fadeSurface[i] && walkSurfaces[i].overpass;
      }
      const auto overpassDetail = [](const tactics::MassingDetail& d) {
        return d.kind == tactics::MassingDetail::Kind::PierCap ||
               d.kind == tactics::MassingDetail::Kind::DeckRail;
      };
      for (size_t i = 0; i < massing.size(); ++i) {
        fadeOverpass |= fadeMassing[i] && overpassDetail(massing[i]);
      }
      if (fadeOverpass) {
        for (size_t i = 0; i < obstacles.size(); ++i) {
          if (obstacles[i].kind == tactics::ObstacleKind::Pier) fadeObstacle[i] = 1;
        }
        for (size_t i = 0; i < walkSurfaces.size(); ++i) {
          if (walkSurfaces[i].overpass) fadeSurface[i] = 1;
        }
        for (size_t i = 0; i < massing.size(); ++i) {
          if (overpassDetail(massing[i])) fadeMassing[i] = 1;
        }
        anyFaded = true;
      }
      fadeCache.valid = true;
      fadeCache.cameraPos = cameraPos;
      fadeCache.geometryKey = geometryKey;
      fadeCache.probes = std::move(probes);
      fadeCache.fadeObstacle = fadeObstacle;
      fadeCache.fadeSurface = fadeSurface;
      fadeCache.fadeMassing = fadeMassing;
      fadeCache.fadeBuilding = fadeBuilding;
      fadeCache.anyFaded = anyFaded;
    }
  }

  // Thermal view only restyles the world: gameplay overlays (selection,
  // paths, ropes, FOV, cones) keep their in-game colors on top of it.
  // Uniform values stick to each program across Use() switches, so setting
  // them once per pane covers all later draws.
  const int thermalFlag = debug.thermal ? 1 : 0;
  litShader_.Use();
  litShader_.SetVec3("uLightDir", lightDir_);
  litShader_.SetVec3("uViewPos", camera.Position());
  litShader_.SetInt("uShadowMap", 0);
  litShader_.SetInt("uDisableShadows", (debug.disableShadows || debug.thermal) ? 1 : 0);
  litShader_.SetInt("uMonochrome", debug.monochrome ? 1 : 0);
  litShader_.SetInt("uThermal", thermalFlag);
  litShader_.SetInt("uThermalBlackHot", debug.thermalBlackHot ? 1 : 0);
  const auto thermalMaterial = [&](ThermalMaterial material) {
    if (debug.thermal) litShader_.SetInt("uThermalMaterial", static_cast<int>(material));
  };
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, shadowDepthTex_);

  if (drawTerrain) {
    // Retaining-wall tops coincide with the high pad. Bias the terrain
    // behind the solid geometry to keep their shared edge free of z-fighting.
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    // Earthy green-brown so the hills read as terrain rather than factory
    // floor; a city with levels is concrete throughout, base ground, raised
    // tiers, cliff faces and stairs alike, like its decks.
    const bool city = !game.GetScene().roads.empty();
    thermalMaterial(city ? ThermalMaterial::Concrete : ThermalMaterial::Ground);
    DrawLitVisible(litShader_, terrainMesh_, viewProj, lightSpaceMatrix_,
                   city ? glm::vec4(kDeckColor, 1.0f) : glm::vec4(0.30f, 0.36f, 0.22f, 1.0f),
                   onScreen);
    thermalMaterial(ThermalMaterial::Concrete);
    DrawLitVisible(litShader_, terrainRaisedMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(kDeckColor, 1.0f), onScreen);
    DrawLitModel(litShader_, terrainStairMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(kDeckColor, 1.0f));
    glDisable(GL_POLYGON_OFFSET_FILL);
  } else {
    thermalMaterial(ThermalMaterial::Ground);
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_,
               glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
               glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f),
               glm::vec4(0.16f, 0.18f, 0.20f, 1.0f));
  }

  thermalMaterial(ThermalMaterial::Concrete);
  for (const AABB& slab : game.GetScene().sidewalks) {
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, slab.min, slab.max - slab.min,
               glm::vec4(0.36f, 0.37f, 0.39f, 1.0f));
  }

  if (drawTerrain) {
    DrawLitVisible(litShader_, terrainSidewalkMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(0.36f, 0.37f, 0.39f, 1.0f), onScreen);
    thermalMaterial(ThermalMaterial::Asphalt);
    DrawLitVisible(litShader_, terrainRoadMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(0.13f, 0.14f, 0.16f, 1.0f), onScreen);
  }
  thermalMaterial(ThermalMaterial::Concrete);
  for (const tactics::RoadSurface& sidewalk : game.GetScene().sidewalkSurfaces) {
    if (drawTerrain) break;
    BuildSurfacePatch(sidewalk.vertices, 0.10f, &geometryMesh_);
    DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(0.36f, 0.37f, 0.39f, 1.0f));
  }

  thermalMaterial(ThermalMaterial::Asphalt);
  for (const tactics::RoadSurface& road : game.GetScene().roads) {
    if (drawTerrain) break;
    BuildSurfacePatch(road.vertices, 0.0f, &geometryMesh_);
    DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                 glm::vec4(0.13f, 0.14f, 0.16f, 1.0f));
  }
  thermalMaterial(ThermalMaterial::Concrete);
  // Every static batch below is grouped (per surface, obstacle or building)
  // so camera-faded occluders are skipped here and re-drawn translucent
  // after the figures, without re-tessellating the rest every frame.
  const auto fadedBuilding = [&](int b) {
    return b >= 0 && b < static_cast<int>(fadeBuilding.size()) && fadeBuilding[b];
  };
  const auto skipFadedBuilding = [&](int b) { return fadedBuilding(b); };
  const auto skipFadedMassing = [&](int i) { return fadeMassing[i] != 0; };
  const auto skipFadedSurface = [&](int i) { return fadeSurface[i] != 0; };
  const auto skipFadedObstacle = [&](int i) { return fadeObstacle[i] != 0; };
  const auto unlessFaded = [&](const std::function<bool(int)>& skip) {
    return anyFaded ? skip : std::function<bool(int)>();
  };
  // Concrete, clearly lighter than the asphalt below: the elevated deck and
  // its ramps must read as a bridge, not as more ground road.
  if (deckMesh_.HasGeometry()) {
    DrawLitVisible(litShader_, deckMesh_, viewProj, lightSpaceMatrix_, glm::vec4(kDeckColor, 1.0f),
                   onScreen, unlessFaded(skipFadedSurface));
  }

  thermalMaterial(ThermalMaterial::Building);
  // Recessed facade openings (doors, factory windows) cut real holes into
  // the drawn walls; the niche geometry lands with the facade mesh below.
  // Shadow/FOV casters keep the solid wall: the niches are shallow and sit
  // behind it.
  if (obstacleMesh_.HasGeometry()) {
    DrawLitVisible(litShader_, obstacleMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(kObstacleColor, 1.0f), onScreen, unlessFaded(skipFadedObstacle));
  }
  if (streetFurnitureMesh_.HasGeometry()) {
    DrawLitVisible(litShader_, streetFurnitureMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(kObstacleColor, 1.0f), onScreen);
  }

  // Visual-only street props (issue #150), batched by material.
  for (size_t i = 0; i < streetPropMeshes_.size(); ++i) {
    if (!streetPropMeshes_[i].HasGeometry()) continue;
    DrawLitVisible(litShader_, streetPropMeshes_[i], viewProj, lightSpaceMatrix_,
                   StreetPropColor(static_cast<StreetPropMaterial>(i)), onScreen);
  }

  // Facade pane extrusions (issue #152) over the walls just drawn. A
  // camera-faded building keeps its openings out of both passes (they would
  // otherwise hang opaque in front of its translucent walls).
  if (facadeMesh_.HasGeometry()) {
    DrawLitVisible(litShader_, facadeMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(kObstacleColor, 1.0f), onScreen, unlessFaded(skipFadedBuilding));
  }

  // Building and overpass massing: parapets, roof furniture, awnings,
  // support bars and deck sides (issues #151 and #223).
  const auto detailFaded = [&](size_t i) {
    return i < fadeMassing.size() && fadeMassing[i];
  };
  const auto drawDetail = [&](const tactics::MassingDetail& d, const glm::vec4& color) {
    if (d.footprint.size() < 3 || d.y1 <= d.y0) return;
    thermalMaterial(ThermalMaterial::Building);
    BuildMassingDetail(d, &geometryMesh_);
    DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f), color);
  };
  if (massingMesh_.HasGeometry()) {
    DrawLitVisible(litShader_, massingMesh_, viewProj, lightSpaceMatrix_,
                   glm::vec4(kObstacleColor, 1.0f), onScreen, unlessFaded(skipFadedMassing));
  }

  // Zipline anchor posts (the cable itself is drawn with the line overlays).
  thermalMaterial(ThermalMaterial::Metal);
  for (const tactics::Zipline& line : game.GetScene().ziplines) {
    for (const glm::vec3& foot : {line.a, line.b}) {
      constexpr float kPostHalf = 0.12f;
      DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_,
                 foot + glm::vec3(-kPostHalf, 0.0f, -kPostHalf),
                 glm::vec3(2.0f * kPostHalf, tactics::constants::kZiplinePostHeight,
                           2.0f * kPostHalf),
                 glm::vec4(kObstacleColor, 1.0f));
    }
  }

  if (!debug.disableTrees) {
    trees_.Draw(viewProj, lightDir_, debug.thermal, debug.thermalBlackHot);
    litShader_.Use();
  }
  thermalMaterial(ThermalMaterial::Figure);
  for (const Unit& unit : game.GetScene().units) {
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility, game.GetScene())) continue;
    DrawUnit(litShader_, cubeMesh_, sphereMesh_, viewProj, lightSpaceMatrix_, unit);
  }
  thermalMaterial(ThermalMaterial::Metal);
  DrawPlantedFlag(litShader_, cubeMesh_, sphereMesh_, viewProj, lightSpaceMatrix_, game);

  if (thermalAo) {
    thermalOcclusion_.Composite(targetFramebuffer, x, y, view, proj, debug.thermalBlackHot,
                                debug.thermalAo);
    litShader_.Use();
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTex_);
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
    // Front faces only: with no culling the stencil cap lets whichever
    // triangle draws first claim the pixel, often a back face, which shows as
    // striped bands and dark slabs through the faded block. Scene meshes use
    // clockwise outward winding (matching CubeMesh), rather than OpenGL's
    // default counter-clockwise front-face convention.
    glFrontFace(GL_CW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    // The faded block would otherwise be shadowed by its own shadow-map
    // entry (it is a caster), darkening its faces into patchy slabs.
    litShader_.SetInt("uDisableShadows", 1);
    // Massing first: it stands proud of the grouped prisms/slabs, so it claims
    // its pixels in the stencil cap instead of being hidden behind them.
    for (size_t i = 0; i < massing.size(); ++i) {
      if (detailFaded(i)) {
        drawDetail(massing[i], glm::vec4(kObstacleColor, kOccluderFadeOpacity));
      }
    }
    for (const FadedOccluder& f : faded) {
      if (f.obstacle >= 0) {
        thermalMaterial(ThermalMaterial::Building);
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
        thermalMaterial(ThermalMaterial::Concrete);
        const std::vector<char> hidden = SharedEdges(walkSurfaces, walkSurfaces[f.surface]);
        BuildSurfacePatch(walkSurfaces[f.surface].vertices, kDeckThickness, &geometryMesh_,
                          &hidden);
        DrawLitModel(litShader_, geometryMesh_, viewProj, lightSpaceMatrix_, glm::mat4(1.0f),
                     glm::vec4(kDeckColor, kOccluderFadeOpacity));
      }
    }
    litShader_.SetInt("uDisableShadows", (debug.disableShadows || debug.thermal) ? 1 : 0);
    glDisable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glDisable(GL_STENCIL_TEST);
    glClear(GL_STENCIL_BUFFER_BIT);  // Later passes expect a clean stencil.
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  if (debug.thermal) ApplyThermalSensor(x, y, width, height);

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
        position.y = std::max(position.y, terrain.Empty() ? sidewalk.vertices.front().y
                         : terrain.MeshHeightAt(position.x, position.z) + 0.12f);
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
  // Thermal view: the tint strength is a debug-panel slider (the pale
  // black-hot world needs a different level than the normal view).
  const float fovAlpha = debug.thermal ? debug.thermalFovAlpha : 0.15f;
  const auto drawTeamFov = [&](Team fovTeam) {
    if (fovOverlayMode_ == FovOverlayMode::ShadowMap) {
      // Issue #110 prototype: projective per-unit mask instead of the
      // analytic ground overlay, one pass for the whole team.
      const glm::vec4 teamColor = fovTeam == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, fovAlpha)
                                                        : glm::vec4(0.9f, 0.25f, 0.22f, fovAlpha);
      DrawFovMasks(fovTeam == team ? teamFovMasks : otherFovMasks, teamColor, viewProj,
                   drawTerrain, mapHalfExtent);
      colorShader_.Use();
    } else {
      const auto build = [&](const Unit& unit) {
        return BuildFovCone(unit, game.SightObstacles(), sidewalks, mapHalfExtent, terrain,
                            game.GetScene().sidewalkSurfaces, game.GetScene().walkSurfaces);
      };
      for (const Unit& unit : game.GetScene().units) {
        if (debug.disableFov || !unit.alive || unit.team != fovTeam) continue;
        if (terrain.Empty()) {
          DrawFovCone(colorShader_, fovConeMesh_, viewProj, unit, build(unit), nullptr, fovAlpha);
          continue;
        }
        auto cached = std::find_if(terrainFovCache_.begin(), terrainFovCache_.end(),
                                   [&](const auto& entry) { return entry.unitId == unit.id; });
        const glm::vec3 eye = unit.EyePosition();
        if (cached == terrainFovCache_.end()) {
          terrainFovCache_.push_back({unit.id, eye, unit.VisualYaw(), build(unit)});
          cached = terrainFovCache_.end() - 1;
        } else if (cached->eye != eye || cached->facingYaw != unit.VisualYaw()) {
          cached->eye = eye;
          cached->facingYaw = unit.VisualYaw();
          cached->points = build(unit);
        }
        DrawFovCone(colorShader_, fovConeMesh_, viewProj, unit, cached->points, nullptr, fovAlpha);
      }
    }
  };
  drawTeamFov(team);
  if (overlays.spectatorFov) {
    // Fresh stencil so the other team's cones blend over (not instead of) these.
    glStencilMask(0xFF);
    glClear(GL_STENCIL_BUFFER_BIT);
    drawTeamFov(team == Team::Blue ? Team::Red : Team::Blue);
  }
  glDisable(GL_STENCIL_TEST);
  // Free-aim (issue #129): the aiming figure's acid-green 360-degree LOS
  // highlight -- the shadow-map mask shader, so it tints every surface the
  // figure could aim at directly (building walls and roofs included, not
  // just the ground). The figure turns to shoot, so its facing doesn't gate
  // the sweep: three 120-degree sectors tile the full circle, in one pass.
  // Own stencil pass so it layers over the team's FOV tint.
  if (aimSweep) {
    glEnable(GL_STENCIL_TEST);
    glStencilMask(0xFF);
    glClear(GL_STENCIL_BUFFER_BIT);
    DrawFovMasks(aimFovMasks, glm::vec4(kAimColor, 0.22f), viewProj, drawTerrain, mapHalfExtent);
    colorShader_.Use();
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
      if (!IsUnitVisibleForRender(other, team, fogActive, visibility, game.GetScene())) continue;
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
        // With a free-aim point placed, the cone previews that shot: it
        // leaves the turned figure and may pitch toward a sky/elevated aim.
        DrawShotCone(colorShader_, coneSurfaceShader_, shotConeMesh_, cubeMesh_, sphereMesh_,
                     viewProj, *selected, coneLitUnits(*selected), obstacles,
                     game.GetScene().sidewalks, game.GetScene().mapHalfExtent,
                     overlays.aimConeTarget ? &*overlays.aimConeTarget : nullptr);
        glDepthFunc(GL_LESS);
        aimingShooter = selected;
      }
    }
  }
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || (!overlays.spectatorFov && unit.team != team) ||
          unit.plan.type != tactics::PlannedActionType::Shoot || &unit == aimingShooter) {
        continue;
      }
      // A planned locked shot aims its cone at the target's torso -- the
      // same point the burst's bullets will be aimed at (and the same
      // marker used while staging the lock), so the footprint shader keeps
      // painting the hit odds on the figure until the commit instead of
      // falling back to the figure's old facing.
      std::optional<glm::vec3> coneAim;
      if (unit.plan.hasAimPoint) {
        coneAim = unit.plan.aimPoint;
      } else if (const Unit* lockTarget = game.FindUnit(unit.plan.shootTargetId)) {
        coneAim = lockTarget->position +
                  glm::vec3(0.0f, tactics::constants::kTorsoAimHeight, 0.0f);
      }
      glEnable(GL_DEPTH_TEST);
      glDepthFunc(GL_LEQUAL);
      DrawShotCone(colorShader_, coneSurfaceShader_, shotConeMesh_, cubeMesh_, sphereMesh_,
                   viewProj, unit, coneLitUnits(unit), obstacles,
                   game.GetScene().sidewalks, game.GetScene().mapHalfExtent,
                   coneAim ? &*coneAim : nullptr);
      glDepthFunc(GL_LESS);
      // A planned shot keeps its "+" selector until the commit, like a
      // planned move keeps its destination ghost.
      if (coneAim) {
        DrawAimMarker(colorShader_, aimOverlayMesh_, viewProj, *coneAim);
      }
    }
  }
  // The unconfirmed "+" selector the aiming player has placed, acid green
  // like the LOS highlight.
  if (overlays.aimMarker) {
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    DrawAimMarker(colorShader_, aimOverlayMesh_, viewProj, *overlays.aimMarker);
    glDepthFunc(GL_LESS);
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
    // A node on its layer's edge counts as touching the unreached class,
    // one step away -- exactly like a reached ground node beside an
    // obstacle -- so the contour lands half a step past the edge node.
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
            d[iz * nx + ix] = f.Reached(ix, iz) && !target ? f.step : 0.0f;
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
            sum += sameLayer(ix, iz, jx, iz) ? g[iz * nx + jx] : -0.5f * f.step;
          }
          tmp[iz * nx + ix] = sum / 5.0f;
        }
      for (int iz = 0; iz < nz; ++iz)
        for (int ix = 0; ix < nx; ++ix) {
          float sum = 0.0f;
          for (int k = -2; k <= 2; ++k) {
            const int jz = std::clamp(iz + k, 0, nz - 1);
            sum += sameLayer(ix, iz, ix, jz) ? tmp[jz * nx + ix] : -0.5f * f.step;
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
              quad[k].p.y = std::max(quad[k].p.y, terrain.Empty() ? sidewalk.vertices.front().y
                               : terrain.MeshHeightAt(quad[k].p.x, quad[k].p.z) + 0.12f);
            }
          }
          quad[k].p.y += kY;
          anyIn |= quad[k].g > 0.0f;
        }
        if (!anyIn) continue;
        // Never triangulate across a layer step (deck/roof edge). Corners
        // on another layer than the strongest one become virtual unreached
        // nodes at the same height, so the contour rounds off at the edge
        // like it does beside a ground obstacle instead of leaving a
        // grid-stepped gap.
        int dom = 0;
        for (int k = 1; k < 4; ++k)
          if (quad[k].g > quad[dom].g) dom = k;
        for (int k = 0; k < 4; ++k) {
          if (sameLayer(cx[dom], cz[dom], cx[k], cz[k])) continue;
          quad[k].p.y = quad[dom].p.y;
          quad[k].g = -0.5f * f.step;
        }
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

  // Overlay green in the thermal view; thermalKey tags it for the mesh caches.
  const int thermalKey = debug.thermal ? (debug.thermalBlackHot ? 2 : 1) : 0;
  const glm::vec3 thermalGreen = debug.thermalBlackHot ? kThermalGreenDark : kThermalGreen;
  // Path/marker green: solid for the first leg, 50% alpha for later ones.
  const glm::vec3 pathGreen = debug.thermal ? thermalGreen : glm::vec3(0.3f, 0.9f, 0.4f);
  // Movement frontier: a glow hugging the reach boundary -- brightest at the
  // boundary, fading to fully transparent within kFadeWidth inside it. The
  // boundary is the zero contour of a blurred signed-distance field, so it
  // is smooth rather than following the sampling grid.
  if (overlays.moveFrontier && overlays.moveFrontier->nx > 0) {
    const tactics::ReachField& f = *overlays.moveFrontier;
    const glm::vec2 origin(f.minX, f.minZ);
    if (frontierKeyField_ != &f || frontierKeyOrigin_ != origin ||
        frontierKeyBudget_ != f.budget || frontierKeyThermal_ != thermalKey) {
      frontierKeyField_ = &f;
      frontierKeyOrigin_ = origin;
      frontierKeyBudget_ = f.budget;
      frontierKeyThermal_ = thermalKey;
      std::vector<ColorTriangleMesh::Vertex> fill;
      std::vector<glm::vec3> border;
      appendFrontier(f, debug.thermal ? thermalGreen : kSetupColor, fill, border);
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
    // Same green for every leg, half-transparent after the
    // first (blending is already on for the whole overlay section; turning it
    // off here would flatten the zipline fill's alpha gradient below).
    const glm::vec3 frontierColor = debug.thermal ? thermalGreen : kSetupColor;
    unlitShader_.SetVec4("uColor", glm::vec4(frontierColor,
                                             overlays.moveFrontierSubsequentLeg ? 0.5f : 1.0f));
    frontierBorder_.DrawSegments();
    glDisable(GL_POLYGON_OFFSET_FILL);
  }
  // Zipline regions: where a walk -> ride -> walk plan can reach, in a second
  // (cyan) tint, with the usable line's cable highlighted.
  if (overlays.ziplineFrontiers && !overlays.ziplineFrontiers->empty()) {
    // Region, cable and entry marker all match the regular move frontier.
    const glm::vec3 regionColor = debug.thermal ? thermalGreen : kSetupColor;
    const auto& frontiers = *overlays.ziplineFrontiers;
    std::vector<glm::vec4> key;
    key.emplace_back(static_cast<float>(thermalKey), 0.0f, 0.0f, 0.0f);
    for (const auto& fr : frontiers) {
      key.emplace_back(fr.field.minX, fr.field.minZ, fr.field.budget,
                       static_cast<float>(fr.zipline));
    }
    if (key != ziplineKey_) {
      ziplineKey_ = key;
      std::vector<ColorTriangleMesh::Vertex> fill;
      std::vector<glm::vec3> border;
      for (const auto& fr : frontiers) {
        if (fr.field.nx > 0) appendFrontier(fr.field, regionColor, fill, border);
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
    const float legAlpha = overlays.moveFrontierSubsequentLeg ? 0.5f : 1.0f;
    unlitShader_.SetVec4("uColor", glm::vec4(regionColor, legAlpha));
    ziplineBorder_.DrawSegments();
    glDisable(GL_POLYGON_OFFSET_FILL);
    for (const auto& fr : frontiers) {
      // The ring below overwrites uMVP/uColor, so restore them per line.
      unlitShader_.SetMat4("uMVP", viewProj);
      unlitShader_.SetVec4("uColor", glm::vec4(regionColor, legAlpha));
      pathLine_.SetPoints(cablePoints(game.GetScene().ziplines[fr.zipline]));
      pathLine_.Draw();
      DrawHighlightOnSurface(fr.entry, glm::vec4(regionColor, legAlpha));
    }
  }
  glDepthMask(GL_TRUE);
  glDisable(GL_BLEND);

  // Enemy ghosts: faint wireframes shown during the turn, fading after it.
  if (fogActive) {
    constexpr float kSightingMaxAlpha = 0.35f;
    unlitShader_.Use();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const Unit& unit : game.GetScene().units) {
      if (unit.team == team) continue;
      for (const auto& s : game.Sightings(team, unit.id)) {
        const float life = game.GhostAlpha(s.at);
        if (life <= 0.0f) continue;
        DrawSighting(unlitShader_, pathLine_, viewProj, unit, s, game.GetScene().sidewalks,
                     life * kSightingMaxAlpha, terrain);
      }
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  // Bullet tracers: a team-colored dotted path from the muzzle to where the
  // shot ended, fading out within 0.5 s of its shot.  A new
  // shot briefly overlays the dots with a full white-hot line.
  {
    constexpr float kTracerMaxAlpha = 0.9f;
    constexpr float kTracerDotLength = 0.04f;
    constexpr float kTracerDotStride = 0.2f;
    unlitShader_.Use();
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    for (const tactics::Tracer& tracer : game.Tracers()) {
      const float life = game.TracerAlpha(tracer);
      if (life <= 0.0f) continue;
      const glm::vec3 teamColor = tracer.team == Team::Blue ? glm::vec3(0.2f, 0.45f, 0.95f)
                                                            : glm::vec3(0.9f, 0.25f, 0.22f);
      const glm::vec3 delta = tracer.to - tracer.from;
      const float length = glm::length(delta);
      if (length <= 1e-4f) continue;
      const glm::vec3 direction = delta / length;
      std::vector<glm::vec3> dots;
      dots.reserve(static_cast<size_t>(std::ceil(length / kTracerDotStride)) * 2);
      for (float start = 0.0f; start < length; start += kTracerDotStride) {
        dots.push_back(tracer.from + direction * start);
        dots.push_back(tracer.from + direction * std::min(start + kTracerDotLength, length));
      }
      pathLine_.SetPoints(dots);
      unlitShader_.SetMat4("uMVP", viewProj);
      unlitShader_.SetVec4("uColor", glm::vec4(teamColor, life * kTracerMaxAlpha));
      pathLine_.DrawSegments();

      // Each shot of a burst still pops as a full white-hot line, but that
      // continuous overlay disappears quickly and leaves only the dots.
      constexpr float kTracerFlashSeconds = 0.25f;
      const float flash = glm::clamp(1.0f - game.TracerAge(tracer) / kTracerFlashSeconds, 0.0f, 1.0f);
      if (flash > 0.0f) {
        pathLine_.SetPoints({tracer.from, tracer.to});
        unlitShader_.SetVec4("uColor", glm::vec4(1.0f, 0.97f, 0.8f, flash));
        pathLine_.Draw();
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
      if (!unit.alive || (!overlays.spectatorFov && unit.team != team)) continue;
      if (!IsUnitVisibleForRender(unit, team, fogActive, visibility, game.GetScene())) continue;
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
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility, game.GetScene())) continue;
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
    const glm::vec4& rideColor = walkColor;
    unlitShader_.SetMat4("uMVP", viewProj);
    const auto drawRange = [&](size_t first, size_t last, const glm::vec4& color) {
      if (last <= first) return;
      pathLine_.SetPoints(liftedPath(std::vector<glm::vec3>(path.begin() + first,
                                                            path.begin() + last + 1)));
      unlitShader_.SetVec4("uColor", color);
      pathLine_.Draw();
      // GL line width is fixed at 1px, so thicken the path with parallel
      // copies shifted sideways (screen-right) by a few centimetres.
      const glm::vec3 right(view[0][0], view[1][0], view[2][0]);
      for (const float offset : {-0.06f, -0.03f, 0.03f, 0.06f}) {
        std::vector<glm::vec3> shifted = liftedPath(std::vector<glm::vec3>(
            path.begin() + first, path.begin() + last + 1));
        for (glm::vec3& p : shifted) p += right * offset;
        pathLine_.SetPoints(shifted);
        pathLine_.Draw();
      }
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
    const glm::vec4 previewColor(pathGreen, overlays.moveFrontierSubsequentLeg ? 0.5f : 1.0f);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    drawMovePath(*overlays.movePreviewPath,
                 overlays.movePreviewRides ? *overlays.movePreviewRides : kNoRides,
                 previewColor);
    // Mark the final position with the same ring used for selection.
    DrawHighlightOnSurface(overlays.movePreviewPath->back(), previewColor);
    glDisable(GL_BLEND);
  } else if (overlays.invalidHoverHighlight) {
    DrawHighlightOnSurface(*overlays.invalidHoverHighlight,
                  glm::vec4(0.9f, 0.15f, 0.15f, 1.0f));
  }
  // Visual feedback for the whole squad's plan so far: a planned move reuses
  // the same path-line rendering as the live preview above; a planned shot
  // gets a simple shooter->target line. Own team only -- the enemy's plans
  // stay hidden even where its figures are visible (except to the spectator).
  if (planning) {
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive || (!overlays.spectatorFov && unit.team != team)) continue;
      if (unit.plan.type == tactics::PlannedActionType::Move && unit.plan.movePath.size() >= 2) {
        drawMovePath(unit.plan.movePath, unit.plan.moveRides,
                     glm::vec4(pathGreen, 1.0f));
        // Wireframe stand-in at the destination, showing the planned final
        // facing (persists until the turn is committed).
        Unit ghost = unit;
        ghost.position = unit.plan.movePath.back();
        ghost.facingYaw = unit.plan.endFacingYaw;
        ghost.turnLag = 0.0f;
        // Common highlight green for both teams (darker in the black-hot view).
        DrawUnitWireframe(unlitShader_, pathLine_, viewProj, ghost,
                          debug.thermal ? glm::vec4(thermalGreen, 1.0f)
                                        : glm::vec4(0.3f, 0.9f, 0.4f, 1.0f));
        // Chained legs for later rounds: the same green at 50% alpha, with a
        // marker at each leg end so they read apart from the leg about to
        // execute (solid).
        const glm::vec4 later(pathGreen, 0.5f);
        const glm::vec4 firstRingColor(pathGreen, 1.0f);
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        const bool chaining = game.Mode() == InputMode::AwaitingMoveDestination &&
                              game.SelectedUnitId() == unit.id;
        if (chaining || !unit.plan.queuedLegs.empty()) {
          DrawNumberedHighlight(unlitShader_, highlightRing_, cubeMesh_, viewProj, view,
                                OnSurface(unit.plan.movePath.back()), firstRingColor, 1,
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
                       later);
          DrawNumberedHighlight(unlitShader_, highlightRing_, cubeMesh_, viewProj, view,
                                OnSurface(leg.back()), later, legNumber,
                                SurfaceNormal(leg.back()));
        }
        glDisable(GL_BLEND);
      }
      // Planned locked shots draw no line here: the shot cone aimed at the
      // target's torso (plus the "+" marker) is the whole preview, same as
      // a planned free-aim shot.
    }
  }
  glDisable(GL_SCISSOR_TEST);
}

}  // namespace gfx
