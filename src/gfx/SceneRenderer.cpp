#include "gfx/SceneRenderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
    if (game.MovePreviewValid()) {
      overlays.movePreviewPath = &game.MovePreviewPath();
    } else if (hoveredGroundPoint) {
      overlays.invalidHoverHighlight = hoveredGroundPoint;
    }
  }
  return overlays;
}
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

// ---------------------------------------------------------------------------
// Procedural humanoid figure. The visible skin is assembled from overlapping
// ellipsoids over a small hierarchical biped rig. Upper/lower limbs have
// separate shoulder/elbow and hip/knee transforms, so the same rig can be
// sampled by idle, locomotion, aim, recoil, and knockdown animation layers.
//
// Figure-local frame: +X forward (FacingDirection), +Y up, +Z the figure's
// right-hand side. A positive local-Z joint rotation moves a hanging bone
// forward. The rounded segments overlap at every joint, hiding seams and
// making the silhouette read as one soft body like the original sketch.

constexpr float kUpperLegLength = 0.22f;
constexpr float kLowerLegLength = 0.22f;
constexpr float kLegRadius = 0.14f;
constexpr float kLegSideOffset = 0.16f;
constexpr float kHipHeight = 0.42f;
constexpr float kTorsoHeight = 0.57f;
constexpr float kTorsoWidth = 0.62f;  // Side to side (Z).
constexpr float kTorsoDepth = 0.46f;  // Front to back (X).
// Keep the head centered where the old ellipsoid was, but use its smaller
// front-to-back radius on every axis so the head is a true sphere.
constexpr float kHeadCenterHeight = 1.375f;
constexpr float kHeadRadius = 0.72f * 0.5f;
constexpr float kShoulderHeight = 0.93f;
constexpr float kUpperArmLength = 0.26f;
constexpr float kLowerArmLength = 0.23f;
constexpr float kArmRadius = 0.11f;
constexpr float kArmSideOffset = kTorsoWidth * 0.48f;
constexpr float kArmSplay = glm::radians(28.0f);
constexpr float kGunLength = 0.36f;
constexpr float kGunThickness = 0.08f;

// Compact retarget of the Idle and Walking clips from RobotExpressive.glb
// (Tomás Laulhé / Quaternius, CC0 1.0). The pinned source and extraction
// details live in THIRD_PARTY.md. Eight samples per loop are enough for this
// small on-screen figure; SampleLoop smooths between them. Angles are degrees
// in the source rig's sagittal plane, then scaled below for the doll's very
// short limbs.
constexpr int kBipedKeyCount = 8;
using BipedCurve = std::array<float, kBipedKeyCount>;
constexpr BipedCurve kWalkLeftHip = {46.2f, 63.7f, 28.2f, -17.1f,
                                     -6.6f, 16.4f, 64.4f, 69.2f};
constexpr BipedCurve kWalkLeftKnee = {-24.6f, -61.1f, -36.6f, -6.7f,
                                      -46.2f, -78.8f, -104.7f, -80.5f};
constexpr BipedCurve kWalkRightHip = {-35.9f, -18.1f, 20.2f, 81.6f,
                                      58.7f, 45.2f, -0.9f, -9.0f};
constexpr BipedCurve kWalkRightKnee = {-34.6f, -66.8f, -97.7f, -110.8f,
                                       -41.7f, -48.1f, -14.4f, -38.2f};
constexpr BipedCurve kWalkLeftShoulder = {-21.6f, -20.4f, -12.2f, -0.7f,
                                          6.6f, 4.3f, -3.8f, -11.8f};
constexpr BipedCurve kWalkLeftElbow = {65.4f, 65.4f, 65.4f, 65.8f,
                                       65.7f, 65.7f, 65.5f, 65.7f};
constexpr BipedCurve kWalkRightShoulder = {-5.1f, -2.7f, -8.0f, -21.4f,
                                           -29.8f, -35.9f, -28.2f, -14.6f};
constexpr BipedCurve kWalkRightElbow = {60.0f, 60.4f, 58.3f, 56.8f,
                                        56.1f, 56.2f, 56.6f, 57.5f};
constexpr BipedCurve kWalkBob = {0.4f, 0.0f, 0.9f, 0.6f, 0.2f, 0.5f, 1.0f, 0.8f};
constexpr BipedCurve kIdleLeftHip = {16.6f, 22.8f, 26.8f, 21.9f,
                                     17.0f, 21.6f, 26.9f, 23.2f};
constexpr BipedCurve kIdleLeftKnee = {-31.7f, -41.9f, -48.9f, -40.5f,
                                      -31.7f, -39.9f, -48.7f, -42.7f};
constexpr BipedCurve kIdleRightHip = {16.1f, 22.2f, 26.2f, 21.4f,
                                      16.5f, 21.0f, 26.3f, 22.7f};
constexpr BipedCurve kIdleRightKnee = {-31.0f, -41.0f, -47.8f, -39.6f,
                                       -31.0f, -39.0f, -47.6f, -41.7f};
constexpr BipedCurve kIdleBob = {1.0f, 0.4f, 0.0f, 0.5f, 1.0f, 0.6f, 0.0f, 0.4f};
constexpr float kWalkBobHeight = 0.04f;
constexpr float kIdleBobHeight = 0.008f;

float SampleLoop(const BipedCurve& curve, float phase) {
  constexpr float kTwoPi = 6.28318530717958647692f;
  float wrapped = std::fmod(phase, kTwoPi);
  if (wrapped < 0.0f) wrapped += kTwoPi;
  const float cursor = wrapped * (kBipedKeyCount / kTwoPi);
  const int current = static_cast<int>(std::floor(cursor)) % kBipedKeyCount;
  const int next = (current + 1) % kBipedKeyCount;
  float t = cursor - std::floor(cursor);
  t = t * t * (3.0f - 2.0f * t);  // C1-continuous easing at each sampled key.
  return glm::mix(curve[current], curve[next], t);
}

// Quick-draw pistol beat (unit.shootElapsed over kShootAnimDuration):
//  stage 1, draw/aim: the gun arm snaps from hanging at the side up to a
//    straight horizontal aim (ease-out over kAimRaiseDuration) while the
//    pistol levels from its low-ready forward tilt to align with the arm,
//    and the whole arm twists at the shoulder onto the target's bearing;
//  stage 2, recoil: at the end of the raise the shot "fires" -- the arm and
//    muzzle kick upward and the pistol slides back into the hand, all
//    decaying exponentially (same decay form as the camera zoom damping)
//    -- and over the last kLowerDuration the arm eases back down to rest.
constexpr float kAimRaiseDuration = 0.12f;
constexpr float kLowerDuration = 0.25f;
constexpr float kRecoilDecayRate = 12.0f;             // Per second.
constexpr float kRecoilArmKick = glm::radians(18.0f);  // Whole-arm lift.
constexpr float kRecoilMuzzleFlip = glm::radians(22.0f);
constexpr float kRecoilSlide = 0.07f;  // Pistol pushed back along its barrel.

float EaseOutQuad(float t) {
  t = glm::clamp(t, 0.0f, 1.0f);
  return 1.0f - (1.0f - t) * (1.0f - t);
}

// Everything the quick-draw beat needs, sampled at the unit's shootElapsed.
struct ShootPose {
  float raise = 0.0f;   // 0 = arm hanging at rest, 1 = fully extended aim.
  float recoil = 0.0f;  // 1 at the instant of the shot, decaying to 0.
  float aimYawDelta = 0.0f;  // Shoulder twist toward the target, relative to facingYaw.
};

ShootPose SampleShootPose(const Unit& unit) {
  ShootPose pose;
  if (unit.shootElapsed < 0.0f) return pose;
  const float t = unit.shootElapsed;
  const float lowerStart = tactics::constants::kShootAnimDuration - kLowerDuration;
  if (t < kAimRaiseDuration) {
    pose.raise = EaseOutQuad(t / kAimRaiseDuration);
  } else {
    pose.raise = 1.0f - EaseOutQuad((t - lowerStart) / kLowerDuration);
    pose.recoil = std::exp(-kRecoilDecayRate * (t - kAimRaiseDuration));
  }
  constexpr float kTwoPi = 6.28318530717958647692f;
  pose.aimYawDelta = std::remainder(unit.shootAimYaw - unit.facingYaw, kTwoPi) * pose.raise;
  return pose;
}

// unit.position / unit.facingYaw as a frame. The rotation axis is (0,-1,0)
// rather than the more usual (0,1,0): FacingDirection() defines "forward"
// directly as (cos(yaw), 0, sin(yaw)) rather than via a rotation matrix, and
// glm::rotate(yaw, {0,1,0}) turns the local +X axis into (cos(yaw), 0,
// -sin(yaw)) -- the mirror image. Negating the axis cancels that sign flip
// so the figure visually faces the same way as its FOV cone.
glm::mat4 YawFrame(const glm::vec3& origin, float yaw) {
  return glm::translate(glm::mat4(1.0f), origin) *
         glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, -1.0f, 0.0f));
}

glm::mat4 BoxModel(const glm::vec3& minCorner, const glm::vec3& size) {
  return glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
}

glm::mat4 EllipsoidModel(const glm::mat4& frame, const glm::vec3& center,
                         const glm::vec3& radii) {
  return frame * glm::translate(glm::mat4(1.0f), center) *
         glm::scale(glm::mat4(1.0f), radii);
}

// Frame of a limb hanging from `pivot` (in the figure frame), swung forward
// by `swing` radians about the pivot, optionally twisted about the vertical
// axis first (yaw, same sign convention as facingYaw). The limb's own box
// then extends from the pivot down the frame's -Y axis.
// `splay` tilts the hanging limb sideways about the forward (X) axis before
// the swing: positive moves the limb toward the figure's left (-Z), so each
// side negates it to splay outward.
glm::mat4 LimbFrame(const glm::mat4& figure, const glm::vec3& pivot, float swing,
                    float yawTwist = 0.0f, float splay = 0.0f) {
  return figure * glm::translate(glm::mat4(1.0f), pivot) *
         glm::rotate(glm::mat4(1.0f), yawTwist, glm::vec3(0.0f, -1.0f, 0.0f)) *
         glm::rotate(glm::mat4(1.0f), splay, glm::vec3(1.0f, 0.0f, 0.0f)) *
         glm::rotate(glm::mat4(1.0f), swing, glm::vec3(0.0f, 0.0f, 1.0f));
}

glm::mat4 ChildBoneFrame(const glm::mat4& parent, float parentLength, float bend) {
  return parent * glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -parentLength, 0.0f)) *
         glm::rotate(glm::mat4(1.0f), bend, glm::vec3(0.0f, 0.0f, 1.0f));
}

glm::mat4 RoundedSegment(const glm::mat4& bone, float length, float radius) {
  // A little extra length makes adjacent ellipsoids overlap at the joint.
  return EllipsoidModel(bone, glm::vec3(0.0f, -length * 0.5f, 0.0f),
                        glm::vec3(radius, length * 0.56f, radius));
}

glm::mat4 JointBall(const glm::mat4& joint, float radius) {
  return EllipsoidModel(joint, glm::vec3(0.0f), glm::vec3(radius));
}

glm::mat4 HangingBox(const glm::mat4& limb, float length, float thickness) {
  return limb * BoxModel(glm::vec3(-thickness * 0.5f, -length, -thickness * 0.5f),
                         glm::vec3(thickness, length, thickness));
}

enum class FigurePrimitive { Rounded, Box };

struct FigurePart {
  glm::mat4 model{1.0f};
  glm::vec4 color{1.0f};
  FigurePrimitive primitive = FigurePrimitive::Rounded;
};

constexpr int kFigurePartCount = 15;
using FigureParts = std::array<FigurePart, kFigurePartCount>;

// Poses the whole figure for the unit's current animation state and returns
// each part's world model matrix (knockdown tip-over included) with its color.
FigureParts BuildFigure(const Unit& unit) {
  // Every body part shares the one flat team color; only the pistol differs.
  const glm::vec4 teamColor = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                      : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  constexpr glm::vec4 kGunColor(0.12f, 0.12f, 0.12f, 1.0f);

  constexpr float kTwoPi = 6.28318530717958647692f;
  const float idlePhase = unit.idleElapsed * (kTwoPi / tactics::constants::kIdleAnimDuration) +
                          unit.id * 0.73f;
  const float walk = unit.walkBlend;

  // Retarget the source's human proportions onto the short plush rig. Idle
  // deltas are intentionally tiny; they keep a standing figure alive without
  // making it visibly shuffle. Locomotion preserves the captured asymmetry,
  // knee flex, bent elbows, and double-support timing instead of reducing the
  // motion to mirrored sine waves.
  const float idleLeftHip = glm::radians((SampleLoop(kIdleLeftHip, idlePhase) - 21.9f) * 0.16f);
  const float idleRightHip =
      glm::radians((SampleLoop(kIdleRightHip, idlePhase) - 21.4f) * 0.16f);
  const float idleLeftKnee =
      glm::radians(-12.0f + (SampleLoop(kIdleLeftKnee, idlePhase) + 40.7f) * 0.12f);
  const float idleRightKnee =
      glm::radians(-12.0f + (SampleLoop(kIdleRightKnee, idlePhase) + 40.0f) * 0.12f);
  const float leftHip =
      glm::mix(idleLeftHip, glm::radians(SampleLoop(kWalkLeftHip, unit.walkPhase) * 0.55f), walk);
  const float rightHip = glm::mix(
      idleRightHip, glm::radians(SampleLoop(kWalkRightHip, unit.walkPhase) * 0.55f), walk);
  const float leftKnee = glm::mix(
      idleLeftKnee, glm::radians(SampleLoop(kWalkLeftKnee, unit.walkPhase) * 0.50f), walk);
  const float rightKnee = glm::mix(
      idleRightKnee, glm::radians(SampleLoop(kWalkRightKnee, unit.walkPhase) * 0.50f), walk);
  const float leftShoulder =
      glm::radians(SampleLoop(kWalkLeftShoulder, unit.walkPhase) * 0.75f) * walk;
  const float rightShoulder =
      glm::radians(SampleLoop(kWalkRightShoulder, unit.walkPhase) * 0.75f) * walk;
  const float leftElbow = glm::mix(
      glm::radians(48.0f), glm::radians(SampleLoop(kWalkLeftElbow, unit.walkPhase) * 0.75f), walk);
  const float rightElbow = glm::mix(
      glm::radians(45.0f), glm::radians(SampleLoop(kWalkRightElbow, unit.walkPhase) * 0.78f), walk);
  const float bob = glm::mix(kIdleBobHeight * SampleLoop(kIdleBob, idlePhase),
                             kWalkBobHeight * SampleLoop(kWalkBob, unit.walkPhase), walk);

  const ShootPose shot = SampleShootPose(unit);
  // The right arm's two bones straighten into a single forward line while
  // aiming; recoil layers on the shoulder after that blend.
  const float gunArmSwing = glm::mix(rightShoulder, glm::half_pi<float>(), shot.raise) +
                            kRecoilArmKick * shot.recoil;
  const float gunElbow = glm::mix(rightElbow, 0.0f, shot.raise);
  // Pistol in the hand: tilted 90 degrees so it points forward while the
  // arm hangs (low ready), aligning with the arm as the aim comes up, then
  // flipping up with the recoil.
  const float gunPitch =
      glm::half_pi<float>() * (1.0f - shot.raise) + kRecoilMuzzleFlip * shot.recoil;

  const glm::mat4 fall = KnockdownModel(unit);
  const glm::mat4 figure =
      fall * YawFrame(unit.position + glm::vec3(0.0f, bob, 0.0f), unit.facingYaw);

  FigureParts parts;
  parts[0] = {EllipsoidModel(figure, glm::vec3(0.0f, kHipHeight + kTorsoHeight * 0.5f, 0.0f),
                             glm::vec3(kTorsoDepth * 0.5f, kTorsoHeight * 0.56f,
                                       kTorsoWidth * 0.5f)),
              teamColor};
  parts[1] = {EllipsoidModel(
                  figure, glm::vec3(0.0f, kHeadCenterHeight, 0.0f), glm::vec3(kHeadRadius)),
              teamColor};

  const glm::mat4 leftThigh =
      LimbFrame(figure, glm::vec3(0.0f, kHipHeight, -kLegSideOffset), leftHip);
  const glm::mat4 leftShin = ChildBoneFrame(leftThigh, kUpperLegLength, leftKnee);
  const glm::mat4 rightThigh =
      LimbFrame(figure, glm::vec3(0.0f, kHipHeight, kLegSideOffset), rightHip);
  const glm::mat4 rightShin = ChildBoneFrame(rightThigh, kUpperLegLength, rightKnee);
  parts[2] = {RoundedSegment(leftThigh, kUpperLegLength, kLegRadius), teamColor};
  parts[3] = {JointBall(leftShin, kLegRadius * 0.96f), teamColor};
  parts[4] = {RoundedSegment(leftShin, kLowerLegLength, kLegRadius * 0.94f), teamColor};
  parts[5] = {RoundedSegment(rightThigh, kUpperLegLength, kLegRadius), teamColor};
  parts[6] = {JointBall(rightShin, kLegRadius * 0.96f), teamColor};
  parts[7] = {RoundedSegment(rightShin, kLowerLegLength, kLegRadius * 0.94f), teamColor};

  const glm::mat4 leftUpperArm =
      LimbFrame(figure, glm::vec3(0.0f, kShoulderHeight, -kArmSideOffset), leftShoulder,
                0.0f, kArmSplay);
  const glm::mat4 leftForearm = ChildBoneFrame(leftUpperArm, kUpperArmLength, leftElbow);
  parts[8] = {RoundedSegment(leftUpperArm, kUpperArmLength, kArmRadius), teamColor};
  parts[9] = {JointBall(leftForearm, kArmRadius), teamColor};
  parts[10] = {RoundedSegment(leftForearm, kLowerArmLength, kArmRadius * 0.92f), teamColor};

  const glm::mat4 gunUpperArm =
      LimbFrame(figure, glm::vec3(0.0f, kShoulderHeight, kArmSideOffset), gunArmSwing,
                shot.aimYawDelta, -kArmSplay * (1.0f - shot.raise));
  const glm::mat4 gunForearm = ChildBoneFrame(gunUpperArm, kUpperArmLength, gunElbow);
  parts[11] = {RoundedSegment(gunUpperArm, kUpperArmLength, kArmRadius), teamColor};
  parts[12] = {JointBall(gunForearm, kArmRadius), teamColor};
  parts[13] = {RoundedSegment(gunForearm, kLowerArmLength, kArmRadius * 0.92f), teamColor};

  // Pistol: gripped at the hand (end of the arm), barrel extending along the
  // hand frame's -Y once pitched, recoil sliding it back toward the hand.
  const glm::mat4 gun = gunForearm *
                        glm::translate(glm::mat4(1.0f),
                                       glm::vec3(0.0f, -kLowerArmLength, 0.0f)) *
                        glm::rotate(glm::mat4(1.0f), gunPitch, glm::vec3(0.0f, 0.0f, 1.0f)) *
                        glm::translate(glm::mat4(1.0f),
                                       glm::vec3(0.0f, kRecoilSlide * shot.recoil, 0.0f));
  parts[14] = {HangingBox(gun, kGunLength, kGunThickness), kGunColor, FigurePrimitive::Box};
  return parts;
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
  for (const FigurePart& part : BuildFigure(unit)) {
    DrawFigureWirePart(shader, lines, viewProj, part, color);
  }
}

// Faded, team-colored wireframe of a remembered sighting plus a floor arrow
// along its movement direction (if it was moving).
void DrawSighting(const Shader& shader, LineMesh& lines, const glm::mat4& viewProj,
                  const Unit& sighted, const GameLogic::EnemySighting& s, float alpha) {
  const glm::vec4 base = sighted.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                    : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  const glm::vec4 color(base.r, base.g, base.b, alpha);
  Unit ghost = sighted;
  ghost.position = s.position;
  ghost.facingYaw = s.facingYaw;
  ghost.knockdownElapsed = -1.0f;
  if (glm::length(s.moveDirection) > 0.0f) {
    const glm::vec3 d = s.moveDirection;
    const glm::vec3 side(-d.z, 0.0f, d.x);
    const glm::vec3 tail = s.position + glm::vec3(0.0f, 0.02f, 0.0f);
    const glm::vec3 tip = tail + d * 1.2f;
    const std::vector<glm::vec3> arrow = {tail, tip, tip - d * 0.3f + side * 0.2f, tip,
                                          tip - d * 0.3f - side * 0.2f};
    lines.SetPoints(arrow);
    shader.SetMat4("uMVP", viewProj);
    shader.SetVec4("uColor", color);
    lines.Draw();
  }
  for (const FigurePart& part : BuildFigure(ghost)) {
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

void DrawHighlight(const Shader& shader, const TriangleMesh& ring, const glm::mat4& viewProj,
                   const glm::vec3& position, const glm::vec4& color) {
  const glm::mat4 model = glm::translate(glm::mat4(1.0f), position + glm::vec3(0.0f, 0.02f, 0.0f));
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetVec4("uColor", color);
  ring.Draw();
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
  if (!colorShader_.Compile(kColorVertexShaderSrc, kColorFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the vertex-color shader\n");
    return false;
  }
  cubeMesh_.Init();
  sphereMesh_.Init();
  frontierFill_.Init();
  frontierBorder_.Init();
  pathLine_.Init();
  fovConeMesh_.Init();
  highlightRing_.Init();
  highlightRing_.SetPoints(BuildRingPoints());

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
  frontierFill_.Destroy();
  frontierBorder_.Destroy();
  pathLine_.Destroy();
  fovConeMesh_.Destroy();
  highlightRing_.Destroy();
  sphereMesh_.Destroy();
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
    DrawUnitDepth(depthShader_, cubeMesh_, sphereMesh_, lightSpaceMatrix_, unit);
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
    const AABB& bounds = obstacles[i].bounds;
    const glm::vec4 color(0.55f, 0.55f, 0.6f, 1.0f);
    DrawBoxLit(litShader_, cubeMesh_, viewProj, lightSpaceMatrix_, bounds.min,
               bounds.max - bounds.min, color);
  }

  for (const Unit& unit : game.GetScene().units) {
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawUnit(litShader_, cubeMesh_, sphereMesh_, viewProj, lightSpaceMatrix_, unit);
  }

  // Rings sit on whatever flat slab (sidewalk) is under them, not inside it.
  const auto DrawHighlightOnSurface = [&](glm::vec3 position, const glm::vec4& color) {
    for (const AABB& slab : game.GetScene().sidewalks) {
      if (position.x >= slab.min.x && position.x <= slab.max.x && position.z >= slab.min.z &&
          position.z <= slab.max.z) {
        position.y = std::max(position.y, slab.max.y);
      }
    }
    DrawHighlight(unlitShader_, highlightRing_, viewProj, position, color);
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
      constexpr float kY = 0.03f;
      constexpr float kFadeWidth = 0.8f;   // World units from boundary to transparent.
      constexpr float kEdgeAlpha = 0.65f;  // Fill alpha right at the boundary.
      const int nx = f.nx, nz = f.nz;
      const float inf = std::numeric_limits<float>::infinity();

      // Chamfer distance (in world units) from each node to the nearest node
      // of the opposite reached/unreached class.
      auto chamfer = [&](bool target) {
        std::vector<float> d(static_cast<size_t>(nx) * nz, inf);
        for (int iz = 0; iz < nz; ++iz)
          for (int ix = 0; ix < nx; ++ix)
            if (f.Reached(ix, iz) == target) d[iz * nx + ix] = 0.0f;
        const float s = f.step, sd = f.step * 1.41421356f;
        auto relax = [&](int ix, int iz, int dx, int dz, float w) {
          const int jx = ix + dx, jz = iz + dz;
          if (jx < 0 || jz < 0 || jx >= nx || jz >= nz) return;
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
          g[i] = f.Reached(ix, iz) ? dIn[i] - 0.5f * f.step : -(dOut[i] - 0.5f * f.step);
        }
      // Separable box blur (radius 2, two passes) rounds off the grid steps.
      std::vector<float> tmp(g.size());
      for (int pass = 0; pass < 2; ++pass) {
        for (int iz = 0; iz < nz; ++iz)
          for (int ix = 0; ix < nx; ++ix) {
            float sum = 0.0f;
            for (int k = -2; k <= 2; ++k) sum += g[iz * nx + std::clamp(ix + k, 0, nx - 1)];
            tmp[iz * nx + ix] = sum / 5.0f;
          }
        for (int iz = 0; iz < nz; ++iz)
          for (int ix = 0; ix < nx; ++ix) {
            float sum = 0.0f;
            for (int k = -2; k <= 2; ++k) sum += tmp[std::clamp(iz + k, 0, nz - 1) * nx + ix];
            g[iz * nx + ix] = sum / 5.0f;
          }
      }

      struct Pt {
        glm::vec3 p;
        float g;
      };
      std::vector<ColorTriangleMesh::Vertex> fill;
      std::vector<glm::vec3> border;
      auto toVertex = [&](const Pt& q) {
        const float a = kEdgeAlpha * std::clamp(1.0f - q.g / kFadeWidth, 0.0f, 1.0f);
        return ColorTriangleMesh::Vertex{q.p, glm::vec4(0.2f, 0.9f, 0.3f, a)};
      };
      for (int iz = 0; iz + 1 < nz; ++iz) {
        for (int ix = 0; ix + 1 < nx; ++ix) {
          const int cx[4] = {ix, ix + 1, ix + 1, ix};
          const int cz[4] = {iz, iz, iz + 1, iz + 1};
          Pt quad[4];
          bool anyIn = false;
          for (int k = 0; k < 4; ++k) {
            quad[k] = {f.Node(cx[k], cz[k]), g[cz[k] * nx + cx[k]]};
            quad[k].p.y = kY;
            anyIn |= quad[k].g > 0.0f;
          }
          if (!anyIn) continue;
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
        }
      }
      frontierFill_.SetVertices(fill);
      frontierBorder_.SetPoints(border);
    }
    colorShader_.Use();
    colorShader_.SetMat4("uMVP", viewProj);
    frontierFill_.Draw();
    unlitShader_.Use();
    unlitShader_.SetMat4("uMVP", viewProj);
    unlitShader_.SetVec4("uColor", glm::vec4(0.2f, 1.0f, 0.3f, 1.0f));
    frontierBorder_.DrawSegments();
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
        DrawSighting(unlitShader_, pathLine_, viewProj, unit, s, life * kSightingMaxAlpha);
      }
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

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
      DrawHighlightOnSurface(unit.position, color);
    }
  }
  // Overwatch indicator: a minimal PoC-grade ground marker (distinct from
  // the plan-then-commit ring and the yellow selection ring) under
  // every figure currently armed to fire during an enemy's move.
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.triggerAction != tactics::TriggerAction::Shoot) continue;
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawHighlightOnSurface(unit.position,
                  glm::vec4(1.0f, 0.55f, 0.0f, 1.0f));
  }
  // Playbook indicator: magenta ring on every figure with a standing
  // shoot-on-FOV-entry rule (distinct from the orange one-shot overwatch ring).
  for (const Unit& unit : game.GetScene().units) {
    if (!unit.alive || unit.reactionOnStationary != tactics::ReactionRule::Shoot) continue;
    if (!IsUnitVisibleForRender(unit, team, fogActive, visibility)) continue;
    DrawHighlight(unlitShader_, highlightRing_, viewProj, unit.position,
                  glm::vec4(0.85f, 0.1f, 0.85f, 1.0f));
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
  if (overlays.movePreviewPath && overlays.movePreviewPath->size() >= 2) {
    pathLine_.SetPoints(*overlays.movePreviewPath);
    unlitShader_.SetMat4("uMVP", viewProj);
    unlitShader_.SetVec4("uColor", glm::vec4(1.0f, 0.85f, 0.2f, 1.0f));
    pathLine_.Draw();
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
