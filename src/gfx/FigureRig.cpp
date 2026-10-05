#include "gfx/FigureRig.h"

#include <array>
#include <cmath>

#include <glm/gtc/constants.hpp>
#include <glm/gtc/matrix_transform.hpp>

using tactics::Team;
using tactics::Unit;
using tactics::WeaponClass;
using tactics::WeaponType;

namespace gfx {
namespace {

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

constexpr float kTwoPi = 6.28318530717958647692f;

float SampleLoop(const BipedCurve& curve, float phase) {
  float wrapped = std::fmod(phase, kTwoPi);
  if (wrapped < 0.0f) wrapped += kTwoPi;
  const float cursor = wrapped * (kBipedKeyCount / kTwoPi);
  const int current = static_cast<int>(std::floor(cursor)) % kBipedKeyCount;
  const int next = (current + 1) % kBipedKeyCount;
  float t = cursor - std::floor(cursor);
  t = t * t * (3.0f - 2.0f * t);  // C1-continuous easing at each sampled key.
  return glm::mix(curve[current], curve[next], t);
}

// Shot beat (unit.shootElapsed over kShootAnimDuration):
//  stage 1, draw/aim: the weapon comes up from its carry pose to a level
//    aim (ease-out over kAimRaiseDuration) while twisting onto the target's
//    bearing;
//  stage 2, recoil: at the end of the raise the shot "fires" -- the weapon
//    kicks (muzzle flip + rearward slide), decaying exponentially (same
//    decay form as the camera zoom damping) -- and over the last
//    kLowerDuration everything eases back down to the carry pose.
constexpr float kAimRaiseDuration = 0.12f;
constexpr float kLowerDuration = 0.25f;
constexpr float kRecoilDecayRate = 12.0f;              // Per second.
constexpr float kRecoilArmKick = glm::radians(18.0f);  // Whole-arm lift (handgun).
constexpr float kRecoilMuzzleFlip = glm::radians(22.0f);
constexpr float kRecoilSlide = 0.07f;  // Weapon pushed back along its barrel.
// Rifles are braced with both hands and the shoulder, so their kick is a
// fraction of the two-handed hand-cannon's.
constexpr float kRifleRecoilScale = 0.4f;

// Aiming a rifle blades the upper body (right shoulder back) around the
// vertical axis: it reads as shouldering the stock and brings the left
// shoulder forward far enough that the short support arm can reach the
// handguard.
constexpr float kRifleAimBlade = glm::radians(35.0f);
// Where the head leans at full rifle aim (figure-local, before the twist
// onto the target bearing): back over the stock and down to the sight line,
// so the eyepiece/rear sight meets the front of the face and the figure
// reads as looking through the scope / down the sights.
const glm::vec3 kRifleAimHeadCenter(-0.14f, 1.30f, 0.03f);
// The pistol's gun arm angles inward as the aim comes up (the wrist
// counter-yaws so the barrel stays on the target bearing); together with
// the support hand joining the grip this makes a two-handed firing stance.
constexpr float kPistolAimInward = glm::radians(30.0f);
// The palm wraps the pistol grip this far below the grip top, so the fist
// reads as holding the grip rather than clamped around the slide.
constexpr float kPistolGripDrop = 0.12f;
// Where the support (left) hand clasps the pistol at full aim, in the
// weapon-local frame: front of the grip, slightly on the left side.
const glm::vec3 kPistolSupportHandLocal(0.02f, -0.10f, -0.05f);

float EaseOutQuad(float t) {
  t = glm::clamp(t, 0.0f, 1.0f);
  return 1.0f - (1.0f - t) * (1.0f - t);
}

// Everything the shot beat needs, sampled at the unit's shootElapsed.
struct ShootPose {
  float raise = 0.0f;   // 0 = weapon at the carry pose, 1 = fully aimed.
  float recoil = 0.0f;  // 1 at the instant of the shot, decaying to 0.
  float aimYawDelta = 0.0f;  // Twist toward the target, relative to facingYaw.
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
  pose.aimYawDelta = std::remainder(unit.shootAimYaw - unit.facingYaw, kTwoPi) * pose.raise;
  return pose;
}

// Zipline ride (unit.rideTravel along a cable of unit.rideLength). Mount
// (first kZiplineMountDistance): the figure runs up, jumps with the knees
// tucked, and its arms reach up to the trolley handle while its weapon is
// slung. Hang (middle): body hangs below the cable with the legs swinging
// forward on a downhill ride (gliding) and trailing back on an uphill one
// (being hauled up). Dismount (last kZiplineMountDistance): the legs reach
// out to land, the hands release and the weapon comes back up. Every phase
// is distance-driven, so it plays the same at any ride speed.
struct RidePose {
  bool riding = false;
  float hang = 0.0f;      // 0 = standing, 1 = fully hanging (body lifted, legs glide).
  float armsUp = 0.0f;    // 0 = arms in the carry pose, 1 = both hands on the handle.
  float lift = 0.0f;      // Body lift above the foot-to-foot line.
  float kick = 0.0f;      // Mount jump beat, 0..1..0.
  float land = 0.0f;      // Dismount landing beat, 0..1..0.
  float hip = 0.0f;       // Glide hip angle (before the mount/landing beats).
  float knee = 0.0f;
  float sway = 0.0f;      // Gentle leg swing, opposite on each leg.
};

float SmoothStep01(float t) {
  t = glm::clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}

RidePose SampleRidePose(const Unit& unit) {
  RidePose pose;
  if (!unit.alive || unit.rideTravel < 0.0f || unit.rideLength <= 0.0f) return pose;
  const float mount = tactics::constants::kZiplineMountDistance;
  const float travel = glm::clamp(unit.rideTravel, 0.0f, unit.rideLength);
  const float toEnd = unit.rideLength - travel;
  const float edge = std::min(travel, toEnd);
  pose.riding = true;
  pose.hang = SmoothStep01(edge / mount);
  pose.armsUp = SmoothStep01(edge / (0.6f * mount));
  pose.lift = tactics::constants::kZiplineHangLift * pose.hang;
  if (travel < mount) pose.kick = std::sin(glm::pi<float>() * travel / mount);
  if (toEnd < mount) pose.land = std::sin(glm::pi<float>() * toEnd / mount);
  // Downhill (slope < 0): legs swing forward. Uphill: they trail back.
  const float forward = 0.5f * (glm::clamp(-unit.rideSlope * 3.0f, -1.0f, 1.0f) + 1.0f);
  pose.hip = glm::radians(glm::mix(-6.0f, 26.0f, forward));
  pose.knee = glm::radians(glm::mix(-14.0f, -30.0f, forward));
  pose.sway = glm::radians(4.0f) * std::sin(travel * 1.3f + unit.id * 0.73f);
  return pose;
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
// axis first (yaw, same sign convention as facingYaw). The limb's own
// segment then extends from the pivot down the frame's -Y axis.
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

// ---------------------------------------------------------------------------
// Weapons (issue #126). Each is a handful of boxes in the weapon-local frame
// documented on BuildWeaponParts: origin at the grip top, +X toward the
// muzzle, +Y up. Proportions are sized against the ~1.7-unit figure.

const glm::vec4 kBlackMetal(0.12f, 0.12f, 0.14f, 1.0f);
const glm::vec4 kGunMetal(0.22f, 0.22f, 0.25f, 1.0f);
const glm::vec4 kDarkFurniture(0.30f, 0.30f, 0.34f, 1.0f);
const glm::vec4 kSteel(0.52f, 0.53f, 0.56f, 1.0f);
const glm::vec4 kOlive(0.27f, 0.30f, 0.20f, 1.0f);
const glm::vec4 kOliveDark(0.20f, 0.23f, 0.15f, 1.0f);

void AddBox(FigureParts* parts, const glm::mat4& frame, const glm::vec3& minCorner,
            const glm::vec3& size, const glm::vec4& color) {
  parts->push_back({frame * BoxModel(minCorner, size), color, FigurePrimitive::Box});
}

// Oversized blocky hand cannon: bulky steel slide over a dark frame, raked
// grip at the origin.
void AppendDesertEagle(FigureParts* parts, const glm::mat4& frame) {
  AddBox(parts, frame, {-0.10f, 0.020f, -0.038f}, {0.44f, 0.075f, 0.076f}, kSteel);  // Slide.
  AddBox(parts, frame, {-0.08f, -0.022f, -0.034f}, {0.30f, 0.048f, 0.068f}, kGunMetal);  // Frame.
  // Grip, raked back from the origin.
  const glm::mat4 grip = frame * glm::rotate(glm::mat4(1.0f), glm::radians(-12.0f),
                                             glm::vec3(0.0f, 0.0f, 1.0f));
  AddBox(parts, grip, {-0.095f, -0.175f, -0.030f}, {0.095f, 0.175f, 0.060f}, kBlackMetal);
  // Trigger guard: bottom bar + front bar.
  AddBox(parts, frame, {0.005f, -0.068f, -0.012f}, {0.075f, 0.018f, 0.024f}, kBlackMetal);
  AddBox(parts, frame, {0.062f, -0.060f, -0.012f}, {0.018f, 0.045f, 0.024f}, kBlackMetal);
  AddBox(parts, frame, {0.305f, 0.095f, -0.008f}, {0.020f, 0.022f, 0.016f}, kBlackMetal);  // Front sight.
}

// Carbine: boxy receiver with a top rail, chunky handguard, thin barrel with
// a front-sight block, straight stock, raked pistol grip, forward-tilted mag.
void AppendAssaultRifle(FigureParts* parts, const glm::mat4& frame) {
  AddBox(parts, frame, {-0.12f, -0.015f, -0.034f}, {0.30f, 0.100f, 0.068f}, kBlackMetal);  // Receiver.
  AddBox(parts, frame, {-0.10f, 0.085f, -0.020f}, {0.26f, 0.025f, 0.040f}, kGunMetal);  // Top rail.
  AddBox(parts, frame, {0.18f, -0.005f, -0.030f}, {0.18f, 0.078f, 0.060f}, kDarkFurniture);  // Handguard.
  AddBox(parts, frame, {0.36f, 0.022f, -0.014f}, {0.17f, 0.028f, 0.028f}, kGunMetal);  // Barrel.
  AddBox(parts, frame, {0.40f, 0.050f, -0.008f}, {0.025f, 0.055f, 0.016f}, kBlackMetal);  // Front sight.
  AddBox(parts, frame, {-0.345f, -0.005f, -0.028f}, {0.235f, 0.085f, 0.056f}, kDarkFurniture);  // Stock.
  AddBox(parts, frame, {-0.375f, -0.022f, -0.030f}, {0.032f, 0.120f, 0.060f}, kBlackMetal);  // Butt pad.
  const glm::mat4 grip = frame * glm::rotate(glm::mat4(1.0f), glm::radians(-15.0f),
                                             glm::vec3(0.0f, 0.0f, 1.0f));
  AddBox(parts, grip, {-0.040f, -0.160f, -0.026f}, {0.072f, 0.160f, 0.052f}, kBlackMetal);
  const glm::mat4 mag = frame * glm::rotate(glm::mat4(1.0f), glm::radians(10.0f),
                                            glm::vec3(0.0f, 0.0f, 1.0f));
  AddBox(parts, mag, {0.050f, -0.200f, -0.024f}, {0.075f, 0.190f, 0.048f}, kGunMetal);  // Magazine.
}

// Bolt-action long rifle: olive stock with a cheek riser, long thin barrel
// with a muzzle brake, and a scope (tube + objective bell + eyepiece) on
// mounts above the action.
void AppendSniperRifle(FigureParts* parts, const glm::mat4& frame) {
  AddBox(parts, frame, {-0.38f, -0.030f, -0.030f}, {0.50f, 0.095f, 0.060f}, kOlive);  // Stock/body.
  AddBox(parts, frame, {-0.33f, 0.062f, -0.026f}, {0.185f, 0.036f, 0.052f}, kOliveDark);  // Cheek riser.
  AddBox(parts, frame, {-0.412f, -0.042f, -0.032f}, {0.032f, 0.125f, 0.064f}, kBlackMetal);  // Butt pad.
  AddBox(parts, frame, {0.12f, 0.014f, -0.014f}, {0.46f, 0.028f, 0.028f}, kGunMetal);  // Barrel.
  AddBox(parts, frame, {0.565f, 0.004f, -0.019f}, {0.055f, 0.048f, 0.038f}, kBlackMetal);  // Muzzle brake.
  AddBox(parts, frame, {-0.12f, 0.102f, -0.021f}, {0.26f, 0.042f, 0.042f}, kBlackMetal);  // Scope tube.
  AddBox(parts, frame, {0.095f, 0.092f, -0.028f}, {0.070f, 0.058f, 0.056f}, kBlackMetal);  // Objective bell.
  AddBox(parts, frame, {-0.165f, 0.096f, -0.025f}, {0.050f, 0.050f, 0.050f}, kBlackMetal);  // Eyepiece.
  AddBox(parts, frame, {-0.065f, 0.058f, -0.012f}, {0.030f, 0.050f, 0.024f}, kGunMetal);  // Mount.
  AddBox(parts, frame, {0.035f, 0.058f, -0.012f}, {0.030f, 0.050f, 0.024f}, kGunMetal);  // Mount.
  AddBox(parts, frame, {-0.105f, 0.030f, 0.028f}, {0.035f, 0.022f, 0.048f}, kGunMetal);  // Bolt handle.
  const glm::mat4 grip = frame * glm::rotate(glm::mat4(1.0f), glm::radians(-18.0f),
                                             glm::vec3(0.0f, 0.0f, 1.0f));
  AddBox(parts, grip, {-0.042f, -0.150f, -0.025f}, {0.070f, 0.150f, 0.050f}, kOliveDark);
  AddBox(parts, frame, {0.055f, -0.105f, -0.022f}, {0.090f, 0.085f, 0.044f}, kBlackMetal);  // Magazine.
}

void AppendWeapon(FigureParts* parts, const glm::mat4& frame, WeaponType type,
                  bool compact) {
  if (compact) {
    // Single bounding box stand-in for wireframe ghosts (see
    // BuildFigureWireframe).
    glm::vec3 lo, hi;
    WeaponLocalBounds(type, &lo, &hi);
    parts->push_back({frame * BoxModel(lo, hi - lo), kGunMetal, FigurePrimitive::Box});
    return;
  }
  switch (type) {
    case WeaponType::DesertEagle: AppendDesertEagle(parts, frame); break;
    case WeaponType::AssaultRifle: AppendAssaultRifle(parts, frame); break;
    case WeaponType::SniperRifle: AppendSniperRifle(parts, frame); break;
  }
}

// Where the supporting (left) hand wraps the weapon, in the weapon-local
// frame: on the handguard (AR) / barrel ahead of the magazine (sniper), not
// back at the receiver.
glm::vec3 WeaponLeftHandLocal(WeaponType type) {
  return type == WeaponType::SniperRifle ? glm::vec3(0.22f, 0.02f, 0.0f)
                                         : glm::vec3(0.26f, 0.03f, 0.0f);
}

// ---------------------------------------------------------------------------
// Two-bone arm IK for the rifle carry: both hands are pinned to the weapon,
// so the arm is solved from the shoulder to the hand instead of being
// forward-posed from swing angles.

// Frame whose -Y axis runs from `origin` toward `toward` (the convention
// RoundedSegment extends along). The roll about that axis is arbitrary --
// the segments are rotationally symmetric.
glm::mat4 BoneFrame(const glm::vec3& origin, const glm::vec3& toward) {
  const glm::vec3 y = -glm::normalize(toward - origin);
  glm::vec3 helper = std::abs(y.x) < 0.9f ? glm::vec3(1.0f, 0.0f, 0.0f)
                                          : glm::vec3(0.0f, 0.0f, 1.0f);
  const glm::vec3 x = glm::normalize(glm::cross(y, helper));
  const glm::vec3 z = glm::cross(x, y);
  glm::mat4 frame(1.0f);
  frame[0] = glm::vec4(x, 0.0f);
  frame[1] = glm::vec4(y, 0.0f);
  frame[2] = glm::vec4(z, 0.0f);
  frame[3] = glm::vec4(origin, 1.0f);
  return frame;
}

// Upper arm + elbow ball + forearm from `shoulder` to (as near as reachable)
// `target`, all in the figure-local frame; the elbow bends toward `pole`.
// Appends the three parts transformed into `figure`'s space.
void AppendArmIK(FigureParts* parts, const glm::mat4& figure, const glm::vec3& shoulder,
                 const glm::vec3& target, const glm::vec3& pole, const glm::vec4& color) {
  const float a = kUpperArmLength;
  const float b = kLowerArmLength;
  glm::vec3 toTarget = target - shoulder;
  float length = glm::length(toTarget);
  if (length < 1e-5f) {
    toTarget = glm::vec3(0.0f, -1.0f, 0.0f);
    length = 1e-5f;
  }
  const glm::vec3 dir = toTarget / length;
  length = glm::clamp(length, std::abs(a - b) + 0.01f, a + b - 0.005f);

  glm::vec3 bend = pole - dir * glm::dot(pole, dir);
  if (glm::dot(bend, bend) < 1e-8f) {
    bend = glm::vec3(0.0f, -1.0f, 0.0f) - dir * glm::dot(glm::vec3(0.0f, -1.0f, 0.0f), dir);
  }
  bend = glm::normalize(bend);
  const float cosA = glm::clamp((a * a + length * length - b * b) / (2.0f * a * length),
                                -1.0f, 1.0f);
  const float sinA = std::sqrt(std::max(0.0f, 1.0f - cosA * cosA));
  const glm::vec3 elbow = shoulder + dir * (a * cosA) + bend * (a * sinA);
  const glm::vec3 hand = shoulder + dir * length;

  parts->push_back({RoundedSegment(figure * BoneFrame(shoulder, elbow), a, kArmRadius), color});
  parts->push_back({JointBall(figure * glm::translate(glm::mat4(1.0f), elbow), kArmRadius),
                    color});
  parts->push_back({RoundedSegment(figure * BoneFrame(elbow, hand), b, kArmRadius * 0.92f),
                    color});
}

}  // namespace

FigureParts BuildWeaponParts(WeaponType type) {
  FigureParts parts;
  AppendWeapon(&parts, glm::mat4(1.0f), type, /*compact=*/false);
  return parts;
}

void WeaponLocalBounds(WeaponType type, glm::vec3* outMin, glm::vec3* outMax) {
  const FigureParts parts = BuildWeaponParts(type);
  glm::vec3 lo(1e9f), hi(-1e9f);
  for (const FigurePart& part : parts) {
    for (int corner = 0; corner < 8; ++corner) {
      const glm::vec4 local(corner & 1 ? 1.0f : 0.0f, corner & 2 ? 1.0f : 0.0f,
                            corner & 4 ? 1.0f : 0.0f, 1.0f);
      const glm::vec3 p = glm::vec3(part.model * local);
      lo = glm::min(lo, p);
      hi = glm::max(hi, p);
    }
  }
  *outMin = lo;
  *outMax = hi;
}

float WeaponLength(WeaponType type) {
  switch (type) {
    case WeaponType::DesertEagle: return 0.54f;
    case WeaponType::AssaultRifle: return 0.91f;
    case WeaponType::SniperRifle: return 1.03f;
  }
  return 1.0f;
}

// Zipline trolley: a pulley block on the cable, with a strap down to the
// handlebar the rider's hands grip (`handleY`, figure-local).
void AppendTrolley(FigureParts* parts, const glm::mat4& figure, float cableY, float handleY) {
  const glm::vec4 steel = kSteel;
  AddBox(parts, figure, {-0.07f, cableY - 0.05f, -0.07f}, {0.20f, 0.10f, 0.14f}, steel);
  AddBox(parts, figure, {0.0f, handleY, -0.012f}, {0.024f, std::max(0.0f, cableY - handleY), 0.024f},
         kBlackMetal);
  AddBox(parts, figure, {0.0f, handleY - 0.02f, -0.19f}, {0.05f, 0.04f, 0.38f}, kBlackMetal);
}

FigureParts BuildFigureImpl(const Unit& unit, bool compactWeapon) {
  // Every body part shares the one flat team color; only the weapon differs.
  const glm::vec4 teamColor = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                      : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);

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
  const float leftHipWalk =
      glm::mix(idleLeftHip, glm::radians(SampleLoop(kWalkLeftHip, unit.walkPhase) * 0.55f), walk);
  const float rightHipWalk = glm::mix(
      idleRightHip, glm::radians(SampleLoop(kWalkRightHip, unit.walkPhase) * 0.55f), walk);
  const float leftKneeWalk = glm::mix(
      idleLeftKnee, glm::radians(SampleLoop(kWalkLeftKnee, unit.walkPhase) * 0.50f), walk);
  const float rightKneeWalk = glm::mix(
      idleRightKnee, glm::radians(SampleLoop(kWalkRightKnee, unit.walkPhase) * 0.50f), walk);
  const float bob = glm::mix(kIdleBobHeight * SampleLoop(kIdleBob, idlePhase),
                             kWalkBobHeight * SampleLoop(kWalkBob, unit.walkPhase), walk);

  const RidePose ride = SampleRidePose(unit);
  float leftHip = leftHipWalk, rightHip = rightHipWalk;
  float leftKnee = leftKneeWalk, rightKnee = rightKneeWalk;
  if (ride.riding) {
    leftHip = glm::mix(leftHip, ride.hip + ride.sway, ride.hang);
    rightHip = glm::mix(rightHip, ride.hip - ride.sway, ride.hang);
    leftKnee = glm::mix(leftKnee, ride.knee, ride.hang);
    rightKnee = glm::mix(rightKnee, ride.knee, ride.hang);
    // Mount: jump with the knees tucked. Dismount: reach out to land.
    const float hipBeat = glm::radians(24.0f) * ride.kick + glm::radians(32.0f) * ride.land;
    const float kneeBeat = glm::radians(-40.0f) * ride.kick + glm::radians(22.0f) * ride.land;
    leftHip += hipBeat;
    rightHip += hipBeat;
    leftKnee += kneeBeat;
    rightKnee += kneeBeat;
  }

  const ShootPose shot = SampleShootPose(unit);
  const glm::mat4 fall = KnockdownModel(unit);
  const glm::mat4 figure =
      fall * YawFrame(unit.position + glm::vec3(0.0f, bob + ride.lift, 0.0f), unit.facingYaw);

  // Rifle aim blades the torso/shoulders onto the target bearing plus
  // kRifleAimBlade and leans the head to the sight line; both follow
  // shot.raise so idle/run/knockdown are untouched.
  const bool rifleCarry = ClassOf(unit.weapon) != WeaponClass::Handgun;
  const float torsoYaw =
      rifleCarry ? shot.aimYawDelta + kRifleAimBlade * shot.raise : 0.0f;
  const glm::mat3 aimRot = glm::mat3(
      glm::rotate(glm::mat4(1.0f), shot.aimYawDelta, glm::vec3(0.0f, -1.0f, 0.0f)));
  glm::vec3 headCenter(0.0f, kHeadCenterHeight, 0.0f);
  if (rifleCarry) {
    headCenter = glm::mix(headCenter, aimRot * kRifleAimHeadCenter, shot.raise);
  }

  FigureParts parts;
  parts.reserve(36);
  // Riding: the trolley rides the cable (kZiplinePostHeight above the
  // foot-to-foot line, so below that in the lifted figure frame), the
  // handlebar hangs as far down its strap as the arms can reach.
  const float cableY = tactics::constants::kZiplinePostHeight - ride.lift;
  const float handleY = std::min(cableY - 0.12f, kShoulderHeight + kUpperArmLength + kLowerArmLength - 0.1f);
  const glm::vec3 handleRight(0.04f, handleY, 0.14f);
  const glm::vec3 handleLeft(0.04f, handleY, -0.14f);
  if (ride.riding) AppendTrolley(&parts, figure, cableY, handleY);
  parts.push_back({EllipsoidModel(
                       figure * glm::rotate(glm::mat4(1.0f), torsoYaw,
                                            glm::vec3(0.0f, -1.0f, 0.0f)),
                       glm::vec3(0.0f, kHipHeight + kTorsoHeight * 0.5f, 0.0f),
                       glm::vec3(kTorsoDepth * 0.5f, kTorsoHeight * 0.56f,
                                 kTorsoWidth * 0.5f)),
                   teamColor});
  parts.push_back({EllipsoidModel(figure, headCenter, glm::vec3(kHeadRadius)), teamColor});

  const glm::mat4 leftThigh =
      LimbFrame(figure, glm::vec3(0.0f, kHipHeight, -kLegSideOffset), leftHip);
  const glm::mat4 leftShin = ChildBoneFrame(leftThigh, kUpperLegLength, leftKnee);
  const glm::mat4 rightThigh =
      LimbFrame(figure, glm::vec3(0.0f, kHipHeight, kLegSideOffset), rightHip);
  const glm::mat4 rightShin = ChildBoneFrame(rightThigh, kUpperLegLength, rightKnee);
  parts.push_back({RoundedSegment(leftThigh, kUpperLegLength, kLegRadius), teamColor});
  parts.push_back({JointBall(leftShin, kLegRadius * 0.96f), teamColor});
  parts.push_back({RoundedSegment(leftShin, kLowerLegLength, kLegRadius * 0.94f), teamColor});
  parts.push_back({RoundedSegment(rightThigh, kUpperLegLength, kLegRadius), teamColor});
  parts.push_back({JointBall(rightShin, kLegRadius * 0.96f), teamColor});
  parts.push_back({RoundedSegment(rightShin, kLowerLegLength, kLegRadius * 0.94f), teamColor});

  if (ClassOf(unit.weapon) == WeaponClass::Handgun) {
    // One-handed carry (the original rig): the free left arm hangs/swings
    // with the gait, the gun arm swings too while running, snaps up to a
    // straight horizontal aim for the shot, and the pistol pitches between
    // low-ready (barrel forward while the arm hangs) and aim.
    const float leftShoulder =
        glm::radians(SampleLoop(kWalkLeftShoulder, unit.walkPhase) * 0.75f) * walk;
    const float rightShoulder =
        glm::radians(SampleLoop(kWalkRightShoulder, unit.walkPhase) * 0.75f) * walk;
    const float leftElbow = glm::mix(
        glm::radians(48.0f), glm::radians(SampleLoop(kWalkLeftElbow, unit.walkPhase) * 0.75f),
        walk);
    const float rightElbow = glm::mix(
        glm::radians(45.0f), glm::radians(SampleLoop(kWalkRightElbow, unit.walkPhase) * 0.78f),
        walk);

    // The gun arm's two bones straighten into a single forward line while
    // aiming; recoil layers on the shoulder after that blend.
    const float gunArmSwing = glm::mix(rightShoulder, glm::half_pi<float>(), shot.raise) +
                              kRecoilArmKick * shot.recoil;
    const float gunElbow = glm::mix(rightElbow, 0.0f, shot.raise);
    // Pistol in the hand: tilted 90 degrees so it points forward while the
    // arm hangs (low ready), aligning with the arm as the aim comes up, then
    // flipping up with the recoil.
    const float gunPitch =
        glm::half_pi<float>() * (1.0f - shot.raise) + kRecoilMuzzleFlip * shot.recoil;

    // Arm frames are built figure-local (identity base) so the weapon frame
    // can double as the IK target space for the support hand below.
    const glm::vec3 leftShoulderPivot(0.0f, kShoulderHeight, -kArmSideOffset);
    const glm::mat4 leftUpperArm =
        LimbFrame(glm::mat4(1.0f), leftShoulderPivot, leftShoulder, 0.0f, kArmSplay);
    const glm::mat4 leftForearm = ChildBoneFrame(leftUpperArm, kUpperArmLength, leftElbow);

    // The gun arm angles inward as the aim comes up so the support hand can
    // reach the grip; negative yawTwist turns it toward the body's center.
    const float inward = -kPistolAimInward * shot.raise;
    const glm::mat4 gunUpperArm =
        LimbFrame(glm::mat4(1.0f), glm::vec3(0.0f, kShoulderHeight, kArmSideOffset),
                  gunArmSwing, shot.aimYawDelta + inward, -kArmSplay * (1.0f - shot.raise));
    const glm::mat4 gunForearm = ChildBoneFrame(gunUpperArm, kUpperArmLength, gunElbow);
    if (ride.armsUp <= 0.0f) {
      parts.push_back({RoundedSegment(figure * gunUpperArm, kUpperArmLength, kArmRadius),
                       teamColor});
      parts.push_back({JointBall(figure * gunForearm, kArmRadius), teamColor});
      parts.push_back({RoundedSegment(figure * gunForearm, kLowerArmLength, kArmRadius * 0.92f),
                       teamColor});
    }

    // Pistol gripped at the hand (end of the arm). The weapon-local +X
    // (barrel) must run along the pitched hand frame's -Y, which
    // rotate(-90 deg about Z) provides; the wrist counter-yaws the inward
    // arm angle so the barrel stays on the aim bearing, the grip drops so
    // the fist wraps it below the slide, and recoil slides the gun back
    // along the barrel.
    glm::mat4 weaponFrame =
        gunForearm *
        glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -kLowerArmLength, 0.0f)) *
        glm::rotate(glm::mat4(1.0f), gunPitch, glm::vec3(0.0f, 0.0f, 1.0f)) *
        glm::rotate(glm::mat4(1.0f), -glm::half_pi<float>(), glm::vec3(0.0f, 0.0f, 1.0f)) *
        glm::rotate(glm::mat4(1.0f), -inward, glm::vec3(0.0f, -1.0f, 0.0f)) *
        glm::translate(glm::mat4(1.0f),
                       glm::vec3(-kRecoilSlide * shot.recoil, kPistolGripDrop, 0.0f));
    // Riding: the pistol goes to the hip holster while both hands grip the handle.
    weaponFrame[3] = glm::mix(weaponFrame[3], glm::vec4(0.05f, 0.5f, 0.34f, 1.0f), ride.armsUp);
    AppendWeapon(&parts, figure * weaponFrame, unit.weapon, compactWeapon);

    if (ride.armsUp > 0.0f) {
      const glm::vec3 gunHand =
          glm::vec3(gunForearm * glm::vec4(0.0f, -kLowerArmLength, 0.0f, 1.0f));
      const glm::vec3 swingHand =
          glm::vec3(leftForearm * glm::vec4(0.0f, -kLowerArmLength, 0.0f, 1.0f));
      AppendArmIK(&parts, figure, glm::vec3(0.0f, kShoulderHeight, kArmSideOffset),
                  glm::mix(gunHand, handleRight, ride.armsUp), glm::vec3(-0.3f, -0.6f, 0.8f),
                  teamColor);
      AppendArmIK(&parts, figure, leftShoulderPivot, glm::mix(swingHand, handleLeft, ride.armsUp),
                  glm::vec3(-0.3f, -0.6f, -0.8f), teamColor);
    } else if (unit.shootElapsed >= 0.0f) {
      // Two-handed firing stance: the support hand leaves its gait swing and
      // clasps the front of the grip as the gun comes up (and rides the
      // recoil with it, since the target lives in the weapon frame).
      const glm::vec3 swingHand =
          glm::vec3(leftForearm * glm::vec4(0.0f, -kLowerArmLength, 0.0f, 1.0f));
      const glm::vec3 supportHand =
          glm::vec3(weaponFrame * glm::vec4(kPistolSupportHandLocal, 1.0f));
      AppendArmIK(&parts, figure, leftShoulderPivot,
                  glm::mix(swingHand, supportHand, shot.raise),
                  glm::vec3(-0.6f, -0.8f, -0.3f), teamColor);
    } else {
      parts.push_back({RoundedSegment(figure * leftUpperArm, kUpperArmLength, kArmRadius),
                       teamColor});
      parts.push_back({JointBall(figure * leftForearm, kArmRadius), teamColor});
      parts.push_back({RoundedSegment(figure * leftForearm, kLowerArmLength,
                                      kArmRadius * 0.92f),
                       teamColor});
    }
  } else {
    // Two-handed rifle carry: the weapon frame is posed from the torso
    // (low-ready across the chest, blending up to a level chest-height aim
    // on the target's bearing), and both arms are IK-solved onto it, so
    // they never swing with the gait -- the distinct "run with rifle" look.
    // A slight yaw sway while running keeps the carry from looking welded.
    const float sway = glm::radians(5.0f) * std::sin(unit.walkPhase) * walk;
    // Low carry sits slightly left of center so the support hand reaches the
    // handguard; the aim grip rises toward the leaned head's sight line.
    const glm::vec3 gripLow(0.26f, 0.75f, -0.02f);
    const glm::vec3 gripAim(0.22f, 1.02f, 0.03f);
    // Riding: the rifle is slung muzzle-up across the back.
    const glm::vec3 gripSling(-0.26f, 0.62f, 0.10f);
    const glm::vec3 grip = glm::mix(glm::mix(gripLow, gripAim, shot.raise), gripSling, ride.armsUp);
    // rotate(+angle, +Y) yaws the muzzle toward the figure's left (-Z);
    // rotate(+angle, Z) pitches it up.
    const float weaponYaw =
        glm::mix(glm::radians(35.0f) + sway, 0.0f, shot.raise) * (1.0f - ride.armsUp);
    const float weaponPitch = glm::mix(glm::mix(glm::radians(-35.0f), 0.0f, shot.raise) +
                                           kRecoilMuzzleFlip * kRifleRecoilScale * shot.recoil,
                                       glm::radians(80.0f), ride.armsUp);
    // The whole weapon-and-arms assembly twists onto the target's bearing
    // around the torso's vertical axis (same sign convention as facingYaw).
    const glm::mat4 weaponFig =
        glm::rotate(glm::mat4(1.0f), shot.aimYawDelta, glm::vec3(0.0f, -1.0f, 0.0f)) *
        glm::translate(glm::mat4(1.0f), grip) *
        glm::rotate(glm::mat4(1.0f), weaponYaw, glm::vec3(0.0f, 1.0f, 0.0f)) *
        glm::rotate(glm::mat4(1.0f), weaponPitch, glm::vec3(0.0f, 0.0f, 1.0f)) *
        glm::translate(glm::mat4(1.0f),
                       glm::vec3(-kRecoilSlide * kRifleRecoilScale * shot.recoil, 0.0f, 0.0f));
    AppendWeapon(&parts, figure * weaponFig, unit.weapon, compactWeapon);

    const glm::vec3 rightHand =
        glm::mix(glm::vec3(weaponFig * glm::vec4(0.0f, -0.03f, 0.0f, 1.0f)), handleRight,
                 ride.armsUp);
    const glm::vec3 leftHand =
        glm::mix(glm::vec3(weaponFig * glm::vec4(WeaponLeftHandLocal(unit.weapon), 1.0f)),
                 handleLeft, ride.armsUp);
    // The IK shoulders ride the bladed torso (torsoYaw), which swings the
    // left shoulder forward toward the handguard while aiming.
    const glm::mat3 shoulderRot = glm::mat3(
        glm::rotate(glm::mat4(1.0f), torsoYaw, glm::vec3(0.0f, -1.0f, 0.0f)));
    AppendArmIK(&parts, figure,
                shoulderRot * glm::vec3(0.0f, kShoulderHeight, kArmSideOffset), rightHand,
                glm::vec3(-0.2f, -1.0f, 0.5f), teamColor);
    AppendArmIK(&parts, figure,
                shoulderRot * glm::vec3(0.0f, kShoulderHeight, -kArmSideOffset), leftHand,
                glm::vec3(-0.2f, -1.0f, -0.5f), teamColor);
  }
  return parts;
}

FigureParts BuildFigure(const Unit& unit) { return BuildFigureImpl(unit, false); }

FigureParts BuildFigureWireframe(const Unit& unit) { return BuildFigureImpl(unit, true); }

}  // namespace gfx
