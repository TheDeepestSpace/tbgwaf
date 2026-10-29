#include "gallery/GalleryScene.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <GLES3/gl3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "game/Types.h"
#include "game/Weapon.h"
#include "gfx/FigureRig.h"

using tactics::Unit;
using tactics::WeaponType;

namespace gallery {
namespace {

constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kTurntableSeconds = 8.0f;
// One full gait cycle at the game's default run: walkPhase advances 2*pi per
// kWalkStrideLength world units covered at kMoveSpeed.
constexpr float kRunCycleSeconds =
    tactics::constants::kWalkStrideLength / tactics::constants::kMoveSpeed;
// Demo zipline ride: a 12-unit cable at a 0.6 slope, covered in kZipRideSeconds.
constexpr float kZipRideLength = 12.0f;
constexpr float kZipRideSlope = 0.6f;
constexpr float kZipRideSeconds = 3.0f;

// Same single directional light as the game scene, minus the shadow map:
// the gallery shows one model on an empty stage, so shadows would only
// add cross-driver noise to the goldens.
const char* kLitVertexShaderSrc = R"(#version 300 es
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

const char* kLitFragmentShaderSrc = R"(#version 300 es
precision highp float;
in vec3 vWorldPos;
in vec3 vWorldNormal;
uniform vec4 uColor;
uniform vec3 uLightDir;  // Direction the light travels; surfaces face -uLightDir.
uniform vec3 uViewPos;
out vec4 FragColor;
void main() {
  vec3 normal = normalize(vWorldNormal);
  vec3 viewDir = normalize(uViewPos - vWorldPos);
  if (dot(normal, viewDir) < 0.0) normal = -normal;
  float diffuse = max(dot(normal, -uLightDir), 0.0);
  const float kAmbient = 0.35;
  float lit = kAmbient + diffuse * 0.65;
  FragColor = vec4(uColor.rgb * lit, uColor.a);
}
)";

const glm::vec3 kLightDir = glm::normalize(glm::vec3(0.35f, -1.0f, 0.25f));
const glm::vec4 kGroundColor(0.16f, 0.18f, 0.20f, 1.0f);

// Empty-mag animation (issue #140): shot k leaves at k * the weapon's fire
// interval, the same pacing a committed burst uses in the game.
constexpr float kEmptyMagHoldSeconds = 1.0f;  // Rest after the last shot.
constexpr float kTracerRange = 6.0f;          // How far the demo bullets fly.

float EmptyMagDuration(tactics::WeaponType weapon) {
  const tactics::WeaponStats& stats = tactics::StatsOf(weapon);
  return (stats.magazineSize - 1) * stats.shotIntervalSeconds +
         tactics::constants::kShootAnimDuration + kEmptyMagHoldSeconds;
}

// Shots fired by loop time `t` (0 before the first) and the shoot-beat time
// the shooter is at, replaying GameLogic's rule: a follow-up shot landing
// while the beat still plays only re-triggers the recoil kick, otherwise the
// weapon is drawn afresh.
struct EmptyMagState {
  int fired = 0;
  float shootElapsed = -1.0f;
};

EmptyMagState SampleEmptyMag(tactics::WeaponType weapon, double t) {
  const tactics::WeaponStats& stats = tactics::StatsOf(weapon);
  EmptyMagState state;
  if (t < 0.0) return state;
  const int last = std::min(stats.magazineSize - 1,
                            static_cast<int>(std::floor(t / stats.shotIntervalSeconds)));
  float elapsed = -1.0f;
  for (int i = 0; i <= last; ++i) {
    if (i > 0 && elapsed >= 0.0f) {
      elapsed += stats.shotIntervalSeconds;
      if (elapsed >= tactics::constants::kShootAnimDuration) elapsed = -1.0f;
    }
    elapsed = elapsed >= 0.0f ? tactics::constants::kShootRecoilStart : 0.0f;
  }
  elapsed += static_cast<float>(t - last * stats.shotIntervalSeconds);
  state.fired = last + 1;
  state.shootElapsed = elapsed < tactics::constants::kShootAnimDuration ? elapsed : -1.0f;
  return state;
}

// End point of bullet `index`: a uniformly scattered ray inside the weapon's
// cone (same hash as the game), flown kTracerRange from the muzzle.
glm::vec3 BulletEnd(const Unit& unit, int index) {
  const float scatter =
      glm::radians(tactics::StatsOf(unit.weapon).scatterHalfAngleDegrees);
  const float radius = std::sqrt(tactics::ScatterUnit(0, 1, index, 1)) * scatter;
  const float theta = kTwoPi * tactics::ScatterUnit(0, 1, index, 2);
  const float yaw = radius * std::cos(theta);
  const float pitch = radius * std::sin(theta);
  const glm::vec3 dir(std::cos(pitch) * std::cos(yaw), std::sin(pitch),
                      std::cos(pitch) * std::sin(yaw));
  return unit.MuzzlePosition() + dir * kTracerRange;
}

// A thin box from `a` to `b` (tracer line) as a lit part.
gfx::FigurePart TracerPart(const glm::vec3& a, const glm::vec3& b, const glm::vec4& color) {
  constexpr float kThickness = 0.02f;
  const glm::vec3 delta = b - a;
  const float length = glm::length(delta);
  const glm::vec3 dir = delta / length;
  // Box local +X runs along the line.
  const glm::vec3 side = glm::normalize(glm::cross(dir, glm::vec3(0.0f, 1.0f, 0.0f)));
  const glm::vec3 up = glm::cross(side, dir);
  glm::mat4 frame(1.0f);
  frame[0] = glm::vec4(dir, 0.0f);
  frame[1] = glm::vec4(up, 0.0f);
  frame[2] = glm::vec4(side, 0.0f);
  frame[3] = glm::vec4(a, 1.0f);
  gfx::FigurePart part;
  part.model = frame * glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, -kThickness * 0.5f, -kThickness * 0.5f)) *
               glm::scale(glm::mat4(1.0f), glm::vec3(length, kThickness, kThickness));
  part.color = color;
  part.primitive = gfx::FigurePrimitive::Box;
  return part;
}

}  // namespace

const std::vector<GalleryItem>& Catalog() {
  static const std::vector<GalleryItem> items = [] {
    std::vector<GalleryItem> v;
    struct W {
      WeaponType type;
      const char* slug;
      const char* label;
    };
    const W weapons[] = {
        {WeaponType::DesertEagle, "deagle", "Desert Eagle"},
        {WeaponType::AssaultRifle, "ar", "Assault rifle"},
        {WeaponType::SniperRifle, "sniper", "Sniper rifle"},
    };
    for (const W& w : weapons) {
      v.push_back({w.slug, w.label, ItemKind::WeaponModel, w.type, AnimKind::Idle,
                   kTurntableSeconds});
    }
    struct A {
      AnimKind kind;
      const char* slug;
      const char* label;
      float duration;
    };
    const A anims[] = {
        {AnimKind::Idle, "idle", "idle", tactics::constants::kIdleAnimDuration},
        {AnimKind::Run, "run", "run", kRunCycleSeconds},
        {AnimKind::Shoot, "shoot", "shoot", tactics::constants::kShootAnimDuration},
        {AnimKind::ZipDown, "zip_down", "zipline (downhill)", kZipRideSeconds},
        {AnimKind::ZipUp, "zip_up", "zipline (uphill)", kZipRideSeconds},
        {AnimKind::EmptyMag, "empty_mag", "empty mag", 0.0f},  // Per weapon, below.
    };
    for (const W& w : weapons) {
      for (A a : anims) {
        if (a.kind == AnimKind::EmptyMag) a.duration = EmptyMagDuration(w.type);
        // Leaked once at startup; ids/labels must outlive the catalog.
        char* id = new char[64];
        char* label = new char[64];
        std::snprintf(id, 64, "%s_%s", w.slug, a.slug);
        std::snprintf(label, 64, "%s - %s", w.label, a.label);
        v.push_back({id, label, ItemKind::Animation, w.type, a.kind, a.duration});
      }
    }
    return v;
  }();
  return items;
}

glm::vec3 ViewState::Position() const {
  return target + distance * glm::vec3(std::cos(pitchRadians) * std::cos(yawRadians),
                                       std::sin(pitchRadians),
                                       std::cos(pitchRadians) * std::sin(yawRadians));
}

glm::mat4 ViewState::ViewMatrix() const {
  return glm::lookAt(Position(), target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 ViewState::ProjectionMatrix(float aspectRatio) const {
  return glm::perspective(glm::radians(45.0f), aspectRatio, 0.05f, 100.0f);
}

ViewState DefaultView(int itemIndex) {
  const GalleryItem& item = Catalog()[itemIndex];
  ViewState view;
  if (item.kind == ItemKind::WeaponModel) {
    glm::vec3 lo, hi;
    gfx::WeaponLocalBounds(item.weapon, &lo, &hi);
    view.target = (lo + hi) * 0.5f;
    // Side profile (+X muzzle pointing screen-right), close enough to fill
    // the frame at any weapon length.
    view.yawRadians = glm::radians(90.0f);
    view.pitchRadians = glm::radians(10.0f);
    view.distance = glm::length(hi - lo) * 1.15f;
  } else {
    // Front-right three-quarter view of the whole figure (it faces +X).
    view.target = glm::vec3(0.0f, 0.8f, 0.0f);
    view.yawRadians = glm::radians(30.0f);
    view.pitchRadians = glm::radians(14.0f);
    view.distance = 3.6f;
    if (item.anim == AnimKind::EmptyMag) {
      // Over the shooter's shoulder, so the fan of scattered bullet lines
      // downrange (+X) toward the backstop reads as a spread.
      view.target = glm::vec3(kTracerRange * 0.45f, 1.0f, 0.0f);
      view.yawRadians = glm::radians(200.0f);
      view.pitchRadians = glm::radians(22.0f);
      view.distance = 7.5f;
    }
  }
  return view;
}

Unit AnimationUnit(const GalleryItem& item, double t) {
  Unit unit;
  unit.id = 0;
  unit.team = tactics::Team::Blue;
  unit.position = glm::vec3(0.0f);
  unit.facingYaw = 0.0f;
  unit.weapon = item.weapon;
  switch (item.anim) {
    case AnimKind::Idle:
      unit.idleElapsed = static_cast<float>(t);
      break;
    case AnimKind::Run:
      unit.walkBlend = 1.0f;
      unit.walkPhase = static_cast<float>(kTwoPi * t / kRunCycleSeconds);
      break;
    case AnimKind::Shoot:
      unit.shootElapsed =
          static_cast<float>(std::fmod(t, tactics::constants::kShootAnimDuration));
      unit.shootAimYaw = unit.facingYaw;
      break;
    case AnimKind::ZipDown:
    case AnimKind::ZipUp:
      unit.idleElapsed = static_cast<float>(t);
      unit.rideLength = kZipRideLength;
      unit.rideTravel = static_cast<float>(std::fmod(t, kZipRideSeconds) / kZipRideSeconds) *
                        kZipRideLength;
      unit.rideSlope = item.anim == AnimKind::ZipDown ? -kZipRideSlope : kZipRideSlope;
      break;
    case AnimKind::EmptyMag:
      unit.shootElapsed = SampleEmptyMag(item.weapon, t).shootElapsed;
      unit.shootAimYaw = unit.facingYaw;
      break;
  }
  return unit;
}

bool GalleryRenderer::Init() {
  if (!litShader_.Compile(kLitVertexShaderSrc, kLitFragmentShaderSrc)) {
    return false;
  }
  cubeMesh_.Init();
  sphereMesh_.Init();
  glEnable(GL_DEPTH_TEST);
  return true;
}

void GalleryRenderer::Destroy() {
  sphereMesh_.Destroy();
  cubeMesh_.Destroy();
}

void GalleryRenderer::Render(int itemIndex, double t, const ViewState& view, int width,
                             int height) {
  const GalleryItem& item = Catalog()[itemIndex];

  glViewport(0, 0, width, height);
  glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  const glm::mat4 viewProj =
      view.ProjectionMatrix(static_cast<float>(width) / static_cast<float>(height)) *
      view.ViewMatrix();
  litShader_.Use();
  litShader_.SetVec3("uLightDir", kLightDir);
  litShader_.SetVec3("uViewPos", view.Position());

  gfx::FigureParts parts;
  if (item.kind == ItemKind::WeaponModel) {
    // Turntable: one revolution per item.duration around the vertical axis
    // through the weapon's center.
    const float yaw = static_cast<float>(kTwoPi * t / item.duration);
    glm::vec3 lo, hi;
    gfx::WeaponLocalBounds(item.weapon, &lo, &hi);
    const glm::vec3 center = (lo + hi) * 0.5f;
    const glm::mat4 spin = glm::translate(glm::mat4(1.0f), center) *
                           glm::rotate(glm::mat4(1.0f), yaw, glm::vec3(0.0f, 1.0f, 0.0f)) *
                           glm::translate(glm::mat4(1.0f), -center);
    parts = gfx::BuildWeaponParts(item.weapon);
    for (gfx::FigurePart& part : parts) part.model = spin * part.model;
  } else {
    // Stage floor under the figure, matching the game's ground color.
    gfx::FigurePart ground;
    ground.model = glm::translate(glm::mat4(1.0f), glm::vec3(-1.6f, -0.05f, -1.6f)) *
                   glm::scale(glm::mat4(1.0f), glm::vec3(3.2f, 0.05f, 3.2f));
    ground.color = kGroundColor;
    ground.primitive = gfx::FigurePrimitive::Box;
    parts.push_back(ground);
    const Unit unit = AnimationUnit(item, t);
    if (item.anim == AnimKind::EmptyMag) {
      // Longer stage with a backstop plate, plus one tracer per bullet fired
      // so far; the newest flashes white-hot like in the game.
      parts.back().model =
          glm::translate(glm::mat4(1.0f), glm::vec3(-1.6f, -0.05f, -1.6f)) *
          glm::scale(glm::mat4(1.0f), glm::vec3(kTracerRange + 2.4f, 0.05f, 3.2f));
      gfx::FigurePart plate;
      plate.model = glm::translate(glm::mat4(1.0f), glm::vec3(kTracerRange + 0.1f, 0.0f, -1.2f)) *
                    glm::scale(glm::mat4(1.0f), glm::vec3(0.08f, 2.0f, 2.4f));
      plate.color = glm::vec4(0.35f, 0.18f, 0.16f, 1.0f);
      plate.primitive = gfx::FigurePrimitive::Box;
      parts.push_back(plate);
      const EmptyMagState mag = SampleEmptyMag(item.weapon, t);
      const tactics::WeaponStats& stats = tactics::StatsOf(item.weapon);
      Unit aimed = unit;
      aimed.facingYaw = unit.shootAimYaw;
      for (int i = 0; i < mag.fired; ++i) {
        const float age = static_cast<float>(t) - i * stats.shotIntervalSeconds;
        const float flash = glm::clamp(1.0f - age / 0.25f, 0.0f, 1.0f);
        const glm::vec4 color(glm::mix(glm::vec3(0.2f, 0.45f, 0.95f),
                                       glm::vec3(1.0f, 0.97f, 0.8f), flash),
                              1.0f);
        parts.push_back(TracerPart(aimed.MuzzlePosition(), BulletEnd(aimed, i), color));
      }
    }
    const gfx::FigureParts figure = gfx::BuildFigure(unit);
    parts.insert(parts.end(), figure.begin(), figure.end());
  }

  for (const gfx::FigurePart& part : parts) {
    litShader_.SetMat4("uModel", part.model);
    litShader_.SetMat4("uMVP", viewProj * part.model);
    litShader_.SetVec4("uColor", part.color);
    if (part.primitive == gfx::FigurePrimitive::Rounded) {
      sphereMesh_.Draw();
    } else {
      cubeMesh_.Draw();
    }
  }
}

}  // namespace gallery
