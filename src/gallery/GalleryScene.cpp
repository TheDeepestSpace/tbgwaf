#include "gallery/GalleryScene.h"

#include <cmath>
#include <cstdio>

#include <GLES3/gl3.h>
#include <glm/gtc/matrix_transform.hpp>

#include "game/Types.h"
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
    };
    for (const W& w : weapons) {
      for (const A& a : anims) {
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
    const gfx::FigureParts figure = gfx::BuildFigure(AnimationUnit(item, t));
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
