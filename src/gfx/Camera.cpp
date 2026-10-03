#include "gfx/Camera.h"

#include <algorithm>
#include <cmath>

#include <glm/gtc/matrix_transform.hpp>

namespace gfx {

void OrbitCamera::Rotate(float deltaYawRadians, float deltaPitchRadians) {
  focusing_ = false;
  yaw_ += deltaYawRadians;
  pitch_ = std::clamp(pitch_ + deltaPitchRadians, kMinPitch, kMaxPitch);
}

void OrbitCamera::Zoom(float deltaDistance) {
  targetDistance_ = std::clamp(targetDistance_ + deltaDistance, kMinDistance, kMaxDistance);
}

void OrbitCamera::Update(float dt) {
  const float blend = std::min(1.0f, kZoomDampingRate * dt);
  distance_ += (targetDistance_ - distance_) * blend;
  if (focusing_) {
    target += (focusTarget_ - target) * blend;
    pitch_ += (focusPitch_ - pitch_) * blend;
    if (glm::length(focusTarget_ - target) < 0.01f && std::fabs(focusPitch_ - pitch_) < 1e-3f) {
      target = focusTarget_;
      pitch_ = focusPitch_;
      focusing_ = false;
    }
  }
}

void OrbitCamera::FocusOn(const glm::vec3& focusPoint, float halfExtent) {
  focusing_ = true;
  focusTarget_ = glm::vec3(focusPoint.x, 0.0f, focusPoint.z);
  focusPitch_ = kMaxPitch;
  // Same margin as FitToExtent.
  targetDistance_ = std::clamp(halfExtent * 2.4f, kMinDistance, kMaxDistance);
}

void OrbitCamera::FitToExtent(float halfExtent) {
  // Margin covers the narrow half-width pane and the tilted view.
  distance_ = targetDistance_ = std::clamp(halfExtent * 2.4f, kMinDistance, kMaxDistance);
}

void OrbitCamera::Pan(float deltaRight, float deltaForward) {
  focusing_ = false;
  const glm::vec3 forward(-std::cos(yaw_), 0.0f, -std::sin(yaw_));
  const glm::vec3 right(std::sin(yaw_), 0.0f, -std::cos(yaw_));
  target += (right * deltaRight + forward * deltaForward) * distance_;
}

glm::vec3 OrbitCamera::Position() const {
  const float horizontalRadius = distance_ * std::cos(pitch_);
  return target + glm::vec3(horizontalRadius * std::cos(yaw_), distance_ * std::sin(pitch_),
                             horizontalRadius * std::sin(yaw_));
}

glm::mat4 OrbitCamera::ViewMatrix() const {
  return glm::lookAt(Position(), target, glm::vec3(0.0f, 1.0f, 0.0f));
}

glm::mat4 OrbitCamera::ProjectionMatrix(float aspectRatio) const {
  return glm::perspective(glm::radians(45.0f), aspectRatio, 0.5f, 400.0f);
}

Ray OrbitCamera::ScreenPointToRay(float screenX, float screenY, float screenWidth,
                                   float screenHeight) const {
  const float ndcX = (2.0f * screenX) / screenWidth - 1.0f;
  const float ndcY = 1.0f - (2.0f * screenY) / screenHeight;

  const glm::mat4 view = ViewMatrix();
  const glm::mat4 proj = ProjectionMatrix(screenWidth / screenHeight);
  const glm::mat4 invVp = glm::inverse(proj * view);

  glm::vec4 nearPoint = invVp * glm::vec4(ndcX, ndcY, -1.0f, 1.0f);
  glm::vec4 farPoint = invVp * glm::vec4(ndcX, ndcY, 1.0f, 1.0f);
  nearPoint /= nearPoint.w;
  farPoint /= farPoint.w;

  Ray ray;
  ray.origin = glm::vec3(nearPoint);
  ray.direction = glm::normalize(glm::vec3(farPoint - nearPoint));
  return ray;
}

bool OrbitCamera::IntersectGroundPlane(const Ray& ray, glm::vec3* outPoint) {
  if (std::fabs(ray.direction.y) < 1e-6f) return false;
  const float t = -ray.origin.y / ray.direction.y;
  if (t < 0.0f) return false;
  if (outPoint) *outPoint = ray.origin + ray.direction * t;
  return true;
}

}  // namespace gfx
