#pragma once

#include <glm/glm.hpp>

namespace gfx {

struct Ray {
  glm::vec3 origin{0.0f};
  glm::vec3 direction{0.0f, 0.0f, -1.0f};
};

// Bird's-eye orbit camera: rotates and zooms around a fixed ground-plane
// focus point. No panning/free-fly, matching the Stage-A spec.
class OrbitCamera {
 public:
  void Rotate(float deltaYawRadians, float deltaPitchRadians);
  void Zoom(float deltaDistance);

  glm::vec3 Position() const;
  glm::mat4 ViewMatrix() const;
  glm::mat4 ProjectionMatrix(float aspectRatio) const;

  // Unprojects a screen-space point (pixels, origin top-left) into a world
  // ray from the camera through that pixel.
  Ray ScreenPointToRay(float screenX, float screenY, float screenWidth, float screenHeight) const;

  // Intersects a ray with the y=0 ground plane. Returns false if the ray is
  // parallel to the plane or points away from it.
  static bool IntersectGroundPlane(const Ray& ray, glm::vec3* outPoint);

  glm::vec3 target{0.0f, 0.0f, 0.0f};

 private:
  float yaw_ = -0.9f;                    // Radians around Y.
  float pitch_ = 0.9599f;                // ~55 degrees above horizon.
  float distance_ = 18.0f;
  static constexpr float kMinPitch = 0.4363f;   // ~25 degrees.
  static constexpr float kMaxPitch = 1.4835f;   // ~85 degrees.
  static constexpr float kMinDistance = 6.0f;
  static constexpr float kMaxDistance = 35.0f;
};

}  // namespace gfx
