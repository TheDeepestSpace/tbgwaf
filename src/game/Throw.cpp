#include "game/Throw.h"

#include <algorithm>
#include <cmath>

namespace tactics {

float ThrowDistance(const Unit& unit, const glm::vec3& point) {
  return std::hypot(point.x - unit.position.x, point.z - unit.position.z);
}

bool CanThrowAt(const Unit& unit, const glm::vec3& point) {
  return unit.alive && unit.grenades > 0 && std::isfinite(point.x) && std::isfinite(point.y) &&
         std::isfinite(point.z) && ThrowDistance(unit, point) <= constants::kThrowRange;
}

ThrowPreview PredictThrow(const Unit& unit, const glm::vec3& point) {
  ThrowPreview out;
  out.landing = point;
  const glm::vec3 from = unit.position + glm::vec3(0.0f, constants::kThrowReleaseHeight, 0.0f);
  const float apex = std::max(from.y, point.y) + constants::kThrowArcApexLift;
  // Quadratic through (0, from.y), (1, point.y) peaking at `apex` between
  // them: y(t) = from.y + (point.y - from.y) t + k t (1 - t), with k set so
  // the lift over the straight chord at its midpoint reaches the apex.
  const float chordMid = 0.5f * (from.y + point.y);
  const float k = 4.0f * (apex - chordMid);
  const int n = constants::kThrowArcSamples;
  out.arc.reserve(static_cast<size_t>(n) + 1);
  for (int i = 0; i <= n; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(n);
    glm::vec3 p = glm::mix(from, point, t);
    p.y = from.y + (point.y - from.y) * t + k * t * (1.0f - t);
    out.arc.push_back(p);
  }
  return out;
}

}  // namespace tactics
