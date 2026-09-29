#pragma once

#include <cmath>

#include <glm/glm.hpp>

#include "game/Types.h"

namespace tactics {

// A standing order a figure can arm on its turn, to react automatically
// during an enemy's move instead of acting immediately. `None` is the
// default (no reaction); `Shoot` is the overwatch PoC. Left room to extend
// with more reactions later.
enum class TriggerAction { None, Shoot };

struct Unit {
  int id = -1;
  Team team = Team::Blue;
  glm::vec3 position{0.0f};  // Feet position; unit box spans [position, position + (0,height,0)].
  float facingYaw = 0.0f;    // Radians, measured from +X axis in the XZ plane.
  bool alive = true;
  TriggerAction triggerAction = TriggerAction::None;

  glm::vec3 EyePosition() const {
    return position + glm::vec3(0.0f, constants::kEyeHeight, 0.0f);
  }

  glm::vec3 FacingDirection() const {
    return glm::vec3(std::cos(facingYaw), 0.0f, std::sin(facingYaw));
  }

  AABB Bounds() const {
    glm::vec3 halfExtents(constants::kUnitHalfWidth, 0.0f, constants::kUnitHalfWidth);
    glm::vec3 base = position - halfExtents;
    glm::vec3 top = position + halfExtents + glm::vec3(0.0f, constants::kUnitHeight, 0.0f);
    return AABB{base, top};
  }
};

}  // namespace tactics
