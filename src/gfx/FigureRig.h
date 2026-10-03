#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Unit.h"
#include "game/Weapon.h"

namespace gfx {

// Procedural humanoid figure + weapon construction (issue #126), shared by
// the in-game SceneRenderer (solid, depth and wireframe passes) and the
// asset/animation gallery. Pure math: no GL calls, so it links into any
// target that has glm.

enum class FigurePrimitive { Rounded, Box };

// One drawable piece of a posed figure or weapon: a world (or caller-frame)
// model matrix over either the shared unit sphere (Rounded) or the shared
// unit cube spanning [0,1]^3 (Box).
struct FigurePart {
  glm::mat4 model{1.0f};
  glm::vec4 color{1.0f};
  FigurePrimitive primitive = FigurePrimitive::Rounded;
};

using FigureParts = std::vector<FigurePart>;

// Poses the whole figure (body + carried weapon) for the unit's current
// animation state and returns each part's world model matrix with its color.
// The carry/aim pose follows ClassOf(unit.weapon): rifles are held
// two-handed (arms solved onto the weapon, no run swing), the handgun
// one-handed at low ready with both arms swinging while running.
FigureParts BuildFigure(const tactics::Unit& unit);

// The weapon alone, in its local frame: origin at the top of the grip
// (where the hand wraps), +X toward the muzzle, +Y up, +Z the weapon's
// right-hand side. Used by the gallery's weapon turntables; BuildFigure
// places the same parts into the figure's hands.
FigureParts BuildWeaponParts(tactics::WeaponType type);

// Overall muzzle-to-butt length of a weapon model, for gallery framing.
float WeaponLength(tactics::WeaponType type);

}  // namespace gfx
