#pragma once

namespace tactics {

// Weapon carried by a figure (issue #126). Purely visual for now: the model
// in the figure's hands and which carry/aim animation class the rig uses.
// Hit resolution is unchanged and identical across weapons.
enum class WeaponType { DesertEagle, AssaultRifle, SniperRifle };

// Animation class: rifles are carried two-handed (both hands on the weapon,
// no arm swing while running); handguns one-handed at low ready with the
// off-weapon arm swinging normally.
enum class WeaponClass { Handgun, Rifle };

inline WeaponClass ClassOf(WeaponType type) {
  return type == WeaponType::DesertEagle ? WeaponClass::Handgun : WeaponClass::Rifle;
}

// Deterministic default loadout: every squad of three fields an assault
// rifle, a sniper rifle, and a Desert Eagle, keyed off the unit id so the
// same unit always carries the same weapon in every client/test/golden.
inline WeaponType DefaultWeaponForUnit(int unitId) {
  switch (((unitId % 3) + 3) % 3) {
    case 0: return WeaponType::AssaultRifle;
    case 1: return WeaponType::SniperRifle;
    default: return WeaponType::DesertEagle;
  }
}

}  // namespace tactics
