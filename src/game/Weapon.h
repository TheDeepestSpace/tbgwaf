#pragma once

#include <algorithm>
#include <cmath>

namespace tactics {

// Weapon carried by a figure (issue #126): the model in the figure's hands
// and which carry/aim animation class the rig uses. Since issue #138 the
// weapon also bounds how many shots one shooting action may fire (see
// WeaponStats); per-shot hit resolution itself is still identical across
// weapons.
enum class WeaponType { DesertEagle, AssaultRifle, SniperRifle };

// Animation class: rifles are carried two-handed (both hands on the weapon,
// no arm swing while running); handguns one-handed at low ready with the
// off-weapon arm swinging normally.
enum class WeaponClass { Handgun, Rifle };

inline WeaponClass ClassOf(WeaponType type) {
  return type == WeaponType::DesertEagle ? WeaponClass::Handgun : WeaponClass::Rifle;
}

// Multi-shot bursts (issue #138): one shooting action may fire up to a full
// magazine, further capped by how many aimed shots fit in the round's fixed
// execution window at the weapon's fire interval. There is no persistent
// ammo pool -- the magazine just bounds one action's burst.
struct WeaponStats {
  int magazineSize;
  float shotIntervalSeconds;  // Time between aimed shots within one action.
};

// Intervals are aimed fire, not cyclic rate, sized against the 5 s round
// (constants::kRoundDuration): Deagle 8 rounds at 0.625 s (8 shots fill the
// window exactly), AR 30 rounds at 0.16 s (~31 fit, so the magazine is the
// binding cap), sniper 5 rounds at 1.0 s of bolt work per shot.
inline const WeaponStats& StatsOf(WeaponType type) {
  static constexpr WeaponStats kDeagle{8, 0.625f};
  static constexpr WeaponStats kAssaultRifle{30, 0.16f};
  static constexpr WeaponStats kSniper{5, 1.0f};
  switch (type) {
    case WeaponType::DesertEagle: return kDeagle;
    case WeaponType::SniperRifle: return kSniper;
    default: return kAssaultRifle;
  }
}

// Shots one action may fire: min(magazine, shots that fit the round window),
// never below 1 (an action always gets its one shot).
inline int MaxShotsPerAction(WeaponType type, float roundDurationSeconds) {
  const WeaponStats& stats = StatsOf(type);
  const int byTime = static_cast<int>(std::floor(roundDurationSeconds / stats.shotIntervalSeconds));
  return std::max(1, std::min(stats.magazineSize, byTime));
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
