#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

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
  // Half-angle of the cone each bullet's trajectory is scattered inside
  // (issue #140): every bullet of a burst flies its own line.
  float scatterHalfAngleDegrees;
};

// Intervals are aimed fire, not cyclic rate, and pace the burst in real
// time during the round's execution (issue #140): Deagle 8 rounds at 0.5 s
// (10 would fit the 5 s window, so the 8-round magazine binds), AR 30
// rounds at 0.16 s (~31 fit, the magazine binds again), sniper 5 rounds at
// 1.0 s of bolt work per shot.
inline const WeaponStats& StatsOf(WeaponType type) {
  static constexpr WeaponStats kDeagle{8, 0.5f, 4.0f};
  static constexpr WeaponStats kAssaultRifle{30, 0.16f, 3.0f};
  static constexpr WeaponStats kSniper{5, 1.0f, 0.8f};
  switch (type) {
    case WeaponType::DesertEagle: return kDeagle;
    case WeaponType::SniperRifle: return kSniper;
    default: return kAssaultRifle;
  }
}

// Deterministic per-bullet randomness in [0,1) for trajectory scatter
// (issue #140): a pure hash of (shooter, round, bullet index, salt), so it is
// tick-size independent and identical in every client/test/gallery without
// touching the hit-roll stream.
inline float ScatterUnit(int shooterId, int round, int index, int salt) {
  uint64_t x = (static_cast<uint64_t>(shooterId + 1) * 0x9E3779B97F4A7C15ULL) ^
               (static_cast<uint64_t>(round) << 32) ^
               (static_cast<uint64_t>(index + 1) * 0xBF58476D1CE4E5B9ULL) ^
               (static_cast<uint64_t>(salt + 1) * 0x94D049BB133111EBULL);
  x ^= x >> 30;
  x *= 0xBF58476D1CE4E5B9ULL;
  x ^= x >> 27;
  x *= 0x94D049BB133111EBULL;
  x ^= x >> 31;
  return static_cast<float>(x >> 40) / 16777216.0f;
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
