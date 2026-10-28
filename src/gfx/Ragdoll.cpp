#include "gfx/Ragdoll.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "game/Types.h"
#include "gfx/FigureRig.h"

using tactics::Unit;

namespace gfx {
namespace {

constexpr float kGravity = 14.0f;
constexpr float kAirDamping = 0.992f;
constexpr float kGroundFriction = 0.80f;  // Horizontal velocity kept per step on the ground.
constexpr float kLaunchSpeed = 3.4f;      // Shot impulse on the head/shoulders, units/s.
constexpr float kLaunchLift = 1.1f;
constexpr int kConstraintIterations = 6;
constexpr float kSleepSpeed = 0.05f;      // Units/s; below this a step counts as still.
constexpr int kSleepSteps = 6;
constexpr int kMaxSimSteps = kRagdollSteps;  // Hard cap on simulated steps.
static_assert(kMaxSimSteps * kRagdollStep >= tactics::constants::kKnockdownMaxSeconds - 1e-3f,
              "ragdoll must cover the longest knockdown timer");

// Standing pose in the figure frame (+X forward, +Y up, +Z right), matching
// the proportions in FigureRig.cpp.
const glm::vec3 kStanding[kRagJointCount] = {
    {0.0f, 1.375f, 0.0f},    // Head
    {0.0f, 1.0f, 0.0f},      // Neck
    {0.0f, 0.42f, 0.0f},     // Pelvis
    {0.0f, 0.93f, -0.30f},   // ShoulderL
    {0.0f, 0.93f, 0.30f},    // ShoulderR
    {0.0f, 0.67f, -0.33f},   // ElbowL
    {0.0f, 0.67f, 0.33f},    // ElbowR
    {0.0f, 0.44f, -0.33f},   // HandL
    {0.0f, 0.44f, 0.33f},    // HandR
    {0.0f, 0.42f, -0.16f},   // HipL
    {0.0f, 0.42f, 0.16f},    // HipR
    {0.0f, 0.20f, -0.16f},   // KneeL
    {0.0f, 0.20f, 0.16f},    // KneeR
    {0.0f, 0.0f, -0.16f},    // FootL
    {0.0f, 0.0f, 0.16f},     // FootR
};

// Height of each joint's centre above the ground when resting on it.
const float kGroundOffset[kRagJointCount] = {
    0.36f, 0.14f, 0.20f, 0.16f, 0.16f, 0.10f, 0.10f, 0.08f,
    0.08f, 0.14f, 0.14f, 0.12f, 0.12f, 0.06f, 0.06f,
};

// Launch weight per joint: the upper body takes the hit, the legs lag.
const float kLaunchWeight[kRagJointCount] = {
    1.0f, 1.0f, 0.55f, 1.0f, 1.0f, 0.8f, 0.8f, 0.7f,
    0.7f, 0.35f, 0.35f, 0.25f, 0.25f, 0.15f, 0.15f,
};

constexpr float kGunRestitution = 0.35f;
constexpr float kGunFriction = 0.7f;      // Horizontal velocity kept per contact step.
constexpr float kGunSpinDamping = 0.6f;   // Angular velocity kept per contact step.
constexpr float kGunInertia = 0.03f;      // Unit mass, roughly a thin box.
constexpr float kGunSpin = 9.0f;          // Max tumble rate on release, rad/s.

// Weapon-local extent of the (largest) weapon, used for ground contact. The
// grip top is the origin, +X is the barrel, the magazine/grip hang below.
const glm::vec3 kGunLo(-0.10f, -0.20f, -0.04f);
const glm::vec3 kGunHi(0.55f, 0.09f, 0.04f);

// Whether the unit lets go of its weapon, decided from the id alone.
bool DropsWeapon(int id) {
  unsigned h = static_cast<unsigned>(id) * 2654435761u;
  h ^= h >> 15;
  return (h & 1u) != 0u;
}

// Map obstacles the ragdoll collides with (bounding boxes only), set by the
// renderer. Changing them drops every cached trajectory.
std::vector<tactics::AABB>& Colliders() {
  static std::vector<tactics::AABB> boxes;
  return boxes;
}

// Top of the highest obstacle a point of radius `r` at (x, z) can rest on: one
// whose top it was already at or above (`refY`, the point's previous height),
// so a body beside a wall is never lifted onto it. Falls back to `base`.
float FloorAt(float x, float z, float refY, float r, float base) {
  float floorY = base;
  for (const tactics::AABB& b : Colliders()) {
    if (x < b.min.x - r || x > b.max.x + r || z < b.min.z - r || z > b.max.z + r) continue;
    if (refY >= b.max.y - 0.1f) floorY = std::max(floorY, b.max.y);
  }
  return floorY;
}

// Pushes a point of radius `r` out of any obstacle it is beside (below the
// top, not resting on it) through the nearest side face. Returns the normal
// of the last face used (zero if none).
glm::vec3 PushOutOfWalls(glm::vec3& p, float refY, float r) {
  glm::vec3 normal(0.0f);
  for (const tactics::AABB& b : Colliders()) {
    if (refY >= b.max.y - 0.1f || p.y > b.max.y + r || p.y < b.min.y - r) continue;
    const float dxMin = p.x - (b.min.x - r), dxMax = (b.max.x + r) - p.x;
    const float dzMin = p.z - (b.min.z - r), dzMax = (b.max.z + r) - p.z;
    if (dxMin <= 0.0f || dxMax <= 0.0f || dzMin <= 0.0f || dzMax <= 0.0f) continue;
    const float m = std::min(std::min(dxMin, dxMax), std::min(dzMin, dzMax));
    if (m == dxMin) { p.x -= dxMin; normal = glm::vec3(-1, 0, 0); }
    else if (m == dxMax) { p.x += dxMax; normal = glm::vec3(1, 0, 0); }
    else if (m == dzMin) { p.z -= dzMin; normal = glm::vec3(0, 0, -1); }
    else { p.z += dzMax; normal = glm::vec3(0, 0, 1); }
  }
  return normal;
}

struct Link {
  int a, b;
  float rest;
};

const std::vector<Link>& Links() {
  static const std::vector<Link> links = [] {
    const int pairs[][2] = {
        {kRagHead, kRagNeck},
        {kRagNeck, kRagPelvis},
        {kRagNeck, kRagShoulderL},
        {kRagNeck, kRagShoulderR},
        {kRagShoulderL, kRagShoulderR},
        {kRagShoulderL, kRagPelvis},
        {kRagShoulderR, kRagPelvis},
        {kRagPelvis, kRagHipL},
        {kRagPelvis, kRagHipR},
        {kRagHipL, kRagHipR},
        {kRagShoulderL, kRagHipL},
        {kRagShoulderR, kRagHipR},
        {kRagShoulderL, kRagHipR},
        {kRagShoulderR, kRagHipL},
        {kRagShoulderL, kRagElbowL},
        {kRagElbowL, kRagHandL},
        {kRagShoulderR, kRagElbowR},
        {kRagElbowR, kRagHandR},
        {kRagHipL, kRagKneeL},
        {kRagKneeL, kRagFootL},
        {kRagHipR, kRagKneeR},
        {kRagKneeR, kRagFootR},
        // Soft stops that keep elbows/knees from folding flat through the body.
        {kRagShoulderL, kRagHandL},
        {kRagShoulderR, kRagHandR},
        {kRagHipL, kRagFootL},
        {kRagHipR, kRagFootR},
    };
    std::vector<Link> out;
    for (const auto& p : pairs) {
      out.push_back({p[0], p[1], glm::length(kStanding[p[0]] - kStanding[p[1]])});
    }
    return out;
  }();
  return links;
}

struct Trajectory {
  int id = 0;
  glm::vec3 position{0.0f};
  float yaw = 0.0f;
  glm::vec3 axis{0.0f};
  Unit anim;  // Only the animation fields that set the starting pose are compared.
  std::vector<Unit::KnockdownImpact> impacts;
  unsigned long stamp = 0;
  std::vector<RagdollPose> frames;  // Up to kRagdollSteps + 1 entries.
  std::vector<RagdollGun> guns;     // Same length as frames.
};

bool SameAnimation(const Unit& a, const Unit& b) {
  return a.walkPhase == b.walkPhase && a.walkBlend == b.walkBlend &&
         a.idleElapsed == b.idleElapsed && a.shootElapsed == b.shootElapsed &&
         a.shootAimYaw == b.shootAimYaw && a.grabElapsed == b.grabElapsed &&
         a.rideTravel == b.rideTravel && a.rideLength == b.rideLength &&
         a.rideSlope == b.rideSlope && a.weapon == b.weapon;
}

bool SameImpacts(const std::vector<Unit::KnockdownImpact>& a,
                 const std::vector<Unit::KnockdownImpact>& b) {
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(), [](const auto& x, const auto& y) {
           return x.at == y.at && x.axis == y.axis;
         });
}

bool SameKey(const Trajectory& t, const Unit& u) {
  return t.id == u.id && t.position == u.position && t.yaw == u.facingYaw &&
         t.axis == u.knockdownAxis && SameAnimation(t.anim, u) &&
         SameImpacts(t.impacts, u.knockdownImpacts);
}

Trajectory Simulate(const Unit& unit) {
  Trajectory traj;
  traj.id = unit.id;
  traj.position = unit.position;
  traj.yaw = unit.facingYaw;
  traj.axis = unit.knockdownAxis;
  traj.anim = unit;
  traj.impacts = unit.knockdownImpacts;

  // KnockDown() stores axis = up x dir, so the shot direction is (-axis.z, axis.x).
  const auto shotDir = [&unit](const glm::vec3& axis) {
    const glm::vec3 d(-axis.z, 0.0f, axis.x);
    return glm::length(d) < 1e-4f ? unit.FacingDirection() : glm::normalize(d);
  };
  const glm::vec3 dir = shotDir(unit.knockdownAxis);
  const glm::vec3 side(-dir.z, 0.0f, dir.x);
  // Vary the strength along the hit line so neighbours do not fall
  // identically.  Do not add a sideways component here: there is no lateral
  // force in the replicated hit state, and doing so made a body drift (and
  // appear to snap) across the shot line as the fall played out.
  const float impactScale =
      1.0f + 0.12f * std::sin(static_cast<float>(unit.id) * 12.9898f);

  // Start from the pose the figure was actually in when it was hit (the
  // animation state is frozen from then on), not a neutral stand.
  RagdollPose pos = AnimatedPose(unit), prev;
  for (int i = 0; i < kRagJointCount; ++i) {
    const glm::vec3 v = (dir * (kLaunchSpeed * impactScale) +
                         glm::vec3(0.0f, kLaunchLift, 0.0f)) *
                        kLaunchWeight[i];
    prev[i] = pos[i] - v * kRagdollStep;
  }
  traj.frames.reserve(kRagdollSteps + 1);
  traj.frames.push_back(pos);

  // Weapon: starts in the right hand (barrel along the forearm, as drawn
  // while held) and, if released, flies off with the hand's velocity.
  const bool drops = DropsWeapon(unit.id);
  glm::vec3 gunPos = pos[kRagHandR];
  glm::vec3 gunVel = (pos[kRagHandR] - prev[kRagHandR]) / kRagdollStep;
  glm::vec3 gunSpin(0.0f);
  glm::quat gunRot(1.0f, 0.0f, 0.0f, 0.0f);
  {
    glm::vec3 x = pos[kRagHandR] - pos[kRagElbowR];
    x = glm::length(x) > 1e-5f ? glm::normalize(x) : dir;
    glm::vec3 z = glm::cross(x, glm::vec3(0.0f, 1.0f, 0.0f));
    z = glm::length(z) < 1e-4f ? glm::vec3(0.0f, 0.0f, 1.0f) : glm::normalize(z);
    gunRot = glm::quat_cast(glm::mat3(x, glm::cross(z, x), z));
    // Tumble about the sideways axis (barrel flips over), plus a little yaw.
    const float s = std::sin(static_cast<float>(unit.id) * 7.531f);
    gunSpin = side * (kGunSpin * (0.5f + 0.5f * std::fabs(s)) * (s < 0 ? -1.0f : 1.0f)) +
              glm::vec3(0.0f, 0.7f * std::sin(static_cast<float>(unit.id) * 12.9898f), 0.0f);
    gunVel += dir * 0.8f + glm::vec3(0.0f, 0.6f, 0.0f);
  }
  bool gunAsleep = false;
  auto gunFrame = [&]() { return RagdollGun{drops, gunPos, gunRot}; };
  traj.guns.reserve(kRagdollSteps + 1);
  traj.guns.push_back(gunFrame());

  // A figure shot while standing on top of an obstacle falls to the real
  // ground (y = 0); otherwise the ground is wherever it was standing.
  float groundY = unit.position.y;
  for (const tactics::AABB& b : Colliders()) {
    if (unit.position.x >= b.min.x && unit.position.x <= b.max.x && unit.position.z >= b.min.z &&
        unit.position.z <= b.max.z && std::fabs(unit.position.y - b.max.y) < 0.15f) {
      groundY = 0.0f;
    }
  }
  int still = 0;
  std::array<bool, kRagJointCount> onFloor{};
  // Later hits on the falling body, in time order, applied as extra shoves.
  std::vector<Unit::KnockdownImpact> impacts = unit.knockdownImpacts;
  std::stable_sort(impacts.begin(), impacts.end(),
                   [](const auto& a, const auto& b) { return a.at < b.at; });
  size_t nextImpact = 0;
  // Run until body and (dropped) gun are at rest, up to the cap: a fall from
  // an obstacle top takes a while.
  for (int step = 0; step < kMaxSimSteps; ++step) {
    while (nextImpact < impacts.size() &&
           std::lround(impacts[nextImpact].at / kRagdollStep) <= step) {
      const glm::vec3 hitDir = shotDir(impacts[nextImpact].axis);
      const glm::vec3 hitV =
          hitDir * (kLaunchSpeed * impactScale) + glm::vec3(0.0f, kLaunchLift, 0.0f);
      for (int i = 0; i < kRagJointCount; ++i) {
        prev[i] -= hitV * kLaunchWeight[i] * kRagdollStep;
      }
      if (drops) {
        gunVel += hitDir * 0.8f + glm::vec3(0.0f, 0.6f, 0.0f);
        gunAsleep = false;
      }
      still = 0;
      ++nextImpact;
    }
    if (still >= kSleepSteps && (!drops || gunAsleep) && nextImpact >= impacts.size()) break;
    if (drops && !gunAsleep) {
      gunVel.y -= kGravity * kRagdollStep;
      const float gunRefY = gunPos.y;
      gunPos += gunVel * kRagdollStep;
      {
        const glm::vec3 n = PushOutOfWalls(gunPos, gunRefY, 0.15f);
        const float into = glm::dot(gunVel, n);
        if (into < 0.0f) gunVel -= (1.0f + kGunRestitution) * into * n;
      }
      const glm::quat dq = glm::quat(0.0f, gunSpin.x * kRagdollStep, gunSpin.y * kRagdollStep,
                                   gunSpin.z * kRagdollStep) * gunRot;
      gunRot = glm::normalize(glm::quat(gunRot.w + 0.5f * dq.w, gunRot.x + 0.5f * dq.x,
                                        gunRot.y + 0.5f * dq.y, gunRot.z + 0.5f * dq.z));
      const glm::mat3 R = glm::mat3_cast(gunRot);
      const glm::vec3 centre = gunPos + R * ((kGunLo + kGunHi) * 0.5f);
      bool touched = false;
      for (int c = 0; c < 8; ++c) {
        const glm::vec3 local((c & 1) ? kGunHi.x : kGunLo.x, (c & 2) ? kGunHi.y : kGunLo.y,
                              (c & 4) ? kGunHi.z : kGunLo.z);
        const glm::vec3 corner = gunPos + R * local;
        const float pen = FloorAt(corner.x, corner.z, gunRefY, 0.0f, groundY) - corner.y;
        if (pen <= 0.0f) continue;
        gunPos.y += pen;
        const glm::vec3 r = corner - centre;
        const glm::vec3 cv = gunVel + glm::cross(gunSpin, r);
        if (cv.y < 0.0f) {
          const glm::vec3 rn = glm::cross(r, glm::vec3(0.0f, 1.0f, 0.0f));
          const float j =
              -(1.0f + kGunRestitution) * cv.y / (1.0f + glm::dot(rn, rn) / kGunInertia);
          gunVel.y += j;
          gunSpin += glm::cross(r, glm::vec3(0.0f, j, 0.0f)) / kGunInertia;
        }
        touched = true;
      }
      if (touched) {
        gunVel.x *= kGunFriction;
        gunVel.z *= kGunFriction;
        gunSpin *= kGunSpinDamping;
        if (glm::length(gunVel) < 0.15f && glm::length(gunSpin) < 0.4f) {
          gunAsleep = true;
          gunVel = glm::vec3(0.0f);
          gunSpin = glm::vec3(0.0f);
        }
      }
    }
    if (still >= kSleepSteps) {  // Asleep: hold the pose while the gun settles.
      traj.frames.push_back(pos);
      traj.guns.push_back(gunFrame());
      continue;
    }
    for (int i = 0; i < kRagJointCount; ++i) {
      const glm::vec3 vel = (pos[i] - prev[i]) * kAirDamping;
      prev[i] = pos[i];
      pos[i] += vel + glm::vec3(0.0f, -kGravity, 0.0f) * (kRagdollStep * kRagdollStep);
    }
    for (int it = 0; it < kConstraintIterations; ++it) {
      for (const Link& l : Links()) {
        glm::vec3 d = pos[l.b] - pos[l.a];
        const float len = glm::length(d);
        if (len < 1e-6f) continue;
        const float diff = (len - l.rest) / len * 0.5f;
        pos[l.a] += d * diff;
        pos[l.b] -= d * diff;
      }
      for (int i = 0; i < kRagJointCount; ++i) {
        const float r = kGroundOffset[i];
        const float refY = prev[i].y;
        const glm::vec3 n = PushOutOfWalls(pos[i], refY, r);
        if (n != glm::vec3(0.0f)) {
          // Kill the velocity into the wall (Verlet: move prev onto pos along n).
          const float into = glm::dot(pos[i] - prev[i], n);
          if (into < 0.0f) prev[i] += into * n;
        }
        const float floorY = FloorAt(pos[i].x, pos[i].z, refY, r, groundY) + r;
        if (pos[i].y < floorY) pos[i].y = floorY;
        onFloor[i] = pos[i].y <= floorY + 1e-3f;
      }
    }
    float maxSpeed = 0.0f;
    for (int i = 0; i < kRagJointCount; ++i) {
      if (onFloor[i]) {
        // Resting on the ground: bleed off horizontal motion (friction).
        prev[i].x = pos[i].x - (pos[i].x - prev[i].x) * kGroundFriction;
        prev[i].z = pos[i].z - (pos[i].z - prev[i].z) * kGroundFriction;
      }
      maxSpeed = std::max(maxSpeed, glm::length(pos[i] - prev[i]) / kRagdollStep);
    }
    still = maxSpeed < kSleepSpeed ? still + 1 : 0;
    traj.frames.push_back(pos);
    traj.guns.push_back(gunFrame());
  }
  return traj;
}

}  // namespace

namespace {
std::vector<Trajectory>& Cache() {
  static std::vector<Trajectory> cache;
  return cache;
}
}  // namespace

void SetRagdollObstacles(const std::vector<tactics::Obstacle>& obstacles) {
  std::vector<tactics::AABB> boxes;
  boxes.reserve(obstacles.size());
  for (const tactics::Obstacle& o : obstacles) boxes.push_back(o.bounds);
  std::vector<tactics::AABB>& cur = Colliders();
  const bool same = boxes.size() == cur.size() &&
                    std::equal(boxes.begin(), boxes.end(), cur.begin(),
                               [](const tactics::AABB& a, const tactics::AABB& b) {
                                 return a.min == b.min && a.max == b.max;
                               });
  if (same) return;
  cur = std::move(boxes);
  Cache().clear();
}

bool RagdollActive(const Unit& unit) { return !unit.alive && unit.knockdownElapsed >= 0.0f; }

namespace {

const Trajectory& CachedTrajectory(const Unit& unit) {
  // Small LRU cache of full trajectories, so the several draw passes per
  // frame share one simulation and the number of live ragdolls stays bounded
  // (an evicted one is simply re-simulated if it's drawn again).
  std::vector<Trajectory>& cache = Cache();
  static unsigned long clock = 0;
  Trajectory* hit = nullptr;
  for (Trajectory& t : cache) {
    if (SameKey(t, unit)) {
      hit = &t;
      break;
    }
  }
  if (!hit) {
    if (static_cast<int>(cache.size()) < kRagdollMaxCached) {
      cache.push_back(Simulate(unit));
      hit = &cache.back();
    } else {
      hit = &*std::min_element(cache.begin(), cache.end(),
                               [](const Trajectory& a, const Trajectory& b) {
                                 return a.stamp < b.stamp;
                               });
      *hit = Simulate(unit);
    }
  }
  hit->stamp = ++clock;
  return *hit;
}

}  // namespace

RagdollPose SampleRagdollStep(const Unit& unit, int step) {
  const std::vector<RagdollPose>& frames = CachedTrajectory(unit).frames;
  return frames[static_cast<size_t>(glm::clamp(step, 0, static_cast<int>(frames.size()) - 1))];
}

RagdollGun SampleRagdollGunStep(const Unit& unit, int step) {
  const std::vector<RagdollGun>& guns = CachedTrajectory(unit).guns;
  return guns[static_cast<size_t>(glm::clamp(step, 0, static_cast<int>(guns.size()) - 1))];
}

namespace {
int CurrentStep(const Unit& unit) {
  return static_cast<int>(std::lround(std::max(0.0f, unit.knockdownElapsed) / kRagdollStep));
}
}  // namespace

RagdollPose SampleRagdoll(const Unit& unit) { return SampleRagdollStep(unit, CurrentStep(unit)); }

RagdollGun SampleRagdollGun(const Unit& unit) {
  return SampleRagdollGunStep(unit, CurrentStep(unit));
}

}  // namespace gfx
