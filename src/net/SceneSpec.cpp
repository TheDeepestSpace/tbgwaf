#include "net/SceneSpec.h"

#include <cmath>
#include <string>

#include "game/MapGenerator.h"
#include "game/Weapon.h"
#include "net/Protocol.h"

namespace tactics::net {

namespace {

constexpr float kPi = 3.14159265358979323846f;

bool Fail(std::string* error, const std::string& why) {
  *error = why;
  return false;
}

bool DecodeVec2(const Json& j, glm::vec2* out) {
  if (!j.IsArray() || j.AsArray().size() != 2) return false;
  *out = glm::vec2(j.AsArray()[0].AsNumber(), j.AsArray()[1].AsNumber());
  return true;
}

}  // namespace

bool SceneFromSpec(const Json& spec, tactics::Scene* out, std::string* error) {
  Scene scene;
  bool generated = false;
  const Json& map = spec["map"];
  if (map.Has("generate")) {
    const Json& gen = map["generate"];
    if (!gen["seed"].IsNumber()) return Fail(error, "map.generate requires 'seed'");
    const uint32_t seed = static_cast<uint32_t>(gen["seed"].AsNumber());
    const std::string type = gen["type"].IsString() ? gen["type"].AsString() : "urban";
    if (type == "urban") {
      MapGeneratorConfig config;
      if (gen.Has("arteries")) config.arteryCount = static_cast<int>(gen["arteries"].AsNumber());
      if (gen.Has("artery_width")) config.arteryWidth = static_cast<float>(gen["artery_width"].AsNumber());
      if (gen.Has("local_street_width")) {
        config.localStreetWidth = static_cast<float>(gen["local_street_width"].AsNumber());
      }
      // `elevated` forces the overpass on/off; omitted keeps the generator's
      // seed-driven draw. `elevated_layout` pins one of the deck layouts.
      if (gen.Has("elevated")) {
        config.elevatedHighway = gen["elevated"].AsBool() ? OverpassMode::On : OverpassMode::Off;
      }
      if (gen["elevated_layout"].IsString()) {
        const std::string layout = gen["elevated_layout"].AsString();
        if (layout == "ramp-up-ramp-down") config.overpassLayout = OverpassLayout::RampUpRampDown;
        else if (layout == "through") config.overpassLayout = OverpassLayout::Through;
        else if (layout == "enter-ramp-up") config.overpassLayout = OverpassLayout::EnterRampUp;
        else if (layout == "enter-ramp-down") config.overpassLayout = OverpassLayout::EnterRampDown;
        else {
          return Fail(error, "map.generate.elevated_layout must be ramp-up-ramp-down/through/"
                             "enter-ramp-up/enter-ramp-down, got '" + layout + "'");
        }
      }
      // `elevated_branch` pins the branch's map-edge end (`ramp`/`off-map`);
      // scenarios that force the overpass default to a ramp.
      if (gen.Has("elevated")) config.branchEnd = BranchEnd::Ramp;
      if (gen["elevated_branch"].IsString()) {
        const std::string end = gen["elevated_branch"].AsString();
        if (end == "ramp") config.branchEnd = BranchEnd::Ramp;
        else if (end == "off-map") config.branchEnd = BranchEnd::OffMap;
        else {
          return Fail(error, "map.generate.elevated_branch must be ramp/off-map, got '" + end + "'");
        }
      }
      // `features: {lanes: on, curbs: {mode: auto, chance: 0.3}, ...}`
      // sets the optional urban detail layers (see UrbanFeatures).
      if (gen["features"].IsObject()) {
        for (const auto& [name, value] : gen["features"].AsObject()) {
          FeatureToggle* toggle = UrbanFeatureByName(&config.features, name);
          if (!toggle) return Fail(error, "map.generate.features: unknown feature '" + name + "'");
          const std::string mode = value.IsObject()
                                       ? (value["mode"].IsString() ? value["mode"].AsString() : "auto")
                                       : (value.IsString() ? value.AsString() : "");
          if (!ParseFeatureMode(mode, &toggle->mode)) {
            return Fail(error, "map.generate.features." + name + " must be auto/off/on, got '" +
                                   mode + "'");
          }
          if (value.IsObject() && value["chance"].IsNumber()) {
            toggle->chance = static_cast<float>(value["chance"].AsNumber());
          }
        }
      }
      scene = GenerateUrbanMap(seed, config);
    } else if (type == "hilly") {
      scene = GenerateHillyMap(seed);
    } else {
      return Fail(error, "map.generate.type must be 'urban' or 'hilly', got '" + type + "'");
    }
    generated = true;
  }
  // `half_extent: N` sizes the ground square (default 15); scenarios whose
  // units sit far apart must grow the map so every figure stands on it.
  if (map["half_extent"].IsNumber()) {
    scene.mapHalfExtent = static_cast<float>(map["half_extent"].AsNumber());
  }
  for (const Json& o : map["obstacles"].AsArray()) {
    glm::vec2 center, half;
    if (!DecodeVec2(o["center"], &center)) return Fail(error, "map.obstacles[].center must be a 2-element [x, z] list");
    if (!DecodeVec2(o["half_extent"], &half)) return Fail(error, "map.obstacles[].half_extent must be a 2-element [x, z] list");
    if (!o["height"].IsNumber()) return Fail(error, "map.obstacles[] requires 'height'");
    Obstacle obstacle;
    obstacle.bounds = AABB{glm::vec3(center.x - half.x, 0.0f, center.y - half.y),
                           glm::vec3(center.x + half.x, o["height"].AsNumber(), center.y + half.y)};
    obstacle.climbable = o["climbable"].AsBool();
    scene.obstacles.push_back(obstacle);
  }

  const Json& units = spec["units"];
  if (generated && !units.IsArray()) {
    *out = std::move(scene);
    return true;
  }
  if (generated) scene.units.clear();
  if (!units.IsArray() || units.AsArray().empty()) {
    return Fail(error, "scenario must declare at least one unit under 'units'");
  }
  for (const Json& u : units.AsArray()) {
    if (!u["id"].IsNumber() || !u["team"].IsString() || !u.Has("position")) {
      return Fail(error, "units[] requires 'id', 'team', and 'position'");
    }
    const auto team = ParseTeamName(u["team"].AsString());
    if (!team) return Fail(error, "unknown team '" + u["team"].AsString() + "' in units[].team");
    Unit unit;
    unit.id = static_cast<int>(u["id"].AsNumber());
    unit.team = *team;
    if (!DecodeVec3(u["position"], &unit.position)) {
      return Fail(error, "units[].position must be a 3-element [x, y, z] list");
    }
    unit.facingYaw = static_cast<float>(u["facing_degrees"].AsNumber()) * kPi / 180.0f;
    unit.alive = true;
    if (std::abs(unit.position.x) > scene.mapHalfExtent ||
        std::abs(unit.position.z) > scene.mapHalfExtent) {
      return Fail(error, "units[] id " + std::to_string(unit.id) +
                             " is outside the map; raise map.half_extent (currently " +
                             std::to_string(scene.mapHalfExtent) + ")");
    }
    unit.weapon = DefaultWeaponForUnit(unit.id);
    scene.units.push_back(unit);
  }
  *out = std::move(scene);
  return true;
}

}  // namespace tactics::net
