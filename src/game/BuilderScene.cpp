#include "game/BuilderScene.h"

#include <algorithm>
#include <cmath>

#include "game/DemoScenes.h"
#include "game/Geometry.h"
#include "game/Json.h"
#include "game/MapGenerator.h"

namespace tactics {
namespace {

using namespace json;

constexpr float kPi = 3.14159265358979323846f;

struct WeaponName {
  WeaponType type;
  const char* name;
};
constexpr WeaponName kWeaponNames[] = {
    {WeaponType::DesertEagle, "desert_eagle"},
    {WeaponType::AssaultRifle, "assault_rifle"},
    {WeaponType::SniperRifle, "sniper_rifle"},
};

const char* WeaponWireName(WeaponType type) {
  for (const WeaponName& entry : kWeaponNames) {
    if (entry.type == type) return entry.name;
  }
  return "assault_rifle";
}

bool ParseWeaponWireName(const std::string& name, WeaponType* out) {
  for (const WeaponName& entry : kWeaponNames) {
    if (name == entry.name) {
      *out = entry.type;
      return true;
    }
  }
  return false;
}

bool BoxContains(const BuilderBox& box, float x, float z) {
  return std::fabs(x - box.cx) <= box.hx && std::fabs(z - box.cz) <= box.hz;
}

bool FailWith(std::string* error, const std::string& what) {
  if (error) *error = what;
  return false;
}

bool ReadNumberAt(const JsonValue& array, size_t index, float* out) {
  return index < array.array.size() && ReadFloat(&array.array[index], out) && std::isfinite(*out);
}

}  // namespace

float SurfaceHeightAt(const BuilderScene& bs, float x, float z) {
  float top = 0.0f;
  for (const BuilderBox& box : bs.boxes) {
    if (BoxContains(box, x, z) && BoxTop(box) > top) top = BoxTop(box);
  }
  return top;
}

Scene BuildScene(const BuilderScene& bs) {
  Scene scene;
  scene.mapHalfExtent = bs.mapHalfExtent;
  scene.roundLimit = bs.roundLimit;
  // Boxes first (Obstacle index == box index), then their tops and ramps.
  for (const BuilderBox& box : bs.boxes) {
    Obstacle o;
    o.bounds = AABB{glm::vec3(box.cx - box.hx, box.y, box.cz - box.hz),
                    glm::vec3(box.cx + box.hx, BoxTop(box), box.cz + box.hz)};
    scene.obstacles.push_back(o);
  }
  // The ramp leans on the first side (toward -X, +X, -Z, +Z) whose footprint
  // stays inside the map and clear of every other box reaching above the
  // ramp's own base; failing that, the -X side. A stacked box's ramp must
  // land on the box it stands on, so its run is capped at the distance from
  // the face to that support's edge (steeper, but connected).
  const glm::vec2 sides[] = {{-1.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, -1.0f}, {0.0f, 1.0f}};
  for (size_t i = 0; i < bs.boxes.size(); ++i) {
    const BuilderBox& box = bs.boxes[i];
    const glm::vec2 c(box.cx, box.cz);
    const glm::vec2 half(box.hx, box.hz);
    const glm::vec2 faceLo = c - half, faceHi = c + half;
    glm::vec2 chosen = sides[0];
    float chosenRun = box.height / kClimbRampSlope;
    bool found = false;
    for (const glm::vec2& side : sides) {
      float run = box.height / kClimbRampSlope;
      if (box.y > 1e-4f) {
        // Support: the box whose top is this one's base and which holds the
        // point just outside the face's middle.
        const glm::vec2 probe = c + side * (glm::dot(glm::abs(side), half) + 0.05f);
        float room = -1.0f;
        for (size_t j = 0; j < bs.boxes.size(); ++j) {
          const BuilderBox& other = bs.boxes[j];
          if (j == i || std::fabs(BoxTop(other) - box.y) > 1e-4f || !BoxContains(other, probe.x, probe.y)) continue;
          const float edge = side.x != 0.0f ? (side.x > 0.0f ? other.cx + other.hx : other.cx - other.hx)
                                            : (side.y > 0.0f ? other.cz + other.hz : other.cz - other.hz);
          const float along = side.x != 0.0f ? probe.x : probe.y;  // Probe, on the side's axis.
          room = std::max(room, std::fabs(edge - along) + 0.05f);  // Face to the support's edge.
        }
        if (room < 0.2f) continue;  // Nothing to stand on beyond this face.
        run = std::min(run, room - 0.05f);
      }
      // Ramp footprint: the box face pushed out by `run`.
      glm::vec2 lo = faceLo, hi = faceHi;
      if (side.x < 0.0f) { hi.x = faceLo.x; lo.x = faceLo.x - run; }
      if (side.x > 0.0f) { lo.x = faceHi.x; hi.x = faceHi.x + run; }
      if (side.y < 0.0f) { hi.y = faceLo.y; lo.y = faceLo.y - run; }
      if (side.y > 0.0f) { lo.y = faceHi.y; hi.y = faceHi.y + run; }
      bool clear = std::max(std::fabs(lo.x), std::fabs(hi.x)) <= bs.mapHalfExtent &&
                   std::max(std::fabs(lo.y), std::fabs(hi.y)) <= bs.mapHalfExtent;
      for (size_t j = 0; j < bs.boxes.size() && clear; ++j) {
        if (j == i) continue;
        const BuilderBox& other = bs.boxes[j];
        if (BoxTop(other) <= box.y + 1e-4f) continue;  // Under the ramp's base: it is the floor.
        clear &= other.cx - other.hx >= hi.x - 1e-4f || other.cx + other.hx <= lo.x + 1e-4f ||
                 other.cz - other.hz >= hi.y - 1e-4f || other.cz + other.hz <= lo.y + 1e-4f;
      }
      if (clear) {
        chosen = side;
        chosenRun = run;
        found = true;
        break;
      }
    }
    if (!found && box.y > 1e-4f) chosenRun = 0.5f;  // Floating box: a stub ramp, as a hint.
    const int top = AddClimbRamp(&scene, static_cast<int>(i), chosen, chosenRun);
    if (top < 0 || box.y <= 1e-4f) continue;
    // Stacked: the ramp's foot stands on the top of the box beneath. Link it
    // to that top (the first earlier top surface containing the foot's
    // midpoint at the same height).
    const int ramp = top + 1;
    const WalkSurface& flight = scene.walkSurfaces[ramp];
    glm::vec3 foot(0.0f);
    int count = 0;
    for (const glm::vec3& v : flight.vertices) {
      if (std::fabs(v.y - box.y) < 1e-3f) { foot += v; ++count; }
    }
    if (count == 0) continue;
    foot /= static_cast<float>(count);
    for (int s = 0; s < ramp; ++s) {
      const WalkSurface& other = scene.walkSurfaces[s];
      if (other.vertices.size() != 4 || other.connectsToGround) continue;
      if (std::fabs(other.vertices[0].y - box.y) > 1e-3f) continue;
      if (!SurfaceContainsXZ(other, foot.x, foot.z)) continue;
      scene.walkSurfaces[s].neighbors.push_back(ramp);
      scene.walkSurfaces[ramp].neighbors.push_back(s);
      break;
    }
  }
  int id = 0;
  for (const BuilderFigure& figure : bs.figures) {
    Unit u;
    u.id = id++;
    u.team = figure.team;
    u.position = glm::vec3(figure.x, SurfaceHeightAt(bs, figure.x, figure.z), figure.z);
    // Blue faces +X, Red faces -X: toward the enemy side, as in the default scene.
    u.facingYaw = figure.hasYaw ? figure.yaw : (figure.team == Team::Blue ? 0.0f : kPi);
    u.alive = true;
    u.weapon = figure.weapon;
    scene.units.push_back(u);
  }
  for (const BuilderBuilding& building : bs.buildings) {
    AppendGeneratedBuilding(&scene, GeneratedBuildingSpec{static_cast<uint32_t>(building.seed),
                                                          building.cx, building.cz, building.hx,
                                                          building.hz});
  }
  return scene;
}

std::string SerializeBuilderScene(const BuilderScene& bs) {
  std::string out = "{\"name\":";
  AppendJsonString(&out, bs.name);
  out += ",\"half_extent\":" + FloatText(bs.mapHalfExtent);
  out += ",\"round_limit\":" + std::to_string(bs.roundLimit);
  out += ",\"boxes\":[";
  for (size_t i = 0; i < bs.boxes.size(); ++i) {
    const BuilderBox& b = bs.boxes[i];
    if (i) out += ',';
    out += '[' + FloatText(b.cx) + ',' + FloatText(b.cz) + ',' + FloatText(b.hx) + ',' +
           FloatText(b.hz) + ',' + FloatText(b.height);
    if (b.y != 0.0f) out += ',' + FloatText(b.y);  // Omitted for ground cubes.
    out += ']';
  }
  out += ']';
  if (!bs.buildings.empty()) {
    out += ",\"buildings\":[";
    for (size_t i = 0; i < bs.buildings.size(); ++i) {
      const BuilderBuilding& b = bs.buildings[i];
      if (i) out += ',';
      out += '[' + std::to_string(b.seed) + ',' + FloatText(b.cx) + ',' + FloatText(b.cz) + ',' +
             FloatText(b.hx) + ',' + FloatText(b.hz) + ']';
    }
    out += ']';
  }
  out += ",\"figures\":[";
  for (size_t i = 0; i < bs.figures.size(); ++i) {
    const BuilderFigure& f = bs.figures[i];
    if (i) out += ',';
    out += "{\"team\":\"";
    out += f.team == Team::Blue ? "blue" : "red";
    out += "\",\"weapon\":\"";
    out += WeaponWireName(f.weapon);
    out += "\",\"x\":" + FloatText(f.x) + ",\"z\":" + FloatText(f.z);
    if (f.hasYaw) out += ",\"yaw\":" + FloatText(f.yaw);
    out += '}';
  }
  out += "]}";
  return out;
}

bool ParseBuilderScene(const std::string& text, BuilderScene* out, std::string* error) {
  JsonValue root;
  std::string parseError;
  if (!ParseJson(text, &root, &parseError)) return FailWith(error, parseError);
  if (root.type != JsonValue::Type::Object) return FailWith(error, "builder scene is not an object");

  BuilderScene bs;
  if (!ReadString(root.Find("name"), &bs.name)) return FailWith(error, "builder scene needs 'name'");
  if (!ReadFloat(root.Find("half_extent"), &bs.mapHalfExtent) || !std::isfinite(bs.mapHalfExtent) ||
      bs.mapHalfExtent <= 0.0f) {
    return FailWith(error, "builder scene needs a positive 'half_extent'");
  }
  if (!ReadInt(root.Find("round_limit"), &bs.roundLimit) || bs.roundLimit < 0) {
    return FailWith(error, "builder scene needs a non-negative 'round_limit'");
  }

  const JsonValue* boxes = root.Find("boxes");
  if (!boxes || boxes->type != JsonValue::Type::Array) {
    return FailWith(error, "builder scene needs 'boxes'");
  }
  for (const JsonValue& v : boxes->array) {
    BuilderBox b;
    if (v.type != JsonValue::Type::Array || (v.array.size() != 5 && v.array.size() != 6) || !ReadNumberAt(v, 0, &b.cx) ||
        !ReadNumberAt(v, 1, &b.cz) || !ReadNumberAt(v, 2, &b.hx) || !ReadNumberAt(v, 3, &b.hz) ||
        !ReadNumberAt(v, 4, &b.height) || (v.array.size() == 6 && !ReadNumberAt(v, 5, &b.y))) {
      return FailWith(error, "box must be [cx, cz, hx, hz, height] or [..., y]");
    }
    if (b.hx <= 0.0f || b.hz <= 0.0f || b.height <= 0.0f || b.y < 0.0f) {
      return FailWith(error, "box extents and height must be positive and y non-negative");
    }
    bs.boxes.push_back(b);
  }

  if (const JsonValue* buildings = root.Find("buildings")) {
    if (buildings->type != JsonValue::Type::Array) {
      return FailWith(error, "builder scene 'buildings' must be an array");
    }
    for (const JsonValue& v : buildings->array) {
      BuilderBuilding b;
      float seed = 0.0f;
      if (v.type != JsonValue::Type::Array || v.array.size() != 5 || !ReadNumberAt(v, 0, &seed) ||
          !ReadNumberAt(v, 1, &b.cx) || !ReadNumberAt(v, 2, &b.cz) || !ReadNumberAt(v, 3, &b.hx) ||
          !ReadNumberAt(v, 4, &b.hz)) {
        return FailWith(error, "building must be [seed, cx, cz, hx, hz]");
      }
      if (!ReadInt(&v.array[0], &b.seed) || b.seed < 0 || b.hx <= 0.0f || b.hz <= 0.0f) {
        return FailWith(error, "building seed must be a non-negative integer and extents positive");
      }
      bs.buildings.push_back(b);
    }
  }

  const JsonValue* figures = root.Find("figures");
  if (!figures || figures->type != JsonValue::Type::Array) {
    return FailWith(error, "builder scene needs 'figures'");
  }
  for (const JsonValue& v : figures->array) {
    BuilderFigure f;
    std::string team, weapon;
    if (v.type != JsonValue::Type::Object || !ReadString(v.Find("team"), &team) ||
        !ReadString(v.Find("weapon"), &weapon) || !ReadFloat(v.Find("x"), &f.x) ||
        !ReadFloat(v.Find("z"), &f.z) || !std::isfinite(f.x) || !std::isfinite(f.z)) {
      return FailWith(error, "figure needs team, weapon, x, z");
    }
    if (const JsonValue* yaw = v.Find("yaw")) {
      if (!ReadFloat(yaw, &f.yaw) || !std::isfinite(f.yaw)) {
        return FailWith(error, "figure 'yaw' must be a number");
      }
      f.hasYaw = true;
    }
    if (team == "blue") f.team = Team::Blue;
    else if (team == "red") f.team = Team::Red;
    else return FailWith(error, "unknown figure team '" + team + "'");
    if (!ParseWeaponWireName(weapon, &f.weapon)) {
      return FailWith(error, "unknown figure weapon '" + weapon + "'");
    }
    bs.figures.push_back(f);
  }

  *out = std::move(bs);
  return true;
}

void InitBuilderGame(GameLogic* game, const BuilderScene& bs, uint32_t seed) {
  // Same session config as InitDemoGame: back to the defaults the log header
  // implies.
  game->SetInputRecorder(nullptr);
  game->SetShotRollSource({});
  game->SetPlaybook(Team::Blue, SquadPlaybook{});
  game->SetPlaybook(Team::Red, SquadPlaybook{});
  game->SetFriendlyFireEnabled(constants::kFriendlyFireDefault);
  game->SeedShotRng(seed);
  game->Reset(BuildScene(bs));
  game->UpdateSightingMemory(0.0f);
}

std::string BuilderSceneLogId(const BuilderScene& bs) { return "builder:" + bs.name; }

ReplayReport ReplayBuilderLog(const std::string& text, GameLogic* game, ReproLog* parsed,
                              const std::function<void(const ReproEntry&, float dt)>& afterEntry) {
  ReplayReport report;
  ReproLog local;
  ReproLog& log = parsed ? *parsed : local;
  if (!ParseReproLog(text, &log, &report.error)) return report;
  if (log.header.builderScene.empty()) {
    report.error = "log has no builder_scene header";
    return report;
  }
  BuilderScene bs;
  std::string sceneError;
  if (!ParseBuilderScene(log.header.builderScene, &bs, &sceneError)) {
    report.error = "bad builder_scene: " + sceneError;
    return report;
  }
  InitBuilderGame(game, bs, log.header.seed);
  return ReplayReproLog(log, BuildScene(bs), game, afterEntry);
}

ReplayReport ReplayAnyLog(const std::string& text, GameLogic* game, ReproLog* parsed,
                          const std::function<void(const ReproEntry&, float dt)>& afterEntry) {
  ReproLog local;
  ReproLog& log = parsed ? *parsed : local;
  ReplayReport report;
  if (!ParseReproLog(text, &log, &report.error)) return report;
  if (!log.header.builderScene.empty()) return ReplayBuilderLog(text, game, &log, afterEntry);
  return ReplayDemoLog(text, game, &log, afterEntry);
}

}  // namespace tactics
