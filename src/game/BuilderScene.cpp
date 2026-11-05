#include "game/BuilderScene.h"

#include <cmath>

#include "game/DemoScenes.h"
#include "game/Json.h"

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
  for (const BuilderBox& box : bs.boxes) {
    Obstacle o;
    o.bounds = AABB{glm::vec3(box.cx - box.hx, box.y, box.cz - box.hz),
                    glm::vec3(box.cx + box.hx, BoxTop(box), box.cz + box.hz)};
    o.climbable = true;
    scene.obstacles.push_back(o);
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
  out += "],\"figures\":[";
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
