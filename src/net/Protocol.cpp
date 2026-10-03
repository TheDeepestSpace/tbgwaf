#include "net/Protocol.h"

#include <cmath>

namespace tactics::net {

namespace {

bool ReadId(const Json& j, int* out) {
  if (!j.IsNumber()) return false;
  const double v = j.AsNumber();
  if (v < 0 || v > 1023 || v != std::floor(v)) return false;
  *out = static_cast<int>(v);
  return true;
}

struct KindName {
  const char* name;
  ActionKind kind;
};
constexpr KindName kKinds[] = {
    {"move", ActionKind::Move},         {"shoot", ActionKind::Shoot},
    {"pass", ActionKind::Pass},         {"cancel", ActionKind::Cancel},
    {"focus", ActionKind::Focus},       {"reaction", ActionKind::Reaction},
    {"commit", ActionKind::Commit},     {"new_match", ActionKind::NewMatch},
};

struct ReactionName {
  const char* name;
  ReactionAction action;
};
constexpr ReactionName kReactions[] = {
    {"none", ReactionAction::DoNothing},   {"shoot", ReactionAction::Shoot},
    {"stop", ReactionAction::Stop},        {"continue", ReactionAction::Continue},
    {"shoot_stop", ReactionAction::ShootStop}, {"shoot_continue", ReactionAction::ShootContinue},
};

}  // namespace

bool DecodeVec3(const Json& j, glm::vec3* out) {
  if (!j.IsArray() || j.AsArray().size() != 3) return false;
  float v[3];
  for (int i = 0; i < 3; ++i) {
    const Json& c = j.AsArray()[i];
    if (!c.IsNumber() || std::fabs(c.AsNumber()) > 1e5) return false;
    v[i] = static_cast<float>(c.AsNumber());
  }
  *out = glm::vec3(v[0], v[1], v[2]);
  return true;
}

Json EncodeVec3(const glm::vec3& v) {
  return Json(Json::Array{Json(static_cast<double>(v.x)), Json(static_cast<double>(v.y)),
                          Json(static_cast<double>(v.z))});
}

Json EncodePlaybook(const SquadPlaybook& playbook) {
  Json list{Json::Array{}};
  for (int moving = 0; moving < 2; ++moving) {
    for (int seen = 0; seen < 2; ++seen) {
      for (const auto& r : kReactions) {
        if (r.action == playbook.table[moving][seen]) list.Push(Json(std::string(r.name)));
      }
    }
  }
  return list;
}

bool DecodePlaybook(const Json& j, SquadPlaybook* out) {
  if (!j.IsArray() || j.AsArray().size() != 4) return false;
  SquadPlaybook pb;
  for (int i = 0; i < 4; ++i) {
    const Json& e = j.AsArray()[i];
    if (!e.IsString()) return false;
    bool found = false;
    for (const auto& r : kReactions) {
      if (e.AsString() == r.name) {
        pb.table[i / 2][i % 2] = r.action;
        found = true;
      }
    }
    if (!found) return false;
  }
  *out = pb;
  return true;
}

bool ParseAction(const Json& m, Action* out, std::string* error) {
  auto Fail = [&](const char* why) {
    *error = why;
    return false;
  };
  if (!m.IsObject() || !m["t"].IsString()) return Fail("missing message type");
  Action a;
  bool known = false;
  for (const auto& k : kKinds) {
    if (m["t"].AsString() == k.name) {
      a.kind = k.kind;
      known = true;
    }
  }
  if (!known) return Fail("unknown action");
  if (m.Has("seq")) {
    if (!m["seq"].IsNumber()) return Fail("bad seq");
    a.seq = static_cast<int>(m["seq"].AsNumber());
  }
  const bool needsUnit = a.kind != ActionKind::Commit && a.kind != ActionKind::NewMatch &&
                         a.kind != ActionKind::Reaction;
  if (needsUnit && !ReadId(m["unit"], &a.unit)) return Fail("bad unit id");

  switch (a.kind) {
    case ActionKind::Shoot:
      if (!ReadId(m["target"], &a.target)) return Fail("bad target id");
      break;
    case ActionKind::Move: {
      if (!m["waypoints"].IsArray()) return Fail("missing waypoints");
      const auto& pts = m["waypoints"].AsArray();
      if (pts.empty() || pts.size() > kMaxWaypoints) return Fail("bad waypoint count");
      for (const Json& p : pts) {
        glm::vec3 v;
        if (!DecodeVec3(p, &v)) return Fail("bad waypoint");
        a.waypoints.push_back(v);
      }
      if (m.Has("facing")) {
        if (!m["facing"].IsNumber() || !std::isfinite(m["facing"].AsNumber())) return Fail("bad facing");
        a.facing = static_cast<float>(m["facing"].AsNumber());
      }
      break;
    }
    case ActionKind::Reaction: {
      if (!DecodePlaybook(m["table"], &a.playbook)) return Fail("bad reaction table");
      break;
    }
    default:
      break;
  }
  *out = std::move(a);
  return true;
}

Json EncodeAction(const Action& a) {
  Json j;
  for (const auto& k : kKinds) {
    if (k.kind == a.kind) j.Set("t", k.name);
  }
  if (a.seq) j.Set("seq", a.seq);
  if (a.unit >= 0) j.Set("unit", a.unit);
  if (a.kind == ActionKind::Shoot) j.Set("target", a.target);
  if (a.kind == ActionKind::Move) {
    Json pts{Json::Array{}};
    for (const auto& w : a.waypoints) pts.Push(EncodeVec3(w));
    j.Set("waypoints", std::move(pts));
    if (a.facing) j.Set("facing", static_cast<double>(*a.facing));
  }
  if (a.kind == ActionKind::Reaction) {
    j.Set("table", EncodePlaybook(a.playbook));
  }
  return j;
}

const char* TeamName(Team team) { return team == Team::Blue ? "blue" : "red"; }

std::optional<Team> ParseTeamName(const std::string& name) {
  if (name == "blue") return Team::Blue;
  if (name == "red") return Team::Red;
  return std::nullopt;
}

const char* PlanName(PlannedActionType type) {
  switch (type) {
    case PlannedActionType::Move: return "move";
    case PlannedActionType::Shoot: return "shoot";
    case PlannedActionType::Pass: return "pass";
    case PlannedActionType::None: break;
  }
  return "none";
}

std::optional<PlannedActionType> ParsePlanName(const std::string& name) {
  if (name == "move") return PlannedActionType::Move;
  if (name == "shoot") return PlannedActionType::Shoot;
  if (name == "pass") return PlannedActionType::Pass;
  if (name == "none") return PlannedActionType::None;
  return std::nullopt;
}

}  // namespace tactics::net
