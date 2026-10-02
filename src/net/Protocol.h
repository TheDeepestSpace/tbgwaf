#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"
#include "game/Unit.h"
#include "net/Json.h"

// Wire protocol between the authoritative server and its clients (JSON text
// frames over WebSocket). Clients send *discrete, validated actions*; the
// server answers with rule state only -- never animation-interpolation
// fields -- and, after a round, a fog-filtered timeline each client plays
// back locally.
//
// Client -> server (all may carry "seq", echoed in the "ack"):
//   {"t":"join","room":"<name>"}                      first message
//   {"t":"move","unit":3,"waypoints":[[x,y,z],...],"facing":yaw}
//   {"t":"shoot","unit":3,"target":4}
//   {"t":"pass","unit":3}      {"t":"overwatch","unit":3}
//   {"t":"cancel","unit":3}                           drop that unit's plan
//   {"t":"focus","unit":3}                            validated no-op (camera is client-side)
//   {"t":"reaction","unit":3,"rule":"shoot"|"none"}   standing playbook rule
//   {"t":"commit"}             this team is ready; the round runs once both are
//   {"t":"new_match"}          only after game over
//
// Server -> client:
//   {"t":"waiting"}
//   {"t":"start","team":"blue"|"red","seed":N,"protocol":1}
//   {"t":"ack","seq":N,"ok":true|false,"error":"..."}
//   {"t":"plans","round":N,"ready":bool,"plans":[<action + "reaction">...]}   own team only
//   {"t":"peer","ready":bool}
//   {"t":"round","round":N,"duration":s,"frames":[...],"shots":[...],"state":{...}}
//   {"t":"state","round":N,"over":bool,"winner":"blue"|"red"|"none"|null,"units":[...]}
//   {"t":"opponent_left"}   {"t":"error","error":"..."}
namespace tactics::net {

constexpr int kProtocolVersion = 1;
constexpr size_t kMaxMessageBytes = 16 * 1024;
constexpr size_t kMaxWaypoints = 32;

enum class ActionKind { Move, Shoot, Pass, Overwatch, Cancel, Focus, Reaction, Commit, NewMatch };

struct Action {
  ActionKind kind = ActionKind::Pass;
  int unit = -1;
  int target = -1;
  std::vector<glm::vec3> waypoints;
  std::optional<float> facing;
  tactics::ReactionRule rule = tactics::ReactionRule::DoNothing;
  int seq = 0;
};

// Parses and structurally validates a client action (types, ranges, sizes
// -- not game rules). On failure returns false with *error set.
bool ParseAction(const Json& message, Action* out, std::string* error);
Json EncodeAction(const Action& action);

const char* TeamName(tactics::Team team);
std::optional<tactics::Team> ParseTeamName(const std::string& name);
const char* PlanName(tactics::PlannedActionType type);
std::optional<tactics::PlannedActionType> ParsePlanName(const std::string& name);

Json EncodeVec3(const glm::vec3& v);
bool DecodeVec3(const Json& j, glm::vec3* out);

}  // namespace tactics::net
