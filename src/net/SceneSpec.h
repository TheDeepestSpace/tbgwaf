#pragma once

#include <string>

#include "game/Scene.h"
#include "net/Json.h"

namespace tactics::net {

// Declarative scene description shared by the YAML scenario files and the
// test-only server control tap (the server cannot receive a built Scene, and
// the client rebuilds it from the same spec in the `start` message):
//
//   {"map": {"generate": {"seed": N, "type": "urban"|"hilly", "arteries": N,
//                         "artery_width": W, "local_street_width": W,
//                         "elevated": bool},
//            "obstacles": [{"center": [x, z], "half_extent": [x, z],
//                           "height": h, "climbable": bool}, ...]},
//    "units": [{"id": 0, "team": "blue", "position": [x, y, z],
//               "facing_degrees": d}, ...]}
//
// A generated map keeps its generated units unless `units` is present.
bool SceneFromSpec(const Json& spec, tactics::Scene* out, std::string* error);

}  // namespace tactics::net
