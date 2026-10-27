#pragma once

#include <cstdint>

#include "game/Scene.h"
#include "gfx/Mesh.h"

namespace gfx {
// PR #178: share the FOV caster and terrain builders with the thermal bake.
// Upload once per content change, then reuse these meshes for every direction.
uint64_t WorldSceneFingerprint(const tactics::Scene& scene);
void BuildWorldSceneMeshes(const tactics::Scene& scene, LitTriangleMesh* objects,
                          LitTriangleMesh* ground, LitTriangleMesh* raised);
}  // namespace gfx
