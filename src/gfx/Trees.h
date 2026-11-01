#pragma once

#include <vector>

#include <GLES3/gl3.h>

#include "game/Scene.h"
#include "gfx/Mesh.h"

namespace gfx {

// Appends one tree's visible geometry (trunk, branches, canopy lobes) as a
// regular static mesh, CPU-transformed by the tree's position/yaw/scale.
// Trees are baked once per scene and then drawn, sun-shadowed and FOV-tinted
// exactly like street furniture: no instancing and no tree-specific shader.
void AppendTreeMesh(const tactics::TreeDecoration& tree,
                    std::vector<LitTriangleMesh::Vertex>* vertices,
                    std::vector<GLuint>* indices);

}  // namespace gfx
