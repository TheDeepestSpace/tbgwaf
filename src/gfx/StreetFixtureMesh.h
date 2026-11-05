#pragma once

// Street fixture models (issue #153): the batched static geometry of parked
// vehicles, benches, dumpsters and the like, built from an Obstacle's
// gameplay box. Shared by the scene renderer (lighting, shadows and FOV
// receivers all use the same triangles) and the asset gallery, which shows
// each model alone on a turntable.

#include <vector>

#include <GLES3/gl3.h>
#include <glm/glm.hpp>

#include "game/Types.h"
#include "gfx/Mesh.h"

namespace gfx {

using LitVertices = std::vector<LitTriangleMesh::Vertex>;

void AppendTriangle(glm::vec3 a, glm::vec3 b, glm::vec3 c, glm::vec3 normal,
                    LitVertices* vertices, std::vector<GLuint>* indices);

// Model matrix of an axis-aligned box: unit cube from `minCorner` scaled by `size`.
glm::mat4 BoxModel(const glm::vec3& minCorner, const glm::vec3& size);

// Unit cube transformed exactly like CubeMesh.
void AppendBoxModel(const glm::mat4& model, LitVertices* vertices, std::vector<GLuint>* indices);

// The fixture's visible model inside its gameplay box (`obstacle.bounds`,
// standing on `obstacle.fixtureBaseY`, heading `obstacle.fixtureYaw`). An
// obstacle without a model appends its plain box.
void AppendStreetFixture(const tactics::Obstacle& obstacle, LitVertices* vertices,
                         std::vector<GLuint>* indices);

}  // namespace gfx
