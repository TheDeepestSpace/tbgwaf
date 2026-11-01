#include "gfx/Trees.h"

#include <array>
#include <cmath>
#include <cstdint>

namespace gfx {
namespace {

using Vertex = LitTriangleMesh::Vertex;

void Branch(std::vector<Vertex>& mesh, glm::vec3 a, glm::vec3 b, float radius) {
  const glm::vec3 axis = glm::normalize(b - a);
  const glm::vec3 right = glm::normalize(glm::cross(axis, glm::vec3(0, 0, 1)));
  const glm::vec3 side = glm::cross(axis, right);
  for (int i = 0; i < 6; ++i) {
    const float t0 = i * 6.2831853f / 6, t1 = (i + 1) * 6.2831853f / 6;
    const glm::vec3 n0 = right * std::cos(t0) + side * std::sin(t0);
    const glm::vec3 n1 = right * std::cos(t1) + side * std::sin(t1);
    const glm::vec3 normal = glm::normalize(n0 + n1);
    const Vertex p{a + radius * n0, normal}, q{a + radius * n1, normal};
    const Vertex r{b + radius * 0.55f * n1, normal};
    const Vertex s{b + radius * 0.55f * n0, normal};
    mesh.insert(mesh.end(), {p, q, r, p, r, s});
  }
}

// Subdivided icosahedra with integer-hashed radial displacement: shared
// corners get identical positions, with no random seams. Normals are radial
// from the canopy center so the low-poly lobes shade as one crown.
void Lobe(std::vector<Vertex>& mesh, glm::vec3 center, glm::vec3 size,
          glm::vec3 canopyCenter, uint32_t seed) {
  constexpr float t = 1.618034f;
  const glm::vec3 points[] = {{-1,t,0},{1,t,0},{-1,-t,0},{1,-t,0},
                             {0,-1,t},{0,1,t},{0,-1,-t},{0,1,-t},
                             {t,0,-1},{t,0,1},{-t,0,-1},{-t,0,1}};
  const int faces[][3] = {{0,11,5},{0,5,1},{0,1,7},{0,7,10},{0,10,11},
                          {1,5,9},{5,11,4},{11,10,2},{10,7,6},{7,1,8},
                          {3,9,4},{3,4,2},{3,2,6},{3,6,8},{3,8,9},
                          {4,9,5},{2,4,11},{6,2,10},{8,6,7},{9,8,1}};
  auto vertex = [&](glm::vec3 direction) {
    const glm::vec3 n = glm::normalize(direction);
    const auto q = glm::ivec3((n + 1.0f) * 4096.0f);
    uint32_t h = seed ^ (uint32_t(q.x) * 73856093u) ^
                       (uint32_t(q.y) * 19349663u) ^ (uint32_t(q.z) * 83492791u);
    h = (h ^ (h >> 16)) * 2246822519u;
    const float noise = 0.94f + 0.12f * float(h & 65535u) / 65535.0f;
    const glm::vec3 p = center + size * n * noise;
    return Vertex{p, glm::normalize(p - canopyCenter)};
  };
  for (const auto& f : faces) {
    const glm::vec3 a = glm::normalize(points[f[0]]), b = glm::normalize(points[f[1]]);
    const glm::vec3 c = glm::normalize(points[f[2]]);
    const glm::vec3 ab = glm::normalize(a + b), bc = glm::normalize(b + c);
    const glm::vec3 ca = glm::normalize(c + a);
    for (const glm::vec3& p : {a, ab, ca, b, bc, ab, c, ca, bc, ab, bc, ca})
      mesh.push_back(vertex(p));
  }
}

std::vector<Vertex> BuildTree(int variant) {
  std::vector<Vertex> mesh;
  const glm::vec3 shapes[] = {{1.15f, 0.9f, 1.0f}, {0.82f, 1.3f, 0.85f},
                             {1.0f, 1.0f, 1.1f}};
  const glm::vec3 shape = shapes[variant], center(0.0f, 3.4f, 0.0f);
  Branch(mesh, {0, 0, 0}, {0.08f, 3.5f, 0}, 0.21f);
  Branch(mesh, {0, 1.6f, 0}, {-0.85f, 3.1f, 0.25f}, 0.12f);
  Branch(mesh, {0, 2.0f, 0}, {0.8f, 3.4f, -0.2f}, 0.11f);
  const glm::vec3 offsets[] = {{-0.65f, -0.15f, 0}, {0.65f, 0.05f, -0.2f},
                              {0.05f, 0.0f, 0.65f}, {-0.05f, 0.8f, 0}};
  for (int i = 0; i < 4; ++i) {
    Lobe(mesh, center + offsets[i] * shape, glm::vec3(1.2f, 1.25f, 1.1f) * shape,
         center, 270u + variant * 17u + i);
  }
  return mesh;
}

const std::vector<Vertex>& TreeVariant(int variant) {
  static const std::array<std::vector<Vertex>, 3> variants = {
      BuildTree(0), BuildTree(1), BuildTree(2)};
  const size_t index = static_cast<size_t>(variant);
  return variants[index < variants.size() ? index : 0];
}

}  // namespace

void AppendTreeMesh(const tactics::TreeDecoration& tree,
                    std::vector<Vertex>* vertices, std::vector<GLuint>* indices) {
  const float c = std::cos(tree.yaw), s = std::sin(tree.yaw);
  const glm::mat3 rotation(c, 0.0f, -s, 0.0f, 1.0f, 0.0f, s, 0.0f, c);
  const GLuint base = static_cast<GLuint>(vertices->size());
  const std::vector<Vertex>& model = TreeVariant(tree.variant);
  vertices->reserve(vertices->size() + model.size());
  for (const Vertex& v : model) {
    vertices->push_back({tree.position + rotation * v.pos * tree.scale,
                         rotation * v.normal});
  }
  indices->reserve(indices->size() + model.size());
  for (GLuint i = 0; i < model.size(); ++i) indices->push_back(base + i);
}

}  // namespace gfx
