#include "gfx/Trees.h"

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "gfx/Mesh.h"

namespace gfx {
namespace {

const char* kVertex = R"(#version 300 es
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec4 aColor;
layout(location = 3) in vec4 aPositionScale;
layout(location = 4) in vec2 aYawTint;
uniform mat4 uViewProj;
out vec3 vNormal;
out vec4 vColor;
void main() {
  float c = cos(aYawTint.x), s = sin(aYawTint.x);
  mat3 rotation = mat3(c, 0.0, -s, 0.0, 1.0, 0.0, s, 0.0, c);
  vec3 world = aPositionScale.xyz + rotation * aPos * aPositionScale.w;
  vNormal = rotation * aNormal;
  vColor = aColor;  // Same color as buildings: no per-tree tint.
  gl_Position = uViewProj * vec4(world, 1.0);
}
)";

const char* kFragment = R"(#version 300 es
precision highp float;
in vec3 vNormal;
in vec4 vColor;
uniform vec3 uLightDir;
uniform int uThermal;
uniform int uBlackHot;
layout(location = 0) out vec4 FragColor;
layout(location = 1) out vec4 WorldNormal;
void main() {
  vec3 n = normalize(vNormal);
  WorldNormal = vec4(n * 0.5 + 0.5, 1.0);
  if (uThermal != 0) {
    float heat = mix(0.18, 0.10, max(n.y, 0.0));
    FragColor = vec4(vec3(uBlackHot != 0 ? 1.0 - heat : heat), 1.0);
  } else {
    // Canopy-wide radial normals and wrapped diffuse soften the joins
    // between the low-poly lobes. Trunks keep ordinary diffuse lighting.
    float wrap = 0.4 * vColor.a;
    float diffuse = max((dot(n, -uLightDir) + wrap) / (1.0 + wrap), 0.0);
    FragColor = vec4(vColor.rgb * (0.38 + 0.62 * diffuse), 1.0);
  }
}
)";

const char* kDepthFragment = R"(#version 300 es
precision mediump float;
void main() {}
)";

struct Vertex { glm::vec3 position, normal; glm::vec4 color; };
struct Instance { glm::vec4 positionScale; glm::vec2 yawTint; };

void Branch(std::vector<Vertex>& mesh, glm::vec3 a, glm::vec3 b, float radius) {
  const glm::vec3 axis = glm::normalize(b - a);
  const glm::vec3 right = glm::normalize(glm::cross(axis, glm::vec3(0, 0, 1)));
  const glm::vec3 side = glm::cross(axis, right);
  const glm::vec4 bark(0.55f, 0.55f, 0.6f, 0.0f);  // Building color.
  for (int i = 0; i < 6; ++i) {
    const float t0 = i * 6.2831853f / 6, t1 = (i + 1) * 6.2831853f / 6;
    const glm::vec3 n0 = right * std::cos(t0) + side * std::sin(t0);
    const glm::vec3 n1 = right * std::cos(t1) + side * std::sin(t1);
    const glm::vec3 normal = glm::normalize(n0 + n1);
    const Vertex p{a + radius * n0, normal, bark}, q{a + radius * n1, normal, bark};
    const Vertex r{b + radius * 0.55f * n1, normal, bark};
    const Vertex s{b + radius * 0.55f * n0, normal, bark};
    mesh.insert(mesh.end(), {p, q, r, p, r, s});
  }
}

// Subdivided icosahedra with integer-hashed radial displacement: shared
// corners get identical positions, with no random seams.
void Lobe(std::vector<Vertex>& mesh, glm::vec3 center, glm::vec3 size,
          glm::vec3 canopyCenter, glm::vec3 color, uint32_t seed) {
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
    return Vertex{p, glm::normalize(p - canopyCenter), glm::vec4(color, 1.0f)};
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
  const glm::vec3 colors[] = {{0.55f, 0.55f, 0.6f}, {0.55f, 0.55f, 0.6f},
                             {0.55f, 0.55f, 0.6f}};  // Building color.
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
         center, colors[variant], 270u + variant * 17u + i);
  }
  return mesh;
}

}  // namespace

bool Trees::Init() {
  if (!shader_.Compile(kVertex, kFragment) || !depthShader_.Compile(kVertex, kDepthFragment))
    return false;
  for (size_t i = 0; i < batches_.size(); ++i) {
    Batch& batch = batches_[i];
    const auto mesh = BuildTree(static_cast<int>(i));
    batch.vertexCount = static_cast<GLsizei>(mesh.size());
    glGenVertexArrays(1, &batch.vao);
    glBindVertexArray(batch.vao);
    glGenBuffers(1, &batch.vertices);
    glBindBuffer(GL_ARRAY_BUFFER, batch.vertices);
    glBufferData(GL_ARRAY_BUFFER, mesh.size() * sizeof(Vertex), mesh.data(), GL_STATIC_DRAW);
    const size_t offsets[] = {offsetof(Vertex, position), offsetof(Vertex, normal),
                              offsetof(Vertex, color)};
    for (GLuint attr = 0; attr < 3; ++attr) {
      glEnableVertexAttribArray(attr);
      glVertexAttribPointer(attr, attr == 2 ? 4 : 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                            reinterpret_cast<void*>(offsets[attr]));
    }
    glGenBuffers(1, &batch.instances);
    glBindBuffer(GL_ARRAY_BUFFER, batch.instances);
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, sizeof(Instance),
                          reinterpret_cast<void*>(offsetof(Instance, positionScale)));
    glVertexAttribDivisor(3, 1);
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 2, GL_FLOAT, GL_FALSE, sizeof(Instance),
                          reinterpret_cast<void*>(offsetof(Instance, yawTint)));
    glVertexAttribDivisor(4, 1);
  }
  glBindVertexArray(0);
  return true;
}

void Trees::Destroy() {
  for (Batch& batch : batches_) {
    glDeleteVertexArrays(1, &batch.vao);
    glDeleteBuffers(1, &batch.vertices);
    glDeleteBuffers(1, &batch.instances);
    batch = {};
  }
  key_.clear();
}

void Trees::SetTrees(const std::vector<tactics::TreeDecoration>& trees) {
  // Compare contents: GameLogic and its vector storage can be reused for a new map.
  if (key_ == trees) return;
  key_ = trees;
  for (size_t i = 0; i < batches_.size(); ++i) {
    std::vector<Instance> instances;
    for (const auto& tree : trees) {
      if (tree.variant == static_cast<int>(i))
        instances.push_back({glm::vec4(tree.position, tree.scale), {tree.yaw, tree.tint}});
    }
    Batch& batch = batches_[i];
    batch.instanceCount = static_cast<GLsizei>(instances.size());
    glBindBuffer(GL_ARRAY_BUFFER, batch.instances);
    glBufferData(GL_ARRAY_BUFFER, instances.size() * sizeof(Instance), instances.data(),
                 GL_STATIC_DRAW);
  }
}

void Trees::DrawBatches() const {
  for (const Batch& batch : batches_) {
    if (!batch.instanceCount) continue;
    glBindVertexArray(batch.vao);
    glDrawArraysInstanced(GL_TRIANGLES, 0, batch.vertexCount, batch.instanceCount);
    const long long vertices = static_cast<long long>(batch.vertexCount) * batch.instanceCount;
    RecordDraw(vertices, vertices / 3);
  }
  glBindVertexArray(0);
}

void Trees::Draw(const glm::mat4& viewProj, const glm::vec3& lightDir,
                 bool thermal, bool blackHot) const {
  shader_.Use();
  shader_.SetMat4("uViewProj", viewProj);
  shader_.SetVec3("uLightDir", lightDir);
  shader_.SetInt("uThermal", thermal);
  shader_.SetInt("uBlackHot", blackHot);
  DrawBatches();
}

void Trees::DrawDepth(const glm::mat4& lightSpace) const {
  depthShader_.Use();
  depthShader_.SetMat4("uViewProj", lightSpace);
  DrawBatches();
}

}  // namespace gfx
