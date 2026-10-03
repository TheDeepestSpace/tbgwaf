#include "gfx/Mesh.h"

#include <cmath>
#include <cstddef>
#include <vector>

namespace gfx {

namespace {
RenderFrameStats* g_drawStatsSink = nullptr;
}

void RecordDraw(long long vertices, long long triangles) {
  if (!g_drawStatsSink) return;
  ++g_drawStatsSink->drawCalls;
  g_drawStatsSink->vertices += vertices;
  g_drawStatsSink->triangles += triangles;
}

ScopedDrawStats::ScopedDrawStats(RenderFrameStats* sink) : previous_(g_drawStatsSink) {
  g_drawStatsSink = sink;
}
ScopedDrawStats::~ScopedDrawStats() { g_drawStatsSink = previous_; }

void CubeMesh::Init() {
  // clang-format off
  static const float kVertices[] = {
      // position     normal
      0, 0, 0,  0, 0,-1,  1, 0, 0,  0, 0,-1,  1, 1, 0,  0, 0,-1,  0, 1, 0,  0, 0,-1,
      0, 0, 1,  0, 0, 1,  1, 0, 1,  0, 0, 1,  1, 1, 1,  0, 0, 1,  0, 1, 1,  0, 0, 1,
      0, 0, 0, -1, 0, 0,  0, 1, 0, -1, 0, 0,  0, 1, 1, -1, 0, 0,  0, 0, 1, -1, 0, 0,
      1, 0, 0,  1, 0, 0,  1, 0, 1,  1, 0, 0,  1, 1, 1,  1, 0, 0,  1, 1, 0,  1, 0, 0,
      0, 1, 0,  0, 1, 0,  1, 1, 0,  0, 1, 0,  1, 1, 1,  0, 1, 0,  0, 1, 1,  0, 1, 0,
      0, 0, 0,  0,-1, 0,  0, 0, 1,  0,-1, 0,  1, 0, 1,  0,-1, 0,  1, 0, 0,  0,-1, 0,
  };
  static const GLuint kIndices[] = {
      0, 1, 2, 2, 3, 0,  // back
      4, 6, 5, 6, 4, 7,  // front
      8, 9,10,10,11, 8,  // left
     12,13,14,14,15,12,  // right
     16,17,18,18,19,16,  // top
     20,21,22,22,23,20,  // bottom
  };
  // clang-format on
  indexCount_ = static_cast<GLsizei>(sizeof(kIndices) / sizeof(kIndices[0]));

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(kVertices), kVertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(kIndices), kIndices, GL_STATIC_DRAW);

  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void*>(3 * sizeof(float)));
  glBindVertexArray(0);
}

void CubeMesh::Destroy() {
  if (ebo_) glDeleteBuffers(1, &ebo_);
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = ebo_ = 0;
}

void CubeMesh::Draw() const {
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
  RecordDraw(indexCount_, indexCount_ / 3);
  glBindVertexArray(0);
}

void SphereMesh::Init() {
  constexpr int kSlices = 20;
  constexpr int kStacks = 14;
  constexpr float kPi = 3.14159265358979323846f;
  std::vector<float> vertices;
  std::vector<GLuint> indices;
  vertices.reserve((kStacks + 1) * (kSlices + 1) * 6);
  indices.reserve(kStacks * kSlices * 6);

  for (int stack = 0; stack <= kStacks; ++stack) {
    const float theta = kPi * static_cast<float>(stack) / kStacks;
    const float y = std::cos(theta);
    const float ring = std::sin(theta);
    for (int slice = 0; slice <= kSlices; ++slice) {
      const float phi = 2.0f * kPi * static_cast<float>(slice) / kSlices;
      const float x = ring * std::cos(phi);
      const float z = ring * std::sin(phi);
      vertices.insert(vertices.end(), {x, y, z, x, y, z});
    }
  }
  for (int stack = 0; stack < kStacks; ++stack) {
    for (int slice = 0; slice < kSlices; ++slice) {
      const GLuint a = static_cast<GLuint>(stack * (kSlices + 1) + slice);
      const GLuint b = a + kSlices + 1;
      indices.insert(indices.end(), {a, b, a + 1, a + 1, b, b + 1});
    }
  }
  indexCount_ = static_cast<GLsizei>(indices.size());

  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(),
               GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), nullptr);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float),
                        reinterpret_cast<void*>(3 * sizeof(float)));
  glBindVertexArray(0);
}

void SphereMesh::Destroy() {
  if (ebo_) glDeleteBuffers(1, &ebo_);
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = ebo_ = 0;
}

void SphereMesh::Draw() const {
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
  RecordDraw(indexCount_, indexCount_ / 3);
  glBindVertexArray(0);
}

void LineMesh::Init() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glBindVertexArray(0);
}

void LineMesh::Destroy() {
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = 0;
}

void LineMesh::SetPoints(const std::vector<glm::vec3>& points) {
  pointCount_ = static_cast<GLsizei>(points.size());
  if (pointCount_ == 0) return;
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(),
               GL_DYNAMIC_DRAW);
}

void LineMesh::Draw() const {
  if (pointCount_ < 2) return;
  glBindVertexArray(vao_);
  glDrawArrays(GL_LINE_STRIP, 0, pointCount_);
  RecordDraw(pointCount_, 0);
  glBindVertexArray(0);
}

void LineMesh::DrawSegments() const {
  if (pointCount_ < 2) return;
  glBindVertexArray(vao_);
  glDrawArrays(GL_LINES, 0, pointCount_);
  RecordDraw(pointCount_, 0);
  glBindVertexArray(0);
}

void TriangleMesh::Init() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glBindVertexArray(0);
}

void TriangleMesh::Destroy() {
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = 0;
}

void TriangleMesh::SetPoints(const std::vector<glm::vec3>& points) {
  pointCount_ = static_cast<GLsizei>(points.size());
  if (pointCount_ == 0) return;
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(),
               GL_DYNAMIC_DRAW);
}

void TriangleMesh::Draw() const {
  if (pointCount_ < 3) return;
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, pointCount_);
  RecordDraw(pointCount_, pointCount_ / 3);
  glBindVertexArray(0);
}

void LitTriangleMesh::Init() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, normal)));
  glBindVertexArray(0);
}

void LitTriangleMesh::Destroy() {
  if (ebo_) glDeleteBuffers(1, &ebo_);
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = ebo_ = 0;
  indexCount_ = 0;
}

void LitTriangleMesh::SetMesh(const std::vector<Vertex>& vertices,
                              const std::vector<GLuint>& indices) {
  indexCount_ = static_cast<GLsizei>(indices.size());
  if (indexCount_ == 0) return;
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(),
               GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(GLuint), indices.data(),
               GL_STATIC_DRAW);
  glBindVertexArray(0);
}

void LitTriangleMesh::Draw() const {
  if (indexCount_ == 0) return;
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, indexCount_, GL_UNSIGNED_INT, nullptr);
  glBindVertexArray(0);
}

void ColorTriangleMesh::Init() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, pos)));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                        reinterpret_cast<void*>(offsetof(Vertex, color)));
  glBindVertexArray(0);
}

void ColorTriangleMesh::Destroy() {
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = 0;
}

void ColorTriangleMesh::SetVertices(const std::vector<Vertex>& vertices) {
  vertexCount_ = static_cast<GLsizei>(vertices.size());
  if (vertexCount_ == 0) return;
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(),
               GL_DYNAMIC_DRAW);
}

void ColorTriangleMesh::Draw() const {
  if (vertexCount_ < 3) return;
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, vertexCount_);
  RecordDraw(vertexCount_, vertexCount_ / 3);
  glBindVertexArray(0);
}

}  // namespace gfx
