#include "gfx/Mesh.h"

#include <algorithm>
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

Frustum::Frustum(const glm::mat4& m) {
  // Gribb/Hartmann: each plane is the last row of the matrix plus or minus
  // one of the others (glm is column-major, so row i is m[*][i]).
  const auto row = [&](int i) { return glm::vec4(m[0][i], m[1][i], m[2][i], m[3][i]); };
  for (int axis = 0; axis < 3; ++axis) {
    planes_[2 * axis] = row(3) + row(axis);
    planes_[2 * axis + 1] = row(3) - row(axis);
  }
}

bool Frustum::Intersects(const glm::vec3& min, const glm::vec3& max) const {
  for (const glm::vec4& plane : planes_) {
    // The box corner furthest along the plane's normal.
    const glm::vec3 corner(plane.x >= 0.0f ? max.x : min.x, plane.y >= 0.0f ? max.y : min.y,
                           plane.z >= 0.0f ? max.z : min.z);
    if (plane.x * corner.x + plane.y * corner.y + plane.z * corner.z + plane.w < 0.0f) {
      return false;
    }
  }
  return true;
}

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
  chunks_.clear();
  groups_.clear();
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

void LitTriangleMesh::SetChunkedMesh(const std::vector<Vertex>& vertices,
                                     const std::vector<GLuint>& indices, int gridCells,
                                     const std::vector<int>* triangleGroups) {
  const size_t triangles = indices.size() / 3;
  if (triangles == 0 || gridCells < 2) {
    SetMesh(vertices, indices);
    return;
  }
  glm::vec2 lo(INFINITY), hi(-INFINITY);
  for (GLuint index : indices) {
    lo = glm::min(lo, glm::vec2(vertices[index].pos.x, vertices[index].pos.z));
    hi = glm::max(hi, glm::vec2(vertices[index].pos.x, vertices[index].pos.z));
  }
  const glm::vec2 cellSize = glm::max((hi - lo) / static_cast<float>(gridCells), glm::vec2(1e-3f));
  // Cells are numbered row by row, alternating direction, so a frustum's
  // accepted cells in neighbouring rows tend to sit next to each other in
  // the index buffer and merge into fewer draws.
  const auto cellOf = [&](size_t triangle) {
    glm::vec2 centroid(0.0f);
    for (int k = 0; k < 3; ++k) {
      const glm::vec3& p = vertices[indices[3 * triangle + k]].pos;
      centroid += glm::vec2(p.x, p.z) / 3.0f;
    }
    const glm::ivec2 c = glm::clamp(glm::ivec2((centroid - lo) / cellSize), glm::ivec2(0),
                                    glm::ivec2(gridCells - 1));
    return c.y * gridCells + (c.y % 2 == 0 ? c.x : gridCells - 1 - c.x);
  };
  const auto groupOf = [&](size_t triangle) {
    return triangleGroups ? (*triangleGroups)[triangle] : 0;
  };
  std::vector<int> cell(triangles);
  std::vector<size_t> order(triangles);
  for (size_t t = 0; t < triangles; ++t) {
    cell[t] = cellOf(t);
    order[t] = t;
  }
  std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
    return cell[a] != cell[b] ? cell[a] < cell[b] : groupOf(a) < groupOf(b);
  });
  std::vector<GLuint> sorted(triangles * 3);
  for (size_t slot = 0; slot < triangles; ++slot) {
    for (int k = 0; k < 3; ++k) sorted[3 * slot + k] = indices[3 * order[slot] + k];
  }
  SetMesh(vertices, sorted);
  for (size_t slot = 0; slot < triangles; ++slot) {
    const size_t t = order[slot];
    const GLsizei first = static_cast<GLsizei>(3 * slot);
    if (slot == 0 || cell[t] != cell[order[slot - 1]]) {
      chunks_.push_back({glm::vec3(INFINITY), glm::vec3(-INFINITY), {first, 0}, groups_.size(), 0});
    }
    Chunk& chunk = chunks_.back();
    for (int k = 0; k < 3; ++k) {
      chunk.min = glm::min(chunk.min, vertices[indices[3 * t + k]].pos);
      chunk.max = glm::max(chunk.max, vertices[indices[3 * t + k]].pos);
    }
    chunk.range.indexCount += 3;
    if (!triangleGroups) continue;
    if (chunk.groupCount == 0 || groups_.back().group != groupOf(t)) {
      groups_.push_back({groupOf(t), {first, 0}});
      ++chunk.groupCount;
    }
    groups_.back().range.indexCount += 3;
  }
}

void LitTriangleMesh::DrawRange(GLsizei firstIndex, GLsizei indexCount) const {
  glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT,
                 reinterpret_cast<const void*>(static_cast<size_t>(firstIndex) * sizeof(GLuint)));
  RecordDraw(indexCount, indexCount / 3);
}

void LitTriangleMesh::Draw() const {
  if (indexCount_ == 0) return;
  glBindVertexArray(vao_);
  DrawRange(0, indexCount_);
  glBindVertexArray(0);
}

void LitTriangleMesh::DrawVisible(const ChunkFilter& visible,
                                  const std::function<bool(int group)>& skipGroup) const {
  if (chunks_.empty()) {
    Draw();
    return;
  }
  glBindVertexArray(vao_);
  Range run{0, 0};
  const auto add = [&](const Range& range) {
    if (run.indexCount > 0 && run.firstIndex + run.indexCount == range.firstIndex) {
      run.indexCount += range.indexCount;
      return;
    }
    if (run.indexCount > 0) DrawRange(run.firstIndex, run.indexCount);
    run = range;
  };
  for (const Chunk& chunk : chunks_) {
    if (!visible(chunk.min, chunk.max)) continue;
    if (!skipGroup || chunk.groupCount == 0) {
      add(chunk.range);
      continue;
    }
    for (size_t g = chunk.firstGroup; g < chunk.firstGroup + chunk.groupCount; ++g) {
      if (!skipGroup(groups_[g].group)) add(groups_[g].range);
    }
  }
  if (run.indexCount > 0) DrawRange(run.firstIndex, run.indexCount);
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
