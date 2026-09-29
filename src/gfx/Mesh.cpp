#include "gfx/Mesh.h"

namespace gfx {

void CubeMesh::Init() {
  // clang-format off
  static const float kVertices[] = {
      0, 0, 0,  1, 0, 0,  1, 1, 0,  0, 1, 0,  // back  (z=0)
      0, 0, 1,  1, 0, 1,  1, 1, 1,  0, 1, 1,  // front (z=1)
  };
  static const GLuint kIndices[] = {
      0, 1, 2, 2, 3, 0,  // back
      4, 6, 5, 6, 4, 7,  // front
      0, 3, 7, 7, 4, 0,  // left
      1, 5, 6, 6, 2, 1,  // right
      3, 2, 6, 6, 7, 3,  // top
      0, 4, 5, 5, 1, 0,  // bottom
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
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
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
  glBindVertexArray(0);
}

void TriangleFanMesh::Init() {
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glBindVertexArray(0);
}

void TriangleFanMesh::Destroy() {
  if (vbo_) glDeleteBuffers(1, &vbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  vao_ = vbo_ = 0;
}

void TriangleFanMesh::SetPoints(const std::vector<glm::vec3>& points) {
  pointCount_ = static_cast<GLsizei>(points.size());
  if (pointCount_ == 0) return;
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(glm::vec3), points.data(),
               GL_DYNAMIC_DRAW);
}

void TriangleFanMesh::Draw() const {
  if (pointCount_ < 3) return;
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLE_FAN, 0, pointCount_);
  glBindVertexArray(0);
}

}  // namespace gfx
