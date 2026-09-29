#pragma once

#include <vector>

#include <GLES3/gl3.h>
#include <glm/glm.hpp>

namespace gfx {

// A unit cube spanning [0,1]^3, drawn with glDrawElements. Callers scale and
// translate it via the model matrix to represent obstacles, unit bodies, etc.
class CubeMesh {
 public:
  void Init();
  void Destroy();
  void Draw() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;
  GLsizei indexCount_ = 0;
};

// A dynamic polyline (GL_LINE_STRIP), rebuilt each frame from a point list.
// Used for the move path preview.
class LineMesh {
 public:
  void Init();
  void Destroy();
  void SetPoints(const std::vector<glm::vec3>& points);
  void Draw() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLsizei pointCount_ = 0;
};

// A dynamic filled polygon (GL_TRIANGLE_FAN), rebuilt each frame from a
// point list whose first entry is the fan's center. Used for the FOV cone
// overlay (center = unit position, remaining points = the cone's arc).
class TriangleFanMesh {
 public:
  void Init();
  void Destroy();
  void SetPoints(const std::vector<glm::vec3>& points);
  void Draw() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLsizei pointCount_ = 0;
};

}  // namespace gfx
