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

// A smooth unit sphere centered at the origin. Callers turn it into heads,
// torsos, joints, and rounded limb segments with non-uniform model scaling.
// Positions and normals are both uploaded so ellipsoids shade smoothly even
// though obstacles continue to use the hard-edged CubeMesh.
class SphereMesh {
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
  // Draws the points as independent segments (pairs) instead of a strip.
  void DrawSegments() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLsizei pointCount_ = 0;
};

// A dynamic triangle soup (GL_TRIANGLES), rebuilt each frame from a point
// list holding three vertices per triangle. Used for the FOV cone overlay,
// which can have holes (obstacle shadows) and so isn't fan-shaped.
class TriangleMesh {
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

// A triangle soup with a per-vertex RGBA color (location 1), for gradient
// overlays. Vertices are interleaved as vec3 position + vec4 color.
class ColorTriangleMesh {
 public:
  struct Vertex {
    glm::vec3 pos;
    glm::vec4 color;
  };
  void Init();
  void Destroy();
  void SetVertices(const std::vector<Vertex>& vertices);
  void Draw() const;

 private:
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLsizei vertexCount_ = 0;
};

}  // namespace gfx
