#pragma once

#include <array>
#include <functional>
#include <vector>

#include <GLES3/gl3.h>
#include <glm/glm.hpp>

namespace gfx {

// Read-only per-frame render counters for the interactive debug panel.
// Draw() calls record into the sink installed by SceneRenderer::RenderPane
// (see ScopedDrawStats); with no sink installed (e.g. the visual runner)
// recording is a no-op.
struct RenderFrameStats {
  int drawCalls = 0;
  long long vertices = 0;
  long long triangles = 0;
};

void RecordDraw(long long vertices, long long triangles);

// The six clip planes of a view-projection matrix, for culling a mesh's
// chunks against a camera or a depth map's frustum.
class Frustum {
 public:
  explicit Frustum(const glm::mat4& viewProj);
  // False only when the box lies wholly outside one plane (conservative).
  bool Intersects(const glm::vec3& min, const glm::vec3& max) const;

 private:
  std::array<glm::vec4, 6> planes_;
};

class ScopedDrawStats {
 public:
  explicit ScopedDrawStats(RenderFrameStats* sink);
  ~ScopedDrawStats();
  ScopedDrawStats(const ScopedDrawStats&) = delete;
  ScopedDrawStats& operator=(const ScopedDrawStats&) = delete;

 private:
  RenderFrameStats* previous_;
};

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

// Accepts a chunk of a LitTriangleMesh by its world-space bounds.
using ChunkFilter = std::function<bool(const glm::vec3& min, const glm::vec3& max)>;

// An indexed triangle mesh with per-vertex normals (location 1), uploaded
// once per scene rather than per frame. Used for the terrain ground mesh,
// which is drawn with the lit shader (so it needs normals, unlike the
// dynamic overlay meshes above).
class LitTriangleMesh {
 public:
  struct Vertex {
    glm::vec3 pos;
    glm::vec3 normal;
  };
  void Init();
  void Destroy();
  void SetMesh(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices);
  // SetMesh for a large static mesh: the triangles are first grouped into a
  // gridCells x gridCells grid over the mesh's XZ extent, so DrawVisible can
  // skip whole cells. `triangleGroups` (optional) tags each triangle with an
  // id (the obstacle or building it belongs to) that DrawVisible can leave
  // out; within a cell triangles are ordered by group, otherwise keeping
  // their original order.
  void SetChunkedMesh(const std::vector<Vertex>& vertices, const std::vector<GLuint>& indices,
                      int gridCells, const std::vector<int>* triangleGroups = nullptr);
  void Draw() const;
  // Draws only the chunks whose bounds `visible` accepts (everything when
  // the mesh is not chunked), minus any triangle group `skipGroup` rejects,
  // merging adjacent ranges into one draw call.
  void DrawVisible(const ChunkFilter& visible,
                   const std::function<bool(int group)>& skipGroup = nullptr) const;
  bool HasGeometry() const { return indexCount_ > 0; }

 private:
  struct Range {
    GLsizei firstIndex;
    GLsizei indexCount;
  };
  struct GroupRange {
    int group;
    Range range;
  };
  struct Chunk {
    glm::vec3 min;
    glm::vec3 max;
    Range range;
    size_t firstGroup, groupCount;  // Slice of groups_.
  };
  void DrawRange(GLsizei firstIndex, GLsizei indexCount) const;
  GLuint vao_ = 0;
  GLuint vbo_ = 0;
  GLuint ebo_ = 0;
  GLsizei indexCount_ = 0;
  std::vector<Chunk> chunks_;
  std::vector<GroupRange> groups_;
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
