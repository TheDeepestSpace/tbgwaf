#pragma once

#include <array>
#include <vector>

#include "game/Scene.h"
#include "gfx/Shader.h"

namespace gfx {

// Three static meshes, one instanced draw per variant and pass. No per-frame
// placement, tessellation, wind updates or camera/LOS probes.
class Trees {
 public:
  bool Init();
  void Destroy();
  void SetTrees(const std::vector<tactics::TreeDecoration>& trees);
  void Draw(const glm::mat4& viewProj, const glm::vec3& lightDir,
            bool thermal, bool blackHot) const;
  // Sunlight only. The FOV shadow mask does not cast these meshes: it casts
  // each tree's gameplay occluder prism instead (BuildFovTreeMesh), so the
  // overlay matches gameplay LOS rather than the visual silhouette.
  void DrawDepth(const glm::mat4& lightSpace) const;

 private:
  struct Batch {
    GLuint vao = 0, vertices = 0, instances = 0;
    GLsizei vertexCount = 0, instanceCount = 0;
  };
  void DrawBatches() const;
  std::array<Batch, 3> batches_{};
  std::vector<tactics::TreeDecoration> key_;
  Shader shader_, depthShader_;
};

}  // namespace gfx
