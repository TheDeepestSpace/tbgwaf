#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "game/Scene.h"
#include "gfx/Mesh.h"
#include "gfx/Shader.h"

namespace gfx {

// PR #178: production defaults; native tests can trade resolution for speed.
struct AmbientOcclusionBakeSettings {
  int directions = 0;  // 0 uses clamp(quality * 16, 16, 256); default quality = 96 rays.
  int depthResolution = 1024;
  int batchLayers = 16;  // At most 16 matrices in the ES 3.0 fragment uniforms.
  float spacing = 0.75f;
};

// Read-only diagnostics. Textures belong to AmbientOcclusion, valid until a
// rebake/Destroy. The native map test attaches each layer for PNG readback.
struct AmbientOcclusionVolume {
  GLuint positive = 0, negative = 0;
  glm::ivec3 dims{0};
  glm::vec3 origin{0.0f};
  float spacing = 0.75f;
  unsigned int bakeCount = 0;
};

// Debug-panel tunables for the thermal contact shading (PR #178 review asked
// for softer, less choppy occlusion with knobs to play with). Defaults are
// the shipped look; the headless visual runner always renders defaults.
struct AmbientOcclusionSettings {
  float radius = 2.4f;    // World-space reach of the shading, in map units.
  float strength = 1.9f;  // Occlusion-to-darkening multiplier.
  int quality = 6;       // Live: samples/ray. Baked: quality * 16 directions.
  int softness = 2;       // Bilateral blur half-width in pixels.
  // "Baked thermal": skip the per-pixel screen-space march and look the
  // shading up from a world-space map baked once per scene (re-baked only
  // when the map geometry, radius or quality change, never for the camera).
  // Strength stays live; softness is unused (the bake is already smooth).
  bool baked = false;
  AmbientOcclusionBakeSettings bake;
};

// Ambient occlusion for the thermal world (screen-space or baked). Capture
// opaque color, world normals and depth; composite before translucent geometry
// and sensor processing. Uses only core GLES3/WebGL2 attachment formats.
// Targets grow to the largest pane seen and are shared by every pane; a
// smaller pane uses their lower-left corner.
class AmbientOcclusion {
 public:
  bool Init();
  void Destroy();
  bool Begin(int width, int height, float background);
  void Composite(GLuint target, int x, int y, const glm::mat4& view,
                 const glm::mat4& projection, bool blackHot,
                 const AmbientOcclusionSettings& settings,
                 const tactics::Scene* world = nullptr,
                 const Shader* depthShader = nullptr,
                 const std::vector<glm::vec4>& contactSpheres = {});
  const AmbientOcclusionVolume& WorldBake() const { return volume_; }

 private:
  Shader occlusionShader_, compositeShader_;
  GLuint sceneFbo_ = 0, occlusionFbo_ = 0, vao_ = 0;
  GLuint color_ = 0, normal_ = 0, depth_ = 0, occlusion_ = 0;
  int width_ = 0, height_ = 0;                    // Current pane.
  int allocatedWidth_ = 0, allocatedHeight_ = 0;  // Target textures.
  bool warned_ = false;

  // Ambient-cube volume (PR #178): six hemispherical visibility channels,
  // independent of the pane/camera. Intermediate depth maps are batch-sized.
  Shader worldOcclusionShader_, worldFillShader_;
  GLuint worldFbo_ = 0;
  std::array<LitTriangleMesh, 3> worldMeshes_;
  AmbientOcclusionVolume volume_;
  uint64_t worldKey_ = 0, bakeKey_ = 0;
  bool EnsureWorldBake(const tactics::Scene& scene, const Shader& depthShader,
                       const AmbientOcclusionSettings& settings);
  void DestroyWorldBake();
};

}  // namespace gfx
