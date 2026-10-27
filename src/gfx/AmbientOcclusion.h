#pragma once

#include <array>
#include <cstdint>

#include "gfx/Shader.h"

namespace gfx {

// Debug-panel tunables for the thermal contact shading (PR #178 review asked
// for softer, less choppy occlusion with knobs to play with). Defaults are
// the shipped look; the headless visual runner always renders defaults.
struct AmbientOcclusionSettings {
  float radius = 2.4f;    // World-space reach of the shading, in map units.
  float strength = 1.9f;  // Occlusion-to-darkening multiplier.
  int quality = 6;        // Depth samples marched per ray (more = smoother).
  int softness = 2;       // Bilateral blur half-width in pixels.
  // "Baked thermal": cache each pane's occlusion pass and re-run it only
  // when the camera, pane size, radius/quality or scene signature changes.
  // Strength and softness are applied at composite time, so they stay live.
  bool baked = false;
};

// Pane-local, screen-space ambient occlusion for the thermal world. Capture
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
                 const AmbientOcclusionSettings& settings, uint64_t sceneKey = 0);

 private:
  Shader occlusionShader_, compositeShader_;
  GLuint sceneFbo_ = 0, occlusionFbo_ = 0, vao_ = 0;
  GLuint color_ = 0, normal_ = 0, depth_ = 0, occlusion_ = 0;
  int width_ = 0, height_ = 0;                    // Current pane.
  int allocatedWidth_ = 0, allocatedHeight_ = 0;  // Target textures.
  bool warned_ = false;

  // Baked occlusion, one slot per pane origin (a split window has two).
  struct BakeSlot {
    GLuint texture = 0;
    bool valid = false;
    int x = 0, y = 0, width = 0, height = 0;
    glm::mat4 inverse{1.0f};
    float radius = 0.0f;
    int quality = 0;
    uint64_t sceneKey = 0;
  };
  std::array<BakeSlot, 4> bakes_;
  void DestroyBakes();
};

}  // namespace gfx
