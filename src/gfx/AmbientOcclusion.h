#pragma once

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
                 const AmbientOcclusionSettings& settings);

 private:
  Shader occlusionShader_, compositeShader_;
  GLuint sceneFbo_ = 0, occlusionFbo_ = 0, vao_ = 0;
  GLuint color_ = 0, normal_ = 0, depth_ = 0, occlusion_ = 0;
  int width_ = 0, height_ = 0;                    // Current pane.
  int allocatedWidth_ = 0, allocatedHeight_ = 0;  // Target textures.
  bool warned_ = false;
};

}  // namespace gfx
