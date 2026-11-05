#pragma once

#include "gfx/Shader.h"

namespace gfx {

// Debug-panel tunables for the thermal contact shading (PR #178 review asked
// for softer, less choppy occlusion with knobs to play with). Defaults are
// the shipped look; the headless visual runner always renders defaults.
struct AmbientOcclusionSettings {
  float radius = 6.0f;    // World-space reach of the shading, in map units.
  float strength = 1.0f;  // Occlusion-to-darkening multiplier.
  int quality = 4;        // Depth samples marched per ray (more = smoother).
  int softness = 3;       // Bilateral blur half-width in pixels.
};

// Grayscale NPR look (tactical-comic cel shading + AO + screen-space
// outlines). Distances are screen pixels so the line weight does not change
// with zoom or pane size. Defaults are the shipped look.
struct NprSettings {
  float outlineThickness = 1.5f;  // Outline reach in screen pixels (0 = off).
  float outlineStrength = 0.85f;  // Outline opacity, 0..1.
  // World size of the finest detail worth outlining (window frames, sills).
  // From afar, edges on features near/below this size would alias into
  // dotted lines, so they fade out and only large steps (silhouettes) keep
  // their outline -- a distance LOD that declutters small screens. 0 = off.
  float detailSize = 0.1f;
  int celBands = 4;               // Lit tone steps (>= 2).
  float celSoftness = 0.5f;       // Band edge width, 0 (hard) .. 1 (fully smooth).
  // Cheaper than the thermal defaults: the AO is evaluated at half
  // resolution and blurred over a small kernel (mobile GPUs).
  AmbientOcclusionSettings ao{4.0f, 0.9f, 3, 1};
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
                 const AmbientOcclusionSettings& settings,
                 const NprSettings* outlines = nullptr);  // Non-null: NPR outlines on top.
  // NPR variant: half-res AO, then one full-res pass that applies the AO
  // and draws depth/normal outlines. Figures (normal alpha 0) keep their
  // team color; only a lightly darkened rim is drawn on them.
  void CompositeNpr(GLuint target, int x, int y, const glm::mat4& view,
                    const glm::mat4& projection, const NprSettings& settings);

 private:
  Shader occlusionShader_, compositeShader_, nprShader_;
  GLuint sceneFbo_ = 0, occlusionFbo_ = 0, vao_ = 0;
  GLuint color_ = 0, normal_ = 0, depth_ = 0, occlusion_ = 0;
  int width_ = 0, height_ = 0;                    // Current pane.
  int allocatedWidth_ = 0, allocatedHeight_ = 0;  // Target textures.
  bool warned_ = false;
};

}  // namespace gfx
