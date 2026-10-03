#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Unit.h"
#include "game/Weapon.h"
#include "gfx/Mesh.h"
#include "gfx/Shader.h"

namespace gallery {

// Asset & animation gallery (issue #126): the catalog of viewable items and
// a small self-contained renderer for one item at a time. Shared verbatim by
// the interactive web viewer (web/gallery.html + gallery_main.cpp) and the
// native viewer.

enum class ItemKind {
  WeaponModel,  // The weapon alone on an auto-rotating turntable.
  Animation,    // A full figure looping one animation of its weapon class.
};

enum class AnimKind { Idle, Run, Shoot };

struct GalleryItem {
  const char* id;     // Stable slug; also the golden-file prefix.
  const char* label;  // Human-readable name for the page.
  ItemKind kind;
  tactics::WeaponType weapon;
  AnimKind anim = AnimKind::Idle;  // Meaningful for Animation items only.
  // Loop length in seconds: one turntable revolution for WeaponModel items,
  // one animation cycle for Animation items.
  float duration = 1.0f;
};

// 3 weapon turntables followed by idle/run/shoot per weapon.
const std::vector<GalleryItem>& Catalog();

// Simple deterministic orbit framing: spherical offset from a look-at
// target. Mutated by the web viewer's mouse input; the golden runner uses
// DefaultView untouched.
struct ViewState {
  float yawRadians = 0.0f;    // Angle of the camera around +Y, from +X.
  float pitchRadians = 0.0f;  // Elevation above the horizon.
  float distance = 3.0f;
  glm::vec3 target{0.0f};

  glm::vec3 Position() const;
  glm::mat4 ViewMatrix() const;
  glm::mat4 ProjectionMatrix(float aspectRatio) const;
};

ViewState DefaultView(int itemIndex);

// The figure state the gallery poses for an Animation item at loop time `t`
// (exposed so tools/tests can reuse the exact mapping).
tactics::Unit AnimationUnit(const GalleryItem& item, double t);

// Owns the GL resources (one unshadowed lit shader + the shared cube and
// sphere meshes) and renders one catalog item at loop time `t` into the
// bottom-left `width` x `height` rect of the current framebuffer.
class GalleryRenderer {
 public:
  bool Init();
  void Destroy();
  void Render(int itemIndex, double t, const ViewState& view, int width, int height);

 private:
  gfx::Shader litShader_;
  gfx::CubeMesh cubeMesh_;
  gfx::SphereMesh sphereMesh_;
};

}  // namespace gallery
