#pragma once

#include <vector>

#include <glm/glm.hpp>

#include "game/Types.h"
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
  WeaponModel,    // The weapon alone on an auto-rotating turntable.
  Animation,      // A full figure looping one animation of its weapon class.
  StreetFixture,  // One street fixture model (vehicle, bench, ...) on a turntable.
};

// ZipDown / ZipUp: a whole zipline ride (mount, hang, dismount) going
// downhill (roof to street) or uphill (street to roof).
// EmptyMag: the whole magazine fired at the weapon's own pace (issue #140),
// every bullet leaving its own scattered tracer line.
enum class AnimKind { Idle, Run, Shoot, ZipDown, ZipUp, EmptyMag };

struct GalleryItem {
  const char* id;     // Stable slug; also the golden-file prefix.
  const char* label;  // Human-readable name for the page.
  ItemKind kind;
  tactics::WeaponType weapon;
  AnimKind anim = AnimKind::Idle;  // Meaningful for Animation items only.
  // Loop length in seconds: one turntable revolution for WeaponModel and
  // StreetFixture items, one animation cycle for Animation items.
  float duration = 1.0f;
  // StreetFixture items: the model, shown at its in-game gameplay box
  // (tactics::StreetFixtureExtent) with its feet on the stage floor.
  tactics::StreetFixtureModel fixture = tactics::StreetFixtureModel::None;
};

// 3 weapon turntables, idle/run/shoot/zip-down/zip-up/empty-mag per weapon,
// then every street fixture model (vehicles first).
const std::vector<GalleryItem>& Catalog();

// The obstacle a StreetFixture item renders: its gameplay box centered on
// the origin, feet at Y=0, heading +X (shared with the golden close-ups).
tactics::Obstacle FixtureObstacle(const GalleryItem& item);

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
  // The selected street fixture's triangles, rebuilt when the item changes.
  gfx::LitTriangleMesh fixtureMesh_;
  int fixtureMeshItem_ = -1;
};

}  // namespace gallery
