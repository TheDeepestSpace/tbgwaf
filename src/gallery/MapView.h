#pragma once

#include <cstdint>
#include <memory>

#include "game/GameLogic.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"

namespace gallery {

// The gallery's map-seed viewer: regenerates an "Urban" (4x4 blocks) or
// "Urban x4" (default 8x8) map and draws it through the game's own
// SceneRenderer (so buildings, facades, street furniture and shadows match
// the game), without figures or fog, and in greyscale. Camera input mirrors
// the game: left-drag pans, right-drag rotates, wheel zooms.
class MapView {
 public:
  // Requires a current GL context.
  bool Init();
  void Destroy();

  void Open(uint32_t seed, bool x4);
  void Close() { game_.reset(); }
  bool IsOpen() const { return game_ != nullptr; }

  void Pan(float dxPixels, float dyPixels);
  void Rotate(float dxPixels, float dyPixels);
  void Zoom(float wheelTicks);

  void Render(float dt, int width, int height);

 private:
  gfx::SceneRenderer renderer_;
  gfx::OrbitCamera camera_;
  std::unique_ptr<tactics::GameLogic> game_;
};

}  // namespace gallery
