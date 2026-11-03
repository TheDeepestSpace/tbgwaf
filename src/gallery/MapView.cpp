#include "gallery/MapView.h"

#include <GLES3/gl3.h>

#include "game/MapGenerator.h"

namespace gallery {

bool MapView::Init() { return renderer_.Init(); }

void MapView::Destroy() {
  game_.reset();
  renderer_.Destroy();
}

void MapView::Open(uint32_t seed, bool x4) {
  tactics::MapGeneratorConfig config;
  if (!x4) {
    config.blocksX = 4;
    config.blocksZ = 4;
  }
  tactics::Scene scene = tactics::GenerateUrbanMap(seed, config);
  scene.units.clear();  // Map only: no figures.
  game_ = std::make_unique<tactics::GameLogic>(std::move(scene));
  camera_ = gfx::OrbitCamera();
  camera_.FitToExtent(game_->GetScene().mapHalfExtent);
}

// Same speeds as the game's main loop (src/main.cpp).
void MapView::Pan(float dx, float dy) { camera_.Pan(-dx * 0.0015f, dy * 0.0015f); }
void MapView::Rotate(float dx, float dy) { camera_.Rotate(-dx * 0.005f, dy * 0.005f); }
void MapView::Zoom(float wheelTicks) {
  camera_.Zoom(-wheelTicks * 0.625f * camera_.TargetDistance());
}

void MapView::Render(float dt, int width, int height) {
  if (!game_) return;
  camera_.Update(dt);
  gfx::RenderDebugOptions debug;
  debug.monochrome = true;
  renderer_.RenderPane(*game_, tactics::Team::Blue, /*fogActive=*/false, tactics::TeamVisibility{},
                       camera_, 0, 0, width, height, {}, 0, debug);
}

}  // namespace gallery
