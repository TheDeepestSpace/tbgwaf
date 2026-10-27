// Exercise thermal post-processing state independently of screenshot goldens:
// contact shading, fog privacy, offscreen targets, pane isolation, resize,
// polarity, and returning to normal.
#include <SDL.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <vector>

#include "game/GameLogic.h"
#include "gfx/SceneRenderer.h"

namespace {
void Require(bool ok, const char* message) {
  if (!ok) throw std::runtime_error(message);
}

std::vector<unsigned char> Capture(int x, int y, int width, int height) {
  std::vector<unsigned char> pixels(width * height * 4);
  glReadPixels(x, y, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
  Require(glGetError() == GL_NO_ERROR, "GL error during thermal render/readback");
  return pixels;
}

void CheckRenderer() {
  constexpr int width = 128, height = 96, x = 13, y = 11;
  constexpr int targetWidth = 300, targetHeight = 180;
  GLuint fbo = 0, color = 0, depth = 0;
  glGenFramebuffers(1, &fbo);
  glGenTextures(1, &color);
  glBindTexture(GL_TEXTURE_2D, color);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, targetWidth, targetHeight, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glGenRenderbuffers(1, &depth);
  glBindRenderbuffer(GL_RENDERBUFFER, depth);
  glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, targetWidth, targetHeight);
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color, 0);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, depth);
  Require(glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE,
          "Offscreen framebuffer incomplete");

  tactics::Scene scene;
  scene.mapHalfExtent = 10.0f;
  tactics::Obstacle building;
  building.bounds = {glm::vec3(-2.0f, 0.0f, -2.0f), glm::vec3(2.0f, 4.0f, 2.0f)};
  scene.obstacles.push_back(building);
  tactics::GameLogic game(scene);
  gfx::OrbitCamera camera;
  camera.FitToExtent(scene.mapHalfExtent);
  gfx::SceneRenderer renderer;
  Require(renderer.Init(), "Renderer initialization failed");
  glBindFramebuffer(GL_FRAMEBUFFER, fbo);
  glClearColor(1.0f, 0.0f, 1.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  gfx::RenderDebugOptions options;
  options.disableFov = true;
  auto render = [&](int paneX, int w, int h) {
    renderer.RenderPane(game, tactics::Team::Blue, false, {}, camera,
                         paneX, y, w, h, {}, fbo, options);
    return Capture(paneX, y, w, h);
  };
  const auto normal = render(x, width, height);
  options.thermal = true;
  const auto blackHot = render(x, width, height);
  Require(normal != blackHot, "Thermal toggle did not change the world image");
  options.thermalBlackHot = false;
  const auto whiteHot = render(x + width + 8, width, height);
  Require(Capture(x, y, width, height) == blackHot, "Second pane changed the first pane");
  Require(Capture(0, 0, 1, 1) == std::vector<unsigned char>({255, 0, 255, 255}),
          "Thermal filtering wrote outside its viewport");
  int paleRoofPixels = 0, midtonePixels = 0;
  for (size_t i = 0; i < blackHot.size(); i += 4) {
    Require(blackHot[i] == blackHot[i + 1] && blackHot[i] == blackHot[i + 2],
            "Thermal world has a color tint");
    // Sensor noise/8-bit rounding may differ by a few levels; the heat
    // image itself must invert, including the off-map background.
    Require(std::abs(int(blackHot[i]) + int(whiteHot[i]) - 255) <= 5,
            "White-hot did not invert the same heat image");
    if (blackHot[i] > 240) ++paleRoofPixels;
    if (blackHot[i] > 100 && blackHot[i] < 225) ++midtonePixels;
  }
  Require(paleRoofPixels > 20 && midtonePixels > 100,
          "Thermal scene lost its roof/surroundings contrast");
  options.thermalBlackHot = true;
  // The AO targets grow to the largest pane and are shared: a smaller pane
  // (the other half of an odd-width window) must render into their corner
  // exactly as it would into tight targets of its own.
  const auto smallShared = render(x, 79, 61);
  {
    gfx::SceneRenderer fresh;
    Require(fresh.Init(), "Second renderer initialization failed");
    fresh.RenderPane(game, tactics::Team::Blue, false, {}, camera, x, y, 79, 61, {}, fbo,
                     options);
    Require(Capture(x, y, 79, 61) == smallShared,
            "Smaller pane rendered differently in shared (larger) AO targets");
    fresh.Destroy();
  }
  Require(render(x, width, height) == blackHot, "Thermal image changed after a viewport resize");
  options.thermal = false;
  Require(render(x, width, height) == normal, "Thermal state leaked into normal rendering");

  // Compare identical ground points with/without nearby geometry. Contact
  // shading must come from the building, not a painted material gradient;
  // it must fall off while exposed roof centers stay pale.
  options.thermal = true;
  tactics::Scene emptyScene = scene;
  emptyScene.obstacles.clear();
  tactics::GameLogic emptyGame(emptyScene);
  renderer.RenderPane(emptyGame, tactics::Team::Blue, false, {}, camera,
                       x, y, width, height, {}, fbo, options);
  const auto empty = Capture(x, y, width, height);
  glm::mat4 vp = camera.ProjectionMatrix(float(width) / height) * camera.ViewMatrix();
  auto brightness = [&](const auto& pixels, const glm::vec3& point) {
    const glm::vec4 clip = vp * glm::vec4(point, 1.0f);
    const int px = int((clip.x / clip.w * 0.5f + 0.5f) * width);
    const int py = int((clip.y / clip.w * 0.5f + 0.5f) * height);
    Require(px > 0 && px < width - 1 && py > 0 && py < height - 1,
            "AO probe outside pane");
    int sum = 0;
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) sum += pixels[((py + dy) * width + px + dx) * 4];
    }
    return float(sum) / 9.0f;
  };
  const glm::vec3 contact(2.5f, 0.0f, 0.0f), openGround(6.0f, 0.0f, 0.0f);
  Require(brightness(empty, contact) - brightness(blackHot, contact) > 1.5f,
          "Building did not cast soft ambient contact shading");
  Require(std::abs(brightness(empty, openGround) - brightness(blackHot, openGround)) < 4.0f,
          "Ambient occlusion spread beyond nearby geometry");
  Require(brightness(blackHot, glm::vec3(0.0f, 4.0f, 0.0f)) > 240.0f,
          "Exposed roof was incorrectly occluded");

  tactics::Scene floatingScene = scene;
  floatingScene.obstacles[0].bounds.min.y += 6.0f;
  floatingScene.obstacles[0].bounds.max.y += 6.0f;
  tactics::GameLogic floatingGame(floatingScene);
  renderer.RenderPane(floatingGame, tactics::Team::Blue, false, {}, camera,
                       x, y, width, height, {}, fbo, options);
  Require(std::abs(brightness(Capture(x, y, width, height), contact) -
                   brightness(empty, contact)) < 4.0f,
          "Distant geometry cast a false screen-space contact shadow");

  // Hidden enemies must contribute neither silhouettes nor contact shading.
  tactics::Scene hiddenScene = scene;
  tactics::Unit enemy;
  enemy.id = 7;
  enemy.team = tactics::Team::Red;
  enemy.position = glm::vec3(5.0f, 0.0f, 0.0f);
  hiddenScene.units.push_back(enemy);
  tactics::GameLogic hiddenGame(hiddenScene);
  renderer.RenderPane(hiddenGame, tactics::Team::Blue, true, {}, camera,
                       x, y, width, height, {}, fbo, options);
  Require(Capture(x, y, width, height) == blackHot, "AO exposed a fog-hidden enemy");

  // "Baked thermal" looks shading up from a world-space bake: it must still
  // darken the building's base like the live pass (loosely), leave open
  // ground and roofs alone, and not depend on the camera.
  options.thermalAo.baked = true;
  // The coarse test probe grid needs a wider reach than the shipped 0.8
  // radius to exercise interpolation between 1.5-unit-spaced probes.
  options.thermalAo.radius = 2.4f;
  options.thermalAo.strength = 1.9f;
  options.thermalAo.bake.directions = 16;
  options.thermalAo.bake.depthResolution = 256;
  options.thermalAo.bake.spacing = 1.5f;
  renderer.RenderPane(game, tactics::Team::Blue, false, {}, camera, x, y, width, height, {}, fbo,
                      options);
  const auto baked = Capture(x, y, width, height);
  Require(brightness(empty, contact) - brightness(baked, contact) > 12.0f,
          "Baked thermal did not shade the building's base");
  Require(std::abs(brightness(empty, openGround) - brightness(baked, openGround)) < 4.0f,
          "Baked thermal spread shading across open ground");
  Require(brightness(baked, glm::vec3(0.0f, 4.0f, 0.0f)) > 240.0f,
          "Baked thermal occluded an exposed roof");
  Require(std::abs(brightness(blackHot, contact) - brightness(baked, contact)) < 60.0f,
          "Baked thermal diverged from the live shading at the contact point");
  const unsigned int firstBake = renderer.ThermalBake().bakeCount;
  Require(firstBake == 1, "First baked frame did not produce exactly one volume");
  renderer.RenderPane(hiddenGame, tactics::Team::Blue, true, {}, camera,
                       x, y, width, height, {}, fbo, options);
  Require(Capture(x, y, width, height) == baked, "Baked contact exposed a fog-hidden enemy");
  Require(renderer.ThermalBake().bakeCount == firstBake, "Units invalidated the static bake");
  renderer.RenderPane(hiddenGame, tactics::Team::Blue, false, {}, camera,
                       x, y, width, height, {}, fbo, options);
  const auto visibleFigure = Capture(x, y, width, height);
  Require(brightness(baked, {5.7f, 0.0f, 0.0f}) -
              brightness(visibleFigure, {5.7f, 0.0f, 0.0f}) > 3.0f,
          "Visible figure did not cast live contact shading");
  Require(renderer.ThermalBake().bakeCount == firstBake, "Visible units caused a rebake");
  gfx::OrbitCamera moved = camera;
  moved.Rotate(0.15f, 0.0f);
  renderer.RenderPane(game, tactics::Team::Blue, false, {}, moved, x, y, width, height, {}, fbo,
                      options);
  const auto bakedMoved = Capture(x, y, width, height);
  Require(bakedMoved != baked, "Baked thermal ignored a camera move");
  Require(renderer.ThermalBake().bakeCount == firstBake, "Camera move caused a rebake");
  options.thermalAo.softness = 9;
  Require(render(x, width, height) == baked, "Softness changed the baked image");
  options.thermalAo.strength = 0.0f;
  const auto unshaded = render(x, width, height);
  Require(brightness(unshaded, contact) - brightness(baked, contact) > 12.0f,
          "Baked strength did not update live");
  Require(renderer.ThermalBake().bakeCount == firstBake, "Live settings caused a rebake");
  options.thermalAo.strength = 1.9f;
  options.thermalAo.radius = 3.0f;
  render(x, width, height);
  Require(renderer.ThermalBake().bakeCount == firstBake + 1, "Radius did not rebake");
  render(x, width, height);
  Require(renderer.ThermalBake().bakeCount == firstBake + 1, "Unchanged radius rebaked again");
  options.thermalAo.bake.directions = 0;
  options.thermalAo.quality = 2;  // 32 directions, two batches.
  render(x, width, height);
  Require(renderer.ThermalBake().bakeCount == firstBake + 2, "Quality did not rebake");
  render(x, width, height);
  Require(renderer.ThermalBake().bakeCount == firstBake + 2, "Unchanged quality rebaked again");
  options.thermalAo.radius = 2.4f;
  options.thermalAo.bake.directions = 16;
  options.thermalAo.bake.batchLayers = 8;
  const auto twoBatches = render(x, width, height);
  for (size_t i = 0; i < baked.size(); ++i) {
    Require(std::abs(int(baked[i]) - int(twoBatches[i])) <= 5,
            "Batch accumulation changed visibility beyond RGBA8 rounding");
  }

  // PR #178: the same vertical wall point viewed at eye level, first open
  // to the sky and then under a floating deck. The old heightmap flattened
  // the gap into a solid column and could not darken this receiver.
  tactics::Scene coveredScene = scene;
  tactics::WalkSurface slab;
  slab.vertices = {{1.0f, 3.25f, -4.0f}, {5.0f, 3.25f, -4.0f},
                   {5.0f, 3.25f, 4.0f}, {1.0f, 3.25f, 4.0f}};
  coveredScene.walkSurfaces.push_back(slab);
  tactics::GameLogic coveredGame(coveredScene);
  gfx::OrbitCamera eyeCamera;
  eyeCamera.Rotate(0.9f, -0.92f);
  eyeCamera.Zoom(-6.0f);
  eyeCamera.Update(1.0f);
  eyeCamera.target = {2.0f, 1.2f, 0.0f};
  vp = eyeCamera.ProjectionMatrix(float(width) / height) * eyeCamera.ViewMatrix();
  options.disableOcclusionFade = true;
  renderer.RenderPane(game, tactics::Team::Blue, false, {}, eyeCamera,
                       x, y, width, height, {}, fbo, options);
  const auto exposedWall = Capture(x, y, width, height);
  renderer.RenderPane(coveredGame, tactics::Team::Blue, false, {}, eyeCamera,
                       x, y, width, height, {}, fbo, options);
  const auto covered = Capture(x, y, width, height);
  const auto coveredBake = renderer.ThermalBake().bakeCount;
  const glm::vec3 wallPoint(2.0f, 1.5f, 0.0f), underDeck(4.0f, 0.0f, 0.0f);
  std::printf("Thermal overpass: wall %.1f -> %.1f, ground %.1f -> %.1f\n",
              brightness(exposedWall, wallPoint), brightness(covered, wallPoint),
              brightness(exposedWall, underDeck), brightness(covered, underDeck));
  Require(brightness(exposedWall, wallPoint) - brightness(covered, wallPoint) > 8.0f,
          "Floating slab did not darken the wall beneath it at eye level");
  Require(brightness(exposedWall, underDeck) - brightness(covered, underDeck) > 8.0f,
          "Floating slab did not darken the ground beneath it");
  options.thermalAo.strength = 0.0f;
  renderer.RenderPane(coveredGame, tactics::Team::Blue, false, {}, eyeCamera,
                       x, y, width, height, {}, fbo, options);
  const auto coveredUnshaded = Capture(x, y, width, height);
  const glm::vec3 underside(4.0f, 2.8f, 0.0f);
  Require(brightness(coveredUnshaded, underside) - brightness(covered, underside) > 8.0f,
          "Deck underside did not receive ambient shading from the lower hemisphere");
  Require(renderer.ThermalBake().bakeCount == coveredBake, "Strength rebuilt the overpass bake");

  // Mutating geometry in-place must invalidate by contents, including props
  // and facade/massing inputs omitted by the old heightmap hash.
  tactics::Scene detailedScene = coveredScene;
  tactics::MassingDetail detail;
  detail.footprint = {{-1.0f, -1.0f}, {1.0f, -1.0f}, {1.0f, 1.0f}, {-1.0f, 1.0f}};
  detail.y0 = 4.0f; detail.y1 = 4.2f;
  detailedScene.massing.push_back(detail);
  tactics::GameLogic detailedGame(detailedScene);
  renderer.RenderPane(detailedGame, tactics::Team::Blue, false, {}, eyeCamera,
                       x, y, width, height, {}, fbo, options);
  Capture(x, y, width, height);
  Require(renderer.ThermalBake().bakeCount == coveredBake + 1, "Massing edit did not rebake");
  options.thermalAo.baked = false;

  renderer.Destroy();
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  glDeleteRenderbuffers(1, &depth);
  glDeleteTextures(1, &color);
  glDeleteFramebuffers(1, &fbo);
}
}  // namespace

int main() {
  if (SDL_Init(SDL_INIT_VIDEO) != 0) return 1;
  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_Window* window = SDL_CreateWindow("Thermal renderer tests", 0, 0, 320, 200,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
  SDL_GLContext context = window ? SDL_GL_CreateContext(window) : nullptr;
  if (!context) {
    std::fprintf(stderr, "GL context failed: %s\n", SDL_GetError());
    if (window) SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  int result = 0;
  try {
    CheckRenderer();
    std::puts("PASS thermal renderer: contact shading, fog privacy, polarity, pane isolation, "
              "resize, mode switching");
  } catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL thermal renderer: %s\n", e.what());
    result = 1;
  }
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
