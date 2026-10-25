// Exercise thermal post-processing state independently of screenshot goldens:
// offscreen targets, pane isolation, resize, polarity, and returning to normal.
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
  render(x, 79, 61);
  Require(render(x, width, height) == blackHot, "Thermal image changed after a viewport resize");
  options.thermal = false;
  Require(render(x, width, height) == normal, "Thermal state leaked into normal rendering");

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
    std::puts("PASS thermal renderer: polarity, pane isolation, resize, and mode switching");
  } catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL thermal renderer: %s\n", e.what());
    result = 1;
  }
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
