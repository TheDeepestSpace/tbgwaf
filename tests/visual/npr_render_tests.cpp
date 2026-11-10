// Exercise the grayscale NPR pass (cel shading + AO + outlines) without
// screenshot goldens. Set NPR_DUMP_DIR to also write PPM before/after images.
#include <SDL.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
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
  Require(glGetError() == GL_NO_ERROR, "GL error during NPR render/readback");
  return pixels;
}

void Dump(const char* name, const std::vector<unsigned char>& px, int w, int h) {
  const char* dir = std::getenv("NPR_DUMP_DIR");
  if (!dir) return;
  std::string path = std::string(dir) + "/" + name + ".ppm";
  FILE* f = std::fopen(path.c_str(), "wb");
  if (!f) return;
  std::fprintf(f, "P6\n%d %d\n255\n", w, h);
  for (int row = h - 1; row >= 0; --row)
    for (int col = 0; col < w; ++col) std::fwrite(&px[(row * w + col) * 4], 1, 3, f);
  std::fclose(f);
}

long TotalRed(const std::vector<unsigned char>& px) {
  long sum = 0;
  for (size_t i = 0; i < px.size(); i += 4) sum += px[i];
  return sum;
}

void CheckRenderer() {
  constexpr int width = 320, height = 192, x = 13, y = 11;
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
  tactics::Unit enemy;
  enemy.id = 7;
  enemy.team = tactics::Team::Red;
  enemy.position = glm::vec3(5.0f, 0.0f, 0.0f);
  scene.units.push_back(enemy);
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
  auto render = [&]() {
    renderer.RenderPane(game, tactics::Team::Blue, false, {}, camera, x, y, width, height, {},
                        fbo, options);
    return Capture(x, y, width, height);
  };
  const auto normal = render();
  options.npr = true;
  const auto npr = render();
  Dump("before", normal, width, height);
  Dump("after", npr, width, height);
  Require(normal != npr, "NPR toggle did not change the image");
  Require(render() == npr, "NPR render is not deterministic");
  Require(Capture(0, 0, 1, 1) == std::vector<unsigned char>({255, 0, 255, 255}),
          "NPR wrote outside its viewport");
  int red = 0, tinted = 0;
  for (size_t i = 0; i < npr.size(); i += 4) {
    const int r = npr[i], g = npr[i + 1], b = npr[i + 2];
    if (r == g && r == b) continue;
    if (r > g + 40 && r > b + 40) ++red; else ++tinted;
  }
  Require(red > 10, "Figure lost its saturated team color under NPR");
  Require(tinted < red, "NPR environment is not grayscale");

  options.nprSettings.outlineStrength = 0.0f;
  const auto noLines = render();
  Require(TotalRed(npr) < TotalRed(noLines) - 500, "Outlines did not darken silhouettes");
  options.nprSettings.outlineStrength = 0.85f;
  options.nprSettings.outlineThickness = 0.0f;
  Require(render() == noLines, "Zero thickness should disable outlines");
  options.nprSettings.outlineThickness = 3.0f;
  Require(render() != npr, "Outline thickness had no effect");
  options.nprSettings.outlineThickness = 1.5f;
  options.nprSettings.celBands = 2;
  Require(render() != npr, "Cel band count had no effect");
  options.nprSettings.celBands = 4;
  Require(render() == npr, "NPR settings leaked state");

  // At the fitted overview distance the LOD has already faded the fine
  // normal-driven edges; turning it off (detail size 0) outlines every
  // crease again and darkens the image.
  options.nprSettings.detailSize = 0.0f;
  const auto noLod = render();
  Require(noLod != npr, "Outline distance LOD had no effect from afar");
  Require(TotalRed(noLod) < TotalRed(npr), "Disabling the outline LOD should add edges");
  options.nprSettings.detailSize = gfx::NprSettings{}.detailSize;
  Require(render() == npr, "Outline LOD setting leaked state");
  options.npr = false;
  Require(render() == normal, "NPR state leaked into normal rendering");

  // See-through buildings keep their (fainter) outlines: a blue unit hidden
  // behind the building makes it fade for the camera.
  {
    const glm::vec3 camPos = camera.Position();
    glm::vec3 away = glm::normalize(glm::vec3(-camPos.x, 0.0f, -camPos.z));
    tactics::Scene seeThrough = scene;
    tactics::Unit friendly;
    friendly.id = 3;
    friendly.team = tactics::Team::Blue;
    friendly.position = away * 3.0f;
    seeThrough.units.push_back(friendly);
    tactics::GameLogic fadeGame(seeThrough);
    auto renderFaded = [&]() {
      renderer.RenderPane(fadeGame, tactics::Team::Blue, false, {}, camera, x, y, width, height,
                          {}, fbo, options);
      return Capture(x, y, width, height);
    };
    options.npr = true;
    const auto fadedNpr = renderFaded();
    Dump("faded", fadedNpr, width, height);
    options.nprSettings.outlineStrength = 0.0f;
    const auto fadedNoLines = renderFaded();
    options.nprSettings.outlineStrength = 0.85f;
    Require(TotalRed(fadedNpr) < TotalRed(fadedNoLines) - 200,
            "See-through building lost its outlines under NPR");
    Require(renderFaded() == fadedNpr, "Faded NPR render is not deterministic");
    options.disableOcclusionFade = true;
    Require(renderFaded() != fadedNpr, "Building did not fade in the see-through test");
    options.disableOcclusionFade = false;
    options.npr = false;
  }

  // Enterable buildings fade to a plain translucent solid while they merely
  // block the view; a figure inside brings the interior back.
  {
    const glm::vec3 camPos = camera.Position();
    const glm::vec3 away = glm::normalize(glm::vec3(-camPos.x, 0.0f, -camPos.z));
    const auto hollowScene = [&](bool enterable, bool figureInside) {
      tactics::Scene s;
      s.mapHalfExtent = 10.0f;
      tactics::Building record;
      record.enterable = enterable;
      record.floorCount = 1;
      record.groundFloorHeight = 4.0f;
      record.floors = {{0.0f, 4.0f}};
      record.obstacle = 0;
      s.buildings.push_back(record);
      const auto add = [&](glm::vec3 lo, glm::vec3 hi) {
        tactics::Obstacle o;
        o.bounds = {lo, hi};
        o.footprint = {{lo.x, lo.z}, {hi.x, lo.z}, {hi.x, hi.z}, {lo.x, hi.z}};
        o.building = 0;
        s.obstacles.push_back(o);
      };
      if (enterable) {
        add(glm::vec3(-2, 3.7f, -2), glm::vec3(2, 4, 2));  // Roof slab (index 0).
        // Four walls, each with a doorway-sized gap (and lintel) mid-way.
        add(glm::vec3(-2, 0, -2), glm::vec3(-0.5f, 3.7f, -1.8f));
        add(glm::vec3(0.5f, 0, -2), glm::vec3(2, 3.7f, -1.8f));
        add(glm::vec3(-0.5f, 2.2f, -2), glm::vec3(0.5f, 3.7f, -1.8f));
        add(glm::vec3(-2, 0, 1.8f), glm::vec3(-0.5f, 3.7f, 2));
        add(glm::vec3(0.5f, 0, 1.8f), glm::vec3(2, 3.7f, 2));
        add(glm::vec3(-0.5f, 2.2f, 1.8f), glm::vec3(0.5f, 3.7f, 2));
        add(glm::vec3(-2, 0, -1.8f), glm::vec3(-1.8f, 3.7f, -0.5f));
        add(glm::vec3(-2, 0, 0.5f), glm::vec3(-1.8f, 3.7f, 1.8f));
        add(glm::vec3(-1.8f, 2.2f, -0.5f), glm::vec3(-2, 3.7f, 0.5f));
        add(glm::vec3(1.8f, 0, -1.8f), glm::vec3(2, 3.7f, -0.5f));
        add(glm::vec3(1.8f, 0, 0.5f), glm::vec3(2, 3.7f, 1.8f));
        add(glm::vec3(1.8f, 2.2f, -0.5f), glm::vec3(2, 3.7f, 0.5f));
      } else {
        add(glm::vec3(-2, 0, -2), glm::vec3(2, 4, 2));
      }
      tactics::Unit friendly;
      friendly.id = 3;
      friendly.team = tactics::Team::Blue;
      friendly.position = figureInside ? glm::vec3(0.0f) : away * 3.0f;
      s.units.push_back(friendly);
      return s;
    };
    options.disableShadows = true;
    options.npr = false;
    const auto renderScene = [&](const tactics::Scene& s) {
      tactics::GameLogic g(s);
      renderer.RenderPane(g, tactics::Team::Blue, false, {}, camera, x, y, width, height, {}, fbo,
                          options);
      return Capture(x, y, width, height);
    };
    // A solid block and a hollow building seen from outside fade identically:
    // no interior, no window/door cuts, just the translucent footprint prism.
    const auto shell = renderScene(hollowScene(true, false));
    const auto solid = renderScene(hollowScene(false, false));
    Dump("shell", shell, width, height);
    Require(shell == solid, "Unengaged enterable building did not fade to a plain shell");
    options.disableOcclusionFade = true;
    Require(renderScene(hollowScene(true, false)) != shell, "Shell building did not fade");
    options.disableOcclusionFade = false;
    // The figure inside is under a hollow building's roof: not hidden from the
    // camera by the walls alone? It is, from above -- the roof blocks it, so
    // the building fades and, being occupied, keeps its interior.
    const auto interior = renderScene(hollowScene(true, true));
    Dump("interior", interior, width, height);
    Require(interior != renderScene(hollowScene(false, true)),
            "Occupied enterable building lost its interior view");
  }

  // NPR on top of the AO (thermal) view: outlines only, base view unchanged
  // when they are off.
  options.thermal = true;
  const auto aoOnly = render();
  options.npr = true;
  const auto aoNpr = render();
  Require(aoNpr != aoOnly, "NPR did nothing on top of the AO view");
  Require(TotalRed(aoNpr) < TotalRed(aoOnly), "AO-view outlines did not darken silhouettes");
  options.nprSettings.outlineStrength = 0.0f;
  Require(render() == aoOnly, "NPR without outlines changed the AO view");
  options.nprSettings.outlineStrength = 0.85f;
  options.thermal = false;
  options.npr = false;

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
  SDL_Window* window = SDL_CreateWindow("NPR renderer tests", 0, 0, 320, 200,
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
    std::puts("PASS NPR renderer: grayscale cel look, outlines, team colors, mode switching");
  } catch (const std::exception& e) {
    std::fprintf(stderr, "FAIL NPR renderer: %s\n", e.what());
    result = 1;
  }
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return result;
}
