// Asset & animation gallery viewer (issue #126): one canvas/window showing
// a single catalog item (weapon turntable, figure animation or street
// fixture turntable) at a time,
// with mouse orbit/zoom and loop playback. The item list, playback
// controls, and labels live in the host page (web/gallery.html), which
// drives this module through the small exported C API below; a native build
// (tactics_gallery_app) offers the same via keyboard for local poking:
// left/right switch item, space toggles playback.
//
// All actual content (catalog, framing, posing, rendering) comes from
// gallery::* in src/gallery/GalleryScene.*.

#include <SDL.h>
#include <GLES3/gl3.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>

#include <cstdint>
#include <string>

#include "gallery/GalleryScene.h"
#include "gallery/MapView.h"

namespace {

constexpr int kInitialWindowWidth = 960;
constexpr int kInitialWindowHeight = 720;

#ifdef __EMSCRIPTEN__
// Emscripten's SDL2 port reaches the canvas through the "#canvas" CSS
// selector; bind it to this instance's canvas (same pattern as main.cpp).
EM_JS(void, gallery_bind_canvas, (), {
  specialHTMLTargets["#canvas"] = Module.canvas;
});
EM_JS(double, gallery_canvas_css_width, (), {
  return Module.canvas.getBoundingClientRect().width;
});
EM_JS(double, gallery_canvas_css_height, (), {
  return Module.canvas.getBoundingClientRect().height;
});
#endif

// Viewer state, shared with the exported C API the host page calls.
struct ViewerState {
  int item = 0;
  bool playing = true;
  double time = 0.0;
  gallery::ViewState view;
};
ViewerState g_state;
gallery::MapView g_mapView;

void SelectItem(int index) {
  const int count = static_cast<int>(gallery::Catalog().size());
  g_state.item = ((index % count) + count) % count;
  g_state.time = 0.0;
  g_state.view = gallery::DefaultView(g_state.item);
}

}  // namespace

extern "C" {

#ifdef __EMSCRIPTEN__
#define GALLERY_EXPORT EMSCRIPTEN_KEEPALIVE
#else
#define GALLERY_EXPORT
#endif

// Map-seed viewer: while open, the canvas shows the generated map drawn by
// the game's renderer instead of the catalog item, and mouse input drives
// the game-style camera.
GALLERY_EXPORT void gallery_map_open(unsigned seed, int x4) {
  g_mapView.Open(static_cast<uint32_t>(seed), x4 != 0);
}
GALLERY_EXPORT void gallery_map_close() { g_mapView.Close(); }

GALLERY_EXPORT int gallery_item_count() {
  return static_cast<int>(gallery::Catalog().size());
}
GALLERY_EXPORT const char* gallery_item_id(int index) {
  return gallery::Catalog()[index].id;
}
GALLERY_EXPORT const char* gallery_item_label(int index) {
  return gallery::Catalog()[index].label;
}
// 0 = weapon model (turntable), 1 = animation, 2 = street fixture (turntable).
GALLERY_EXPORT int gallery_item_kind(int index) {
  switch (gallery::Catalog()[index].kind) {
    case gallery::ItemKind::WeaponModel: return 0;
    case gallery::ItemKind::Animation: return 1;
    case gallery::ItemKind::StreetFixture: return 2;
  }
  return 1;
}
GALLERY_EXPORT double gallery_item_duration(int index) {
  return gallery::Catalog()[index].duration;
}
GALLERY_EXPORT void gallery_select(int index) { SelectItem(index); }
GALLERY_EXPORT int gallery_selected() { return g_state.item; }
GALLERY_EXPORT void gallery_set_playing(int playing) { g_state.playing = playing != 0; }
GALLERY_EXPORT int gallery_playing() { return g_state.playing ? 1 : 0; }
GALLERY_EXPORT void gallery_set_time(double t) { g_state.time = t; }
GALLERY_EXPORT double gallery_time() { return g_state.time; }

}  // extern "C"

int main() {
#ifdef __EMSCRIPTEN__
  gallery_bind_canvas();
#endif
  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

  SDL_Window* window = SDL_CreateWindow(
      "tbgwaf asset gallery", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      kInitialWindowWidth, kInitialWindowHeight,
      SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
  if (!window) {
    std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_GLContext glContext = SDL_GL_CreateContext(window);
  if (!glContext) {
    std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  SDL_GL_SetSwapInterval(1);

  static gallery::GalleryRenderer renderer;
  if (!renderer.Init() || !g_mapView.Init()) return 1;
  SelectItem(0);

  static bool quit = false;
  static bool dragging = false;
  static bool rightDragging = false;
  static Uint32 lastTicks = SDL_GetTicks();
  static SDL_Window* win = window;

  auto runFrame = [&]() {
#ifdef __EMSCRIPTEN__
    // Track the canvas element's CSS size.
    {
      const double cssW = gallery_canvas_css_width();
      const double cssH = gallery_canvas_css_height();
      int curW = 0, curH = 0;
      SDL_GetWindowSize(win, &curW, &curH);
      if (cssW >= 1.0 && cssH >= 1.0 &&
          (curW != static_cast<int>(cssW) || curH != static_cast<int>(cssH))) {
        SDL_SetWindowSize(win, static_cast<int>(cssW), static_cast<int>(cssH));
      }
    }
#endif
    const Uint32 now = SDL_GetTicks();
    const double dt = (now - lastTicks) / 1000.0;
    lastTicks = now;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      switch (event.type) {
        case SDL_QUIT:
          quit = true;
          break;
        case SDL_MOUSEBUTTONDOWN:
          if (event.button.button == SDL_BUTTON_LEFT) dragging = true;
          if (event.button.button == SDL_BUTTON_RIGHT) rightDragging = true;
          break;
        case SDL_MOUSEBUTTONUP:
          if (event.button.button == SDL_BUTTON_LEFT) dragging = false;
          if (event.button.button == SDL_BUTTON_RIGHT) rightDragging = false;
          break;
        case SDL_MOUSEMOTION:
          if (g_mapView.IsOpen()) {
            if (dragging) g_mapView.Pan(event.motion.xrel, event.motion.yrel);
            if (rightDragging) g_mapView.Rotate(event.motion.xrel, event.motion.yrel);
          } else if (dragging) {
            g_state.view.yawRadians += event.motion.xrel * 0.01f;
            g_state.view.pitchRadians = std::clamp(
                g_state.view.pitchRadians + event.motion.yrel * 0.01f,
                -1.2f, 1.45f);
          }
          break;
        case SDL_MOUSEWHEEL:
          if (g_mapView.IsOpen()) {
#if SDL_VERSION_ATLEAST(2, 0, 18)
            g_mapView.Zoom(event.wheel.preciseY);
#else
            g_mapView.Zoom(static_cast<float>(event.wheel.y));
#endif
          } else {
            g_state.view.distance = std::clamp(
                g_state.view.distance * (event.wheel.y > 0 ? 0.9f : 1.1f), 0.3f, 20.0f);
          }
          break;
        case SDL_KEYDOWN:
          if (event.key.keysym.sym == SDLK_RIGHT) SelectItem(g_state.item + 1);
          if (event.key.keysym.sym == SDLK_LEFT) SelectItem(g_state.item - 1);
          if (event.key.keysym.sym == SDLK_SPACE) g_state.playing = !g_state.playing;
#ifndef __EMSCRIPTEN__
          if (event.key.keysym.sym == SDLK_ESCAPE) quit = true;
#endif
          break;
        default:
          break;
      }
    }

    const gallery::GalleryItem& item = gallery::Catalog()[g_state.item];
    if (g_state.playing) {
      g_state.time = std::fmod(g_state.time + dt, static_cast<double>(item.duration));
    }

    int width = 0, height = 0;
    SDL_GL_GetDrawableSize(win, &width, &height);
    if (g_mapView.IsOpen()) {
      g_mapView.Render(static_cast<float>(dt), width, height);
    } else {
      renderer.Render(g_state.item, g_state.time, g_state.view, width, height);
    }
    SDL_GL_SwapWindow(win);

#ifdef __EMSCRIPTEN__
    if (quit) emscripten_cancel_main_loop();
#endif
  };

#ifdef __EMSCRIPTEN__
  static std::function<void()> frameFn = runFrame;
  emscripten_set_main_loop_arg(
      [](void* arg) { (*static_cast<std::function<void()>*>(arg))(); }, &frameFn, 0, 1);
#else
  while (!quit) runFrame();

  g_mapView.Destroy();
  renderer.Destroy();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();
#endif
  return 0;
}
