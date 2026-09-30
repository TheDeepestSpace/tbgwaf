#include <SDL.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl2.h>

#ifdef __EMSCRIPTEN__
#include <emscripten.h>
#include <functional>
#endif

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <vector>

#include "game/GameLogic.h"
#include "game/Raycast.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"

using tactics::GameLogic;
using tactics::InputMode;
using tactics::Obstacle;
using tactics::Team;
using tactics::TeamVisibility;
using tactics::Unit;

namespace {

constexpr int kInitialWindowWidth = 1280;
constexpr int kInitialWindowHeight = 720;

// Stage-D split-screen: one shared window/canvas is divided into two
// side-by-side viewports, left = Blue, right = Red. Both read the same
// single GameLogic instance; only the camera and the fog-of-war-filtered
// draw/pick lists differ per pane.
constexpr int kPaneCount = 2;

// Finds the alive unit whose bounding box the ray hits nearest, or -1.
int PickUnit(const gfx::Ray& ray, const std::vector<Unit>& units) {
  int bestId = -1;
  float bestT = std::numeric_limits<float>::infinity();
  for (const auto& unit : units) {
    if (!unit.alive) continue;
    float t = 0.0f;
    if (tactics::RayIntersectsAABB(ray.origin, ray.direction, unit.Bounds(), &t)) {
      if (t < bestT) {
        bestT = t;
        bestId = unit.id;
      }
    }
  }
  return bestId;
}

const char* TeamName(Team team) { return team == Team::Blue ? "Blue" : "Red"; }

// Stage-C climbing: a ground/move click can land either on the y=0 ground
// plane or on top of a climbable obstacle (a crate's top face). Both are
// finite planes from the ray's point of view, so pick whichever the ray
// actually hits nearer the camera -- matches how the scene is rendered
// (the ground plane is infinite but obstacles occlude it visually).
bool IntersectGroundOrClimbTop(const gfx::Ray& ray, const std::vector<Obstacle>& obstacles,
                                glm::vec3* outPoint) {
  bool found = false;
  float bestT = std::numeric_limits<float>::infinity();
  glm::vec3 bestPoint(0.0f);

  glm::vec3 groundPoint;
  if (gfx::OrbitCamera::IntersectGroundPlane(ray, &groundPoint)) {
    const float t = glm::dot(groundPoint - ray.origin, ray.direction);
    found = true;
    bestT = t;
    bestPoint = groundPoint;
  }

  constexpr float kTopFaceEpsilon = 1e-2f;
  for (const auto& obstacle : obstacles) {
    if (!obstacle.climbable) continue;
    float t = 0.0f;
    if (!tactics::RayIntersectsAABB(ray.origin, ray.direction, obstacle.bounds, &t)) continue;
    const glm::vec3 hit = ray.origin + ray.direction * t;
    if (hit.y < obstacle.bounds.max.y - kTopFaceEpsilon) continue;  // Hit a side, not the top.
    if (t < bestT) {
      found = true;
      bestT = t;
      bestPoint = hit;
    }
  }

  if (found && outPoint) *outPoint = bestPoint;
  return found;
}

// Pane 0 is the left half of the window (Blue), pane 1 is the right half
// (Red). Arbitrary but fixed for the lifetime of the app.
Team PaneTeam(int pane) { return pane == 0 ? Team::Blue : Team::Red; }

struct PaneRect {
  int x = 0;
  int width = 0;
};

PaneRect ComputePaneRect(int pane, int windowWidth) {
  const int leftWidth = windowWidth / 2;
  if (pane == 0) return PaneRect{0, leftWidth};
  return PaneRect{leftWidth, windowWidth - leftWidth};
}

int PaneForX(int x, int windowWidth) { return x < windowWidth / 2 ? 0 : 1; }

}  // namespace

int main() {
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
      "tbgwaf", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, kInitialWindowWidth,
      kInitialWindowHeight, SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
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

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  // Our loop drains all pending SDL events before each NewFrame(), so we
  // want queued input applied immediately rather than spread across
  // multiple frames (the default trickling is meant for low-framerate apps
  // that call NewFrame() between individual OS events).
  ImGui::GetIO().ConfigInputTrickleEventQueue = false;
  ImGui_ImplSDL2_InitForOpenGL(window, glContext);
  ImGui_ImplOpenGL3_Init("#version 300 es");

  glEnable(GL_DEPTH_TEST);

  // All scene-pass GL resources (shaders, meshes, shadow map, light) live
  // in the shared SceneRenderer, so the headless visual scenario runner
  // (issue #14) draws through exactly the same code path as the app.
  gfx::SceneRenderer renderer;
  if (!renderer.Init()) return 1;

  // Stage-D: one independent orbit camera per pane/team (index 0 = Blue,
  // 1 = Red), so each side can freely rotate/zoom its own view without
  // affecting the other's.
  std::array<gfx::OrbitCamera, kPaneCount> cameras;
  // Each pane only gets half the window's horizontal space, so the default
  // zoom (tuned for a single full-width view) would clip the far edge of
  // the map; start pulled back further so both spawns fit by default.
  for (auto& camera : cameras) camera.Zoom(10.0f);
  GameLogic game;

  bool quit = false;
  int rightDragPane = -1;  // -1 = not dragging; else the pane a right-drag started in.
  glm::vec3 hoveredGroundPoint(0.0f);
  bool hasHoveredGroundPoint = false;

  // Headless CI (xvfb) has no real user input; bound the run so the smoke
  // test still exits on its own after exercising a few frames.
  const bool isSmokeTest = std::getenv("TBGWAF_SMOKE_TEST") != nullptr;
  int frameCount = 0;
  constexpr int kSmokeTestMaxFrames = 60;
  Uint32 lastFrameTicks = SDL_GetTicks();

  auto runFrame = [&]() {
    int windowWidth = kInitialWindowWidth, windowHeight = kInitialWindowHeight;
    SDL_GetWindowSize(window, &windowWidth, &windowHeight);

    std::array<PaneRect, kPaneCount> paneRects;
    for (int pane = 0; pane < kPaneCount; ++pane) paneRects[pane] = ComputePaneRect(pane, windowWidth);

    // Mouse-driven game input (world picking) must not be dispatched until
    // io.WantCaptureMouse reflects the UI actually built *this* frame:
    // checking it during event polling (before ImGui::NewFrame()/the UI
    // pass) reads a stale value from the previous frame, letting clicks
    // meant for an ImGui button leak into world-space picking (or vice
    // versa). So we just record raw input here and dispatch it below, after
    // the UI for this frame has been built.
    bool leftClickPending = false;
    int leftClickX = 0, leftClickY = 0;
    bool escapePending = false;
    int mouseX = 0, mouseY = 0;
    SDL_GetMouseState(&mouseX, &mouseY);

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      ImGui_ImplSDL2_ProcessEvent(&event);

      if (event.type == SDL_QUIT) {
        quit = true;
      } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_RIGHT) {
        rightDragPane = PaneForX(event.button.x, windowWidth);
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_RIGHT) {
        rightDragPane = -1;
      } else if (event.type == SDL_MOUSEMOTION) {
        mouseX = event.motion.x;
        mouseY = event.motion.y;
        if (rightDragPane >= 0 && !ImGui::GetIO().WantCaptureMouse) {
          constexpr float kRotateSpeed = 0.005f;
          cameras[rightDragPane].Rotate(-event.motion.xrel * kRotateSpeed,
                                         event.motion.yrel * kRotateSpeed);
        }
      } else if (event.type == SDL_MOUSEWHEEL) {
        if (!ImGui::GetIO().WantCaptureMouse) {
          constexpr float kZoomSpeed = 1.5f;
          cameras[PaneForX(mouseX, windowWidth)].Zoom(-event.wheel.y * kZoomSpeed);
        }
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
        leftClickPending = true;
        leftClickX = event.button.x;
        leftClickY = event.button.y;
      } else if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_ESCAPE) {
        escapePending = true;
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    const Uint32 nowTicks = SDL_GetTicks();
    const float dt = static_cast<float>(nowTicks - lastFrameTicks) / 1000.0f;
    lastFrameTicks = nowTicks;
    game.Update(dt);

    // Stage-D: whichever team currently has the turn is the "active" pane --
    // only that side's viewport accepts game-action input (unit selection,
    // move/shoot targeting). The other pane keeps rendering its own live
    // fog-of-war view (so both players can always watch the match) but is
    // dimmed and ignores clicks, matching local split-screen console games
    // where only the active player's half responds during their turn.
    // Camera orbit/zoom is *not* gated this way -- either player can look
    // around their own pane at any time.
    std::optional<Team> activeTeam;
    if (game.Mode() != InputMode::GameOver) {
      activeTeam = game.CurrentTeam();
    }
    const bool fogActive = game.Mode() != InputMode::GameOver;
    auto isPaneActive = [&](int pane) { return activeTeam && *activeTeam == PaneTeam(pane); };

    std::array<TeamVisibility, kPaneCount> paneVisibility;
    for (int pane = 0; pane < kPaneCount; ++pane) {
      if (fogActive) paneVisibility[pane] = game.ComputeVisibility(PaneTeam(pane));
    }

    // --- UI ---
    if (const auto winner = game.Winner()) {
      ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, windowHeight * 0.3f), ImGuiCond_Always,
                               ImVec2(0.5f, 0.5f));
      ImGui::Begin("Game Over", nullptr,
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::Text("%s team wins!", TeamName(*winner));
      if (ImGui::Button("New Match")) {
        game.Reset();
      }
      ImGui::End();
    } else {
      int plannedCount = 0, totalCount = 0;
      for (const Unit& unit : game.GetScene().units) {
        if (!unit.alive || !activeTeam || unit.team != *activeTeam) continue;
        ++totalCount;
        if (unit.plan.type != tactics::PlannedActionType::None) ++plannedCount;
      }

      ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
      ImGui::Begin("Turn", nullptr,
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                       ImGuiWindowFlags_NoMove);
      ImGui::Text("Round %d", game.RoundNumber());
      if (activeTeam) {
        ImGui::Text("%s team's turn -- plan every figure, then commit.", TeamName(*activeTeam));
        ImGui::Text("Planned: %d / %d", plannedCount, totalCount);
      }
      switch (game.Mode()) {
        case InputMode::AwaitingSelection:
          ImGui::TextWrapped("Click one of your figures to plan its action.");
          break;
        case InputMode::ActionMenu:
          ImGui::TextWrapped("Choose an action to plan.");
          break;
        case InputMode::AwaitingMoveDestination:
          ImGui::TextWrapped("Click a destination on the ground (Esc to cancel).");
          break;
        case InputMode::AwaitingShootTarget:
          ImGui::TextWrapped("Click an enemy figure to plan a shot (Esc to cancel).");
          break;
        case InputMode::Moving:
          ImGui::TextWrapped("Committing turn: figures are moving...");
          break;
        default:
          break;
      }
      ImGui::BeginDisabled(!game.CanCommitTurn());
      if (ImGui::Button("Commit Turn")) game.CommitTurn();
      ImGui::EndDisabled();
      ImGui::End();

      if (const auto selectedId = game.SelectedUnitId(); selectedId && activeTeam) {
        const Unit* selected = game.FindUnit(*selectedId);
        if (selected && (game.Mode() == InputMode::ActionMenu ||
                          game.Mode() == InputMode::AwaitingMoveDestination ||
                          game.Mode() == InputMode::AwaitingShootTarget)) {
          const int activePane = *activeTeam == Team::Blue ? 0 : 1;
          const PaneRect& activeRect = paneRects[activePane];
          const glm::mat4 activeView = cameras[activePane].ViewMatrix();
          const glm::mat4 activeProj = cameras[activePane].ProjectionMatrix(
              static_cast<float>(activeRect.width) / static_cast<float>(windowHeight));
          const glm::vec4 activeViewport(static_cast<float>(activeRect.x), 0.0f,
                                          static_cast<float>(activeRect.width),
                                          static_cast<float>(windowHeight));
          const glm::vec3 headTop = selected->position + glm::vec3(0.0f, 1.9f, 0.0f);
          const glm::vec3 screenPos =
              glm::project(headTop, activeView, activeProj, activeViewport);
          // glm::project assumes a bottom-left viewport origin; flip Y for
          // ImGui's top-left screen space. X is already absolute window
          // space since activeViewport.x carries the pane's own offset.
          const ImVec2 windowPos(screenPos.x, windowHeight - screenPos.y);
          ImGui::SetNextWindowPos(windowPos, ImGuiCond_Always, ImVec2(0.5f, 1.0f));
          ImGui::Begin("Actions", nullptr,
                       ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                           ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoTitleBar);
          if (game.Mode() == InputMode::ActionMenu) {
            if (ImGui::Button("Move")) game.ChooseMove();
            ImGui::SameLine();
            if (ImGui::Button("Shoot")) game.ChooseShoot();
            ImGui::SameLine();
            if (ImGui::Button("Overwatch")) game.ChooseOverwatch();
            ImGui::SameLine();
            if (ImGui::Button("Pass")) game.ChoosePass();
          } else {
            if (ImGui::Button("Cancel")) game.CancelAction();
          }
          ImGui::End();
        }
      }
    }

    // Pane divider, per-pane team labels, and a dimming overlay on whichever
    // pane isn't currently allowed to act -- the split-screen equivalent of
    // "grey out the inactive side" for local multiplayer.
    ImDrawList* overlay = ImGui::GetForegroundDrawList();
    overlay->AddLine(ImVec2(static_cast<float>(paneRects[1].x), 0.0f),
                      ImVec2(static_cast<float>(paneRects[1].x), static_cast<float>(windowHeight)),
                      IM_COL32(255, 255, 255, 60), 2.0f);
    for (int pane = 0; pane < kPaneCount; ++pane) {
      const PaneRect& rect = paneRects[pane];
      const bool active = isPaneActive(pane);
      const char* status = game.Winner() ? "" : (active ? " - your turn" : "");
      char label[64];
      std::snprintf(label, sizeof(label), "%s%s", TeamName(PaneTeam(pane)), status);
      overlay->AddText(ImVec2(rect.x + 10.0f, windowHeight - 24.0f), IM_COL32(255, 255, 255, 220),
                        label);
      if (game.Mode() != InputMode::GameOver && !active) {
        overlay->AddRectFilled(ImVec2(static_cast<float>(rect.x), 0.0f),
                                ImVec2(static_cast<float>(rect.x + rect.width),
                                       static_cast<float>(windowHeight)),
                                IM_COL32(0, 0, 0, 110));
      }
    }

    // --- Dispatch deferred input, now that WantCaptureMouse reflects the UI
    // actually built this frame. ---
    if (escapePending) game.CancelAction();

    const bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse;
    if (!uiWantsMouse) {
      const int hoverPane = PaneForX(mouseX, windowWidth);
      if (isPaneActive(hoverPane) && game.Mode() == InputMode::AwaitingMoveDestination) {
        const PaneRect& rect = paneRects[hoverPane];
        const gfx::Ray hoverRay = cameras[hoverPane].ScreenPointToRay(
            static_cast<float>(mouseX - rect.x), static_cast<float>(mouseY),
            static_cast<float>(rect.width), static_cast<float>(windowHeight));
        glm::vec3 hoverPoint;
        if (IntersectGroundOrClimbTop(hoverRay, game.GetScene().obstacles, &hoverPoint)) {
          hoveredGroundPoint = hoverPoint;
          hasHoveredGroundPoint = true;
          game.HoverGround(hoverPoint);
        }
      }
      if (leftClickPending) {
        const int clickPane = PaneForX(leftClickX, windowWidth);
        if (isPaneActive(clickPane)) {
          const PaneRect& rect = paneRects[clickPane];
          const gfx::Ray clickRay = cameras[clickPane].ScreenPointToRay(
              static_cast<float>(leftClickX - rect.x), static_cast<float>(leftClickY),
              static_cast<float>(rect.width), static_cast<float>(windowHeight));
          // Only figures actually rendered on this pane this frame (own
          // team, or enemies currently inside this team's FOV) are pickable
          // -- a hidden enemy's collision box must not be clickable just
          // because it happens to sit behind something that is drawn.
          const Team clickTeam = PaneTeam(clickPane);
          std::vector<Unit> pickableUnits;
          for (const Unit& unit : game.GetScene().units) {
            if (unit.alive && gfx::IsUnitVisibleForRender(unit, clickTeam, fogActive,
                                                            paneVisibility[clickPane])) {
              pickableUnits.push_back(unit);
            }
          }
          const int hitUnit = PickUnit(clickRay, pickableUnits);
          if (hitUnit >= 0) {
            game.ClickUnit(hitUnit);
          } else if (game.Mode() == InputMode::AwaitingMoveDestination) {
            glm::vec3 point;
            if (IntersectGroundOrClimbTop(clickRay, game.GetScene().obstacles, &point)) {
              game.ClickGround(point);
            }
          }
        }
      }
    }

    // --- Render: one shadow pass + one color pass per pane, both inside
    // SceneRenderer::RenderPane. Selection/move-preview overlays belong to
    // whichever team is currently acting, so only their pane gets them. ---
    for (int pane = 0; pane < kPaneCount; ++pane) {
      const PaneRect& rect = paneRects[pane];
      gfx::PaneOverlays overlays;
      if (isPaneActive(pane)) {
        if (const auto selectedId = game.SelectedUnitId()) {
          if (const Unit* selected = game.FindUnit(*selectedId)) {
            overlays.selectionHighlight = selected->position;
          }
        }
        if (game.Mode() == InputMode::AwaitingMoveDestination) {
          if (game.MovePreviewValid()) {
            overlays.movePreviewPath = &game.MovePreviewPath();
          } else if (hasHoveredGroundPoint) {
            overlays.invalidHoverHighlight = hoveredGroundPoint;
          }
        }
      }
      renderer.RenderPane(game, PaneTeam(pane), fogActive, paneVisibility[pane], cameras[pane],
                          rect.x, 0, rect.width, windowHeight, overlays);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(window);

    if (isSmokeTest && ++frameCount >= kSmokeTestMaxFrames) {
      quit = true;
    }

#ifdef __EMSCRIPTEN__
    if (quit) emscripten_cancel_main_loop();
#endif
  };

#ifdef __EMSCRIPTEN__
  std::function<void()> frameFn = runFrame;
  emscripten_set_main_loop_arg(
      [](void* arg) { (*static_cast<std::function<void()>*>(arg))(); }, &frameFn, 0, 1);
#else
  while (!quit) runFrame();

  renderer.Destroy();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();
#endif
  return 0;
}
