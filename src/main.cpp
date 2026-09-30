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
#include <string>
#include <vector>

#include "game/GameLogic.h"
#include "game/Raycast.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"

using tactics::GameLogic;
using tactics::GameSnapshot;
using tactics::DeserializeSnapshot;
using tactics::InputMode;
using tactics::SerializeSnapshot;
using tactics::Obstacle;
using tactics::Team;
using tactics::TeamVisibility;
using tactics::Unit;

namespace {

constexpr int kInitialWindowWidth = 1280;
constexpr int kInitialWindowHeight = 720;

// Two-pane play: the page (web/index.html) shows both teams side by side by
// instantiating this module twice -- one client instance per team, each with
// its own <canvas>, camera, input, and UI. The instance whose team has the
// turn is authoritative: it runs input + simulation and posts a GameSnapshot
// on the page-level message bus after every change. The other instance is a
// follower: it renders the last received snapshot, ignores game input, and
// its pane is blurred by the page. A bare module load without the page's
// Module overrides (or a native build) has no bus and the single view simply
// follows whichever team is on the move (hot-seat).
constexpr int kTeamCount = 2;
constexpr Uint32 kPeerSyncTimeoutMs = 500;

#ifdef __EMSCRIPTEN__
// Emscripten's SDL2 port reaches the canvas through the "#canvas" CSS
// selector (canvas sizing, mouse-event registration), which cannot address
// per-instance canvases in a page that instantiates this module twice. Bind
// the selector to this instance's own canvas via the module-scoped
// specialHTMLTargets table instead; must run before SDL_Init.
EM_JS(void, tbgwaf_bind_canvas, (), {
  specialHTMLTargets["#canvas"] = Module.canvas;
});

// 0 = blue, 1 = red, -1 = not specified. The page assigns each client
// instance its team via the Module.tbgwafPlayer override.
EM_JS(int, tbgwaf_requested_player, (), {
  if (Module.tbgwafPlayer === "blue") return 0;
  if (Module.tbgwafPlayer === "red") return 1;
  return -1;
});

// The bus (Module.tbgwafBus, one object shared by both instances on the
// page) is just a list of per-instance inboxes: posting appends the message
// to every inbox but our own, and each instance drains its inbox at the top
// of its frame.
EM_JS(void, tbgwaf_channel_open, (), {
  if (Module.tbgwafInbox) return;
  Module.tbgwafInbox = [];
  if (Module.tbgwafBus) Module.tbgwafBus.inboxes.push(Module.tbgwafInbox);
});

EM_JS(void, tbgwaf_channel_post, (const char* msg), {
  if (!Module.tbgwafBus) return;
  const text = UTF8ToString(msg);
  for (const inbox of Module.tbgwafBus.inboxes) {
    if (inbox !== Module.tbgwafInbox) inbox.push(text);
  }
});

// Returns a malloc'd message (caller frees) or null if the inbox is empty.
EM_JS(char*, tbgwaf_channel_next, (), {
  if (!Module.tbgwafInbox || Module.tbgwafInbox.length === 0) return 0;
  const msg = Module.tbgwafInbox.shift();
  const size = lengthBytesUTF8(msg) + 1;
  const ptr = _malloc(size);
  stringToUTF8(msg, ptr, size);
  return ptr;
});

EM_JS(void, tbgwaf_set_blur, (int on), {
  const el = Module.tbgwafBlurOverlay;
  if (el) el.classList.toggle("on", !!on);
});

// CSS size of this instance's own canvas (each client renders into its own
// <canvas>, so a global "#canvas" selector would not do).
EM_JS(double, tbgwaf_canvas_css_width, (), {
  return Module.canvas.getBoundingClientRect().width;
});
EM_JS(double, tbgwaf_canvas_css_height, (), {
  return Module.canvas.getBoundingClientRect().height;
});
#endif

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

}  // namespace

int main() {
#ifdef __EMSCRIPTEN__
  tbgwaf_bind_canvas();
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

  // One independent orbit camera per team (index = static_cast<int>(Team)),
  // so hot-seat play keeps each side's own view; in two-pane play only the
  // instance's own team's camera is ever used.
  std::array<gfx::OrbitCamera, kTeamCount> cameras;
  GameLogic game;

  // Which team this client instance plays. nullopt = hot-seat (follow the
  // active team).
  std::optional<Team> fixedTeam;
  bool networked = false;
  // Until a peer answers our hello (or the timeout passes), our fresh local
  // state may be stale relative to a match already in progress in the other
  // instance, so we don't act on it.
  bool awaitingPeerSync = false;
  Uint32 peerSyncDeadline = 0;
#ifdef __EMSCRIPTEN__
  if (const int requested = tbgwaf_requested_player(); requested >= 0) {
    fixedTeam = requested == 0 ? Team::Blue : Team::Red;
    networked = true;
    tbgwaf_channel_open();
    tbgwaf_channel_post("H");
    awaitingPeerSync = true;
    peerSyncDeadline = SDL_GetTicks() + kPeerSyncTimeoutMs;
  }
#endif
  std::string lastSentState;
  bool forceBroadcast = false;
  Team hotSeatTeam = Team::Blue;

  bool quit = false;
  bool leftDragging = false;
  float leftDragDistance = 0.0f;  // Accumulated pixels moved during the current left-drag.
  glm::vec3 hoveredGroundPoint(0.0f);
  bool hasHoveredGroundPoint = false;

  // Headless CI (xvfb) has no real user input; bound the run so the smoke
  // test still exits on its own after exercising a few frames.
  const bool isSmokeTest = std::getenv("TBGWAF_SMOKE_TEST") != nullptr;
  int frameCount = 0;
  constexpr int kSmokeTestMaxFrames = 60;
  Uint32 lastFrameTicks = SDL_GetTicks();

  auto currentActiveTeam = [&]() -> std::optional<Team> {
    if (game.Mode() == InputMode::GameOver) return std::nullopt;
    return game.CurrentTeam();
  };

  auto runFrame = [&]() {
#ifdef __EMSCRIPTEN__
    // Track our own canvas element's CSS size (half the page per pane).
    {
      const double cssW = tbgwaf_canvas_css_width();
      const double cssH = tbgwaf_canvas_css_height();
      int curW = 0, curH = 0;
      SDL_GetWindowSize(window, &curW, &curH);
      if (cssW >= 1.0 && cssH >= 1.0 && (curW != static_cast<int>(cssW) || curH != static_cast<int>(cssH))) {
        SDL_SetWindowSize(window, static_cast<int>(cssW), static_cast<int>(cssH));
      }
    }
#endif
    int windowWidth = kInitialWindowWidth, windowHeight = kInitialWindowHeight;
    SDL_GetWindowSize(window, &windowWidth, &windowHeight);

    // --- Sync: apply the peer's state / answer its hello. ---
#ifdef __EMSCRIPTEN__
    if (networked) {
      while (char* raw = tbgwaf_channel_next()) {
        const std::string msg(raw);
        std::free(raw);
        if (msg == "H") {
          // A peer instance just opened: hand it the current match state.
          forceBroadcast = true;
        } else if (msg.size() > 2 && msg[0] == 'S' && msg[1] == ' ') {
          // Only a follower (or a finished match) takes state from the
          // peer; the active instance is the source of truth.
          const auto active = currentActiveTeam();
          const bool authoritative = !awaitingPeerSync && active && *active == *fixedTeam;
          GameSnapshot snap;
          if (!authoritative && DeserializeSnapshot(msg.substr(2), &snap) &&
              game.ImportState(snap)) {
            awaitingPeerSync = false;
            lastSentState = SerializeSnapshot(game.ExportState());
          }
        }
      }
      if (awaitingPeerSync && SDL_GetTicks() >= peerSyncDeadline) awaitingPeerSync = false;
    }
#endif

    const std::optional<Team> activeTeam = currentActiveTeam();
    if (activeTeam) hotSeatTeam = *activeTeam;
    const Team localTeam = fixedTeam.value_or(hotSeatTeam);
    // The local UI may drive the game only on our team's turn (and, when
    // networked, once we know we're not looking at a stale fresh match).
    const bool isActive = !awaitingPeerSync && activeTeam && *activeTeam == localTeam;
    const bool blurred = networked && game.Mode() != InputMode::GameOver && !isActive;
#ifdef __EMSCRIPTEN__
    tbgwaf_set_blur(blurred ? 1 : 0);
#endif
    gfx::OrbitCamera& camera = cameras[static_cast<int>(localTeam)];
    const float aspect = static_cast<float>(windowWidth) / static_cast<float>(windowHeight);

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
      } else if (event.type == SDL_MOUSEBUTTONDOWN && event.button.button == SDL_BUTTON_LEFT) {
        leftDragging = true;
        leftDragDistance = 0.0f;
      } else if (event.type == SDL_MOUSEMOTION) {
        mouseX = event.motion.x;
        mouseY = event.motion.y;
        if (leftDragging && !ImGui::GetIO().WantCaptureMouse) {
          constexpr float kPanSpeed = 0.0015f;
          // Drag the world under the cursor: target moves opposite to the drag.
          camera.Pan(-event.motion.xrel * kPanSpeed, event.motion.yrel * kPanSpeed);
          leftDragDistance += std::hypot(static_cast<float>(event.motion.xrel),
                                          static_cast<float>(event.motion.yrel));
        }
        if ((event.motion.state & SDL_BUTTON_RMASK) && !ImGui::GetIO().WantCaptureMouse) {
          constexpr float kRotateSpeed = 0.005f;
          camera.Rotate(-event.motion.xrel * kRotateSpeed, event.motion.yrel * kRotateSpeed);
        }
      } else if (event.type == SDL_MOUSEWHEEL) {
        if (!ImGui::GetIO().WantCaptureMouse) {
          constexpr float kZoomSpeed = 1.5f;
          camera.Zoom(-event.wheel.y * kZoomSpeed);
        }
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_LEFT) {
        // A drag that panned the camera must not also fire a click.
        constexpr float kClickDragThresholdPx = 5.0f;
        if (leftDragDistance < kClickDragThresholdPx) leftClickPending = true;
        leftDragging = false;
        leftDragDistance = 0.0f;
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
    // Only the authoritative side simulates; a follower just mirrors
    // snapshots (and in hot-seat mode the local side is always the actor).
    if (isActive) game.Update(dt);

    const bool fogActive = game.Mode() != InputMode::GameOver;
    TeamVisibility visibility;
    if (fogActive) visibility = game.ComputeVisibility(localTeam);

    // --- UI ---
    if (const auto winner = game.Winner()) {
      ImGui::SetNextWindowPos(ImVec2(windowWidth * 0.5f, windowHeight * 0.3f), ImGuiCond_Always,
                               ImVec2(0.5f, 0.5f));
      ImGui::Begin("Game Over", nullptr,
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize);
      ImGui::Text("%s team wins!", TeamName(*winner));
      if (ImGui::Button("New Match")) {
        game.Reset();
        forceBroadcast = true;
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

      if (const auto selectedId = game.SelectedUnitId(); selectedId && isActive) {
        const Unit* selected = game.FindUnit(*selectedId);
        if (selected && (game.Mode() == InputMode::ActionMenu ||
                          game.Mode() == InputMode::AwaitingMoveDestination ||
                          game.Mode() == InputMode::AwaitingShootTarget)) {
          const glm::mat4 view = camera.ViewMatrix();
          const glm::mat4 proj = camera.ProjectionMatrix(aspect);
          const glm::vec4 viewport(0.0f, 0.0f, static_cast<float>(windowWidth),
                                    static_cast<float>(windowHeight));
          const glm::vec3 headTop = selected->position + glm::vec3(0.0f, 1.9f, 0.0f);
          const glm::vec3 screenPos = glm::project(headTop, view, proj, viewport);
          // glm::project assumes a bottom-left viewport origin; flip Y for
          // ImGui's top-left screen space.
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

    // Team label for this pane.
    {
      const char* status = game.Winner() ? "" : (isActive ? " - your turn" : "");
      char label[64];
      std::snprintf(label, sizeof(label), "%s%s", TeamName(localTeam), status);
      ImGui::GetForegroundDrawList()->AddText(ImVec2(10.0f, windowHeight - 24.0f),
                                               IM_COL32(255, 255, 255, 220), label);
    }

    // --- Dispatch deferred input, now that WantCaptureMouse reflects the UI
    // actually built this frame. ---
    if (isActive && escapePending) game.CancelAction();

    const bool uiWantsMouse = ImGui::GetIO().WantCaptureMouse;
    if (isActive && !uiWantsMouse) {
      if (game.Mode() == InputMode::AwaitingMoveDestination) {
        const gfx::Ray hoverRay =
            camera.ScreenPointToRay(static_cast<float>(mouseX), static_cast<float>(mouseY),
                                     static_cast<float>(windowWidth),
                                     static_cast<float>(windowHeight));
        glm::vec3 hoverPoint;
        if (IntersectGroundOrClimbTop(hoverRay, game.GetScene().obstacles, &hoverPoint)) {
          hoveredGroundPoint = hoverPoint;
          hasHoveredGroundPoint = true;
          game.HoverGround(hoverPoint);
        }
      }
      if (leftClickPending) {
        const gfx::Ray clickRay =
            camera.ScreenPointToRay(static_cast<float>(leftClickX), static_cast<float>(leftClickY),
                                     static_cast<float>(windowWidth),
                                     static_cast<float>(windowHeight));
        // Only figures actually rendered this frame (own team, or enemies
        // currently inside this team's FOV) are pickable -- a hidden
        // enemy's collision box must not be clickable just because it
        // happens to sit behind something that is drawn.
        std::vector<Unit> pickableUnits;
        for (const Unit& unit : game.GetScene().units) {
          if (unit.alive &&
              gfx::IsUnitVisibleForRender(unit, localTeam, fogActive, visibility)) {
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

    // --- Sync: publish our state if we drove it (or a peer asked for it). ---
#ifdef __EMSCRIPTEN__
    if (networked && !awaitingPeerSync) {
      const std::string state = SerializeSnapshot(game.ExportState());
      if ((isActive && state != lastSentState) || forceBroadcast) {
        tbgwaf_channel_post(("S " + state).c_str());
        lastSentState = state;
      }
      forceBroadcast = false;
    }
#endif

    // --- Render: one shadow pass + one color pass, inside
    // SceneRenderer::RenderPane. Selection/move-preview overlays belong to
    // whichever team is currently acting, so only the active pane gets them. ---
    gfx::PaneOverlays overlays;
    if (isActive) {
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
    renderer.RenderPane(game, localTeam, fogActive, visibility, camera, 0, 0, windowWidth,
                        windowHeight, overlays);

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
