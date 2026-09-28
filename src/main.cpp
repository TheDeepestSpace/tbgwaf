#include <SDL.h>
#include <GLES3/gl3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <imgui.h>
#include <backends/imgui_impl_opengl3.h>
#include <backends/imgui_impl_sdl2.h>

#include <cstdio>
#include <cstdlib>
#include <limits>
#include <vector>

#include "game/GameLogic.h"
#include "game/Raycast.h"
#include "game/Types.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/Mesh.h"
#include "gfx/Shader.h"

using tactics::AABB;
using tactics::GameLogic;
using tactics::InputMode;
using tactics::Obstacle;
using tactics::Team;
using tactics::TeamVisibility;
using tactics::Unit;

namespace {

constexpr int kInitialWindowWidth = 1280;
constexpr int kInitialWindowHeight = 720;
constexpr int kShadowMapSize = 2048;

// Flat, unlit shader used for UI-ish overlays (selection highlights, the
// move-path preview line) that should stay crisp regardless of shadowing.
const char* kUnlitVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
void main() {
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kUnlitFragmentShaderSrc = R"(#version 300 es
precision mediump float;
uniform vec4 uColor;
out vec4 FragColor;
void main() {
  FragColor = uColor;
}
)";

// Lit shader used for the actual scene geometry (ground, obstacles,
// figures): a single directional light with a basic shadow map, and a
// per-face normal derived from screen-space position derivatives (GLSL ES
// 3.00 has dFdx/dFdy as core, so every cube face gets a correct flat normal
// without needing a dedicated per-face-vertex mesh).
const char* kLitVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uMVP;
uniform mat4 uModel;
uniform mat4 uLightSpaceMatrix;
out vec3 vWorldPos;
out vec4 vLightSpacePos;
void main() {
  vec4 world = uModel * vec4(aPos, 1.0);
  vWorldPos = world.xyz;
  vLightSpacePos = uLightSpaceMatrix * world;
  gl_Position = uMVP * vec4(aPos, 1.0);
}
)";

const char* kLitFragmentShaderSrc = R"(#version 300 es
precision highp float;
in vec3 vWorldPos;
in vec4 vLightSpacePos;
uniform vec4 uColor;
uniform vec3 uLightDir;  // Direction the light travels; surfaces face -uLightDir.
uniform vec3 uViewPos;
uniform sampler2D uShadowMap;
out vec4 FragColor;

float ComputeShadow(vec3 normal) {
  vec3 proj = vLightSpacePos.xyz / vLightSpacePos.w;
  proj = proj * 0.5 + 0.5;
  if (proj.x < 0.0 || proj.x > 1.0 || proj.y < 0.0 || proj.y > 1.0 || proj.z > 1.0) {
    return 0.0;  // Outside the light's frustum: treat as unshadowed.
  }
  float closestDepth = texture(uShadowMap, proj.xy).r;
  float currentDepth = proj.z;
  float bias = max(0.003 * (1.0 - max(dot(normal, -uLightDir), 0.0)), 0.0008);
  return (currentDepth - bias) > closestDepth ? 1.0 : 0.0;
}

void main() {
  vec3 normal = normalize(cross(dFdx(vWorldPos), dFdy(vWorldPos)));
  vec3 viewDir = normalize(uViewPos - vWorldPos);
  if (dot(normal, viewDir) < 0.0) normal = -normal;

  float diffuse = max(dot(normal, -uLightDir), 0.0);
  float shadow = ComputeShadow(normal);
  const float kAmbient = 0.35;
  float lit = kAmbient + (1.0 - shadow) * diffuse * 0.65;
  FragColor = vec4(uColor.rgb * lit, uColor.a);
}
)";

// Depth-only shader for the shadow map pass.
const char* kDepthVertexShaderSrc = R"(#version 300 es
layout(location = 0) in vec3 aPos;
uniform mat4 uLightMVP;
void main() {
  gl_Position = uLightMVP * vec4(aPos, 1.0);
}
)";

const char* kDepthFragmentShaderSrc = R"(#version 300 es
precision mediump float;
void main() {}
)";

void DrawBox(const gfx::Shader& shader, const gfx::CubeMesh& cube, const glm::mat4& viewProj,
             const glm::vec3& minCorner, const glm::vec3& size, const glm::vec4& color) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetVec4("uColor", color);
  cube.Draw();
}

// Lit variant: also uploads the model matrix (for world-space position/light
// coordinates in the fragment shader) alongside the color. Light direction,
// view position, and the shadow map itself are set once per frame, not
// per-object, since they don't vary between draw calls.
void DrawBoxLit(const gfx::Shader& shader, const gfx::CubeMesh& cube, const glm::mat4& viewProj,
                 const glm::mat4& lightSpaceMatrix, const glm::vec3& minCorner,
                 const glm::vec3& size, const glm::vec4& color) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
  shader.SetMat4("uModel", model);
  shader.SetMat4("uMVP", viewProj * model);
  shader.SetMat4("uLightSpaceMatrix", lightSpaceMatrix);
  shader.SetVec4("uColor", color);
  cube.Draw();
}

void DrawBoxDepth(const gfx::Shader& shader, const gfx::CubeMesh& cube,
                   const glm::mat4& lightSpaceMatrix, const glm::vec3& minCorner,
                   const glm::vec3& size) {
  const glm::mat4 model =
      glm::translate(glm::mat4(1.0f), minCorner) * glm::scale(glm::mat4(1.0f), size);
  shader.SetMat4("uLightMVP", lightSpaceMatrix * model);
  cube.Draw();
}

void UnitBoxes(const Unit& unit, glm::vec3* outBodyMin, glm::vec3* outBodySize,
               glm::vec3* outHeadMin, glm::vec3* outHeadSize) {
  constexpr float kHalfWidth = tactics::constants::kUnitHalfWidth;
  constexpr float kBodyHeight = 1.4f;
  constexpr float kHeadSize = 0.4f;
  *outBodyMin = unit.position - glm::vec3(kHalfWidth, 0.0f, kHalfWidth);
  *outBodySize = glm::vec3(kHalfWidth * 2.0f, kBodyHeight, kHalfWidth * 2.0f);
  *outHeadMin = unit.position + glm::vec3(-kHeadSize * 0.5f, kBodyHeight, -kHeadSize * 0.5f);
  *outHeadSize = glm::vec3(kHeadSize, kHeadSize, kHeadSize);
}

void DrawUnit(const gfx::Shader& shader, const gfx::CubeMesh& cube, const glm::mat4& viewProj,
              const glm::mat4& lightSpaceMatrix, const Unit& unit) {
  const glm::vec4 color = unit.team == Team::Blue ? glm::vec4(0.2f, 0.45f, 0.95f, 1.0f)
                                                    : glm::vec4(0.9f, 0.25f, 0.22f, 1.0f);
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  DrawBoxLit(shader, cube, viewProj, lightSpaceMatrix, bodyMin, bodySize, color);
  DrawBoxLit(shader, cube, viewProj, lightSpaceMatrix, headMin, headSize, color);
}

void DrawUnitDepth(const gfx::Shader& shader, const gfx::CubeMesh& cube,
                    const glm::mat4& lightSpaceMatrix, const Unit& unit) {
  glm::vec3 bodyMin, bodySize, headMin, headSize;
  UnitBoxes(unit, &bodyMin, &bodySize, &headMin, &headSize);
  DrawBoxDepth(shader, cube, lightSpaceMatrix, bodyMin, bodySize);
  DrawBoxDepth(shader, cube, lightSpaceMatrix, headMin, headSize);
}

void DrawHighlight(const gfx::Shader& shader, const gfx::CubeMesh& cube, const glm::mat4& viewProj,
                    const glm::vec3& position, const glm::vec4& color) {
  constexpr float kHalf = 0.5f;
  const glm::vec3 minCorner = position + glm::vec3(-kHalf, 0.01f, -kHalf);
  DrawBox(shader, cube, viewProj, minCorner, glm::vec3(kHalf * 2.0f, 0.04f, kHalf * 2.0f), color);
}

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

// Stage-B fog-of-war: a unit is drawable/pickable in this frame's view if
// it's on the viewing team (you always see your own figures) or, when fog
// is active, currently inside that team's combined FOV.
bool IsUnitVisibleForRender(const Unit& unit, Team viewingTeam, bool fogActive,
                             const TeamVisibility& visibility) {
  if (unit.team == viewingTeam) return true;
  if (!fogActive) return true;
  return visibility.UnitVisible(unit.id);
}

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

  gfx::Shader unlitShader;
  if (!unlitShader.Compile(kUnlitVertexShaderSrc, kUnlitFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the unlit shader\n");
    return 1;
  }
  gfx::Shader litShader;
  if (!litShader.Compile(kLitVertexShaderSrc, kLitFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the lit shader\n");
    return 1;
  }
  gfx::Shader depthShader;
  if (!depthShader.Compile(kDepthVertexShaderSrc, kDepthFragmentShaderSrc)) {
    std::fprintf(stderr, "Failed to compile the shadow depth shader\n");
    return 1;
  }
  gfx::CubeMesh cubeMesh;
  cubeMesh.Init();
  gfx::LineMesh pathLine;
  pathLine.Init();

  // Stage-C: a single directional light (simulating overhead factory
  // lighting) casting a basic shadow map, single cascade, hard-edged.
  GLuint shadowFbo = 0;
  GLuint shadowDepthTex = 0;
  glGenFramebuffers(1, &shadowFbo);
  glGenTextures(1, &shadowDepthTex);
  glBindTexture(GL_TEXTURE_2D, shadowDepthTex);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, kShadowMapSize, kShadowMapSize, 0,
               GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowDepthTex, 0);
  {
    const GLenum noColorBuffer = GL_NONE;
    glDrawBuffers(1, &noColorBuffer);
    glReadBuffer(GL_NONE);
  }
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
    std::fprintf(stderr, "Shadow map framebuffer incomplete\n");
    return 1;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // The light and map geometry are both static, so the light-space matrix
  // is fixed for the whole session.
  const glm::vec3 lightDir = glm::normalize(glm::vec3(0.35f, -1.0f, 0.25f));
  const float mapHalfExtentForLight = tactics::constants::kMapHalfExtent;
  const float lightDistance = mapHalfExtentForLight * 3.0f;
  const glm::vec3 lightPos = -lightDir * lightDistance;
  const glm::mat4 lightView = glm::lookAt(lightPos, glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
  const float orthoHalfExtent = mapHalfExtentForLight * 1.5f;
  const glm::mat4 lightProj =
      glm::ortho(-orthoHalfExtent, orthoHalfExtent, -orthoHalfExtent, orthoHalfExtent, 0.1f,
                 lightDistance * 2.0f);
  const glm::mat4 lightSpaceMatrix = lightProj * lightView;

  gfx::OrbitCamera camera;
  GameLogic game;

  bool quit = false;
  bool rightDragging = false;
  glm::vec3 hoveredGroundPoint(0.0f);
  bool hasHoveredGroundPoint = false;

  // Headless CI (xvfb) has no real user input; bound the run so the smoke
  // test still exits on its own after exercising a few frames.
  const bool isSmokeTest = std::getenv("TBGWAF_SMOKE_TEST") != nullptr;
  int frameCount = 0;
  constexpr int kSmokeTestMaxFrames = 60;

  while (!quit) {
    int windowWidth = kInitialWindowWidth, windowHeight = kInitialWindowHeight;
    SDL_GetWindowSize(window, &windowWidth, &windowHeight);

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
        rightDragging = true;
      } else if (event.type == SDL_MOUSEBUTTONUP && event.button.button == SDL_BUTTON_RIGHT) {
        rightDragging = false;
      } else if (event.type == SDL_MOUSEMOTION) {
        mouseX = event.motion.x;
        mouseY = event.motion.y;
        if (rightDragging && !ImGui::GetIO().WantCaptureMouse) {
          constexpr float kRotateSpeed = 0.005f;
          camera.Rotate(-event.motion.xrel * kRotateSpeed, event.motion.yrel * kRotateSpeed);
        }
      } else if (event.type == SDL_MOUSEWHEEL) {
        if (!ImGui::GetIO().WantCaptureMouse) {
          constexpr float kZoomSpeed = 1.5f;
          camera.Zoom(-event.wheel.y * kZoomSpeed);
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

    const glm::mat4 view = camera.ViewMatrix();
    const glm::mat4 proj =
        camera.ProjectionMatrix(static_cast<float>(windowWidth) / static_cast<float>(windowHeight));
    const glm::mat4 viewProj = proj * view;
    const ImVec4 viewport(0, 0, static_cast<float>(windowWidth), static_cast<float>(windowHeight));

    // Stage-B fog-of-war: this is a local hot-seat prototype (see the
    // WASM/Pages issue for why it isn't split into two per-team views yet),
    // so we render whichever team currently has the turn's fog-of-war. Once
    // the match ends there's nothing left to hide.
    Team viewingTeam = Team::Blue;
    TeamVisibility viewingTeamVisibility;
    bool fogActive = false;
    if (game.Mode() != InputMode::GameOver) {
      if (const auto actorId = game.CurrentActorId()) {
        if (const Unit* actor = game.FindUnit(*actorId)) {
          viewingTeam = actor->team;
          viewingTeamVisibility = game.ComputeVisibility(viewingTeam);
          fogActive = true;
        }
      }
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
      ImGui::SetNextWindowPos(ImVec2(10, 10), ImGuiCond_Always);
      ImGui::Begin("Turn", nullptr,
                   ImGuiWindowFlags_NoResize | ImGuiWindowFlags_AlwaysAutoResize |
                       ImGuiWindowFlags_NoMove);
      ImGui::Text("Round %d", game.RoundNumber());
      if (const auto actorId = game.CurrentActorId()) {
        const Unit* actor = game.FindUnit(*actorId);
        if (actor) {
          ImGui::Text("%s team's turn (figure #%d)", TeamName(actor->team), actor->id);
        }
      }
      switch (game.Mode()) {
        case InputMode::AwaitingSelection:
          ImGui::TextWrapped("Click the highlighted figure to act.");
          break;
        case InputMode::ActionMenu:
          ImGui::TextWrapped("Choose an action.");
          break;
        case InputMode::AwaitingMoveDestination:
          ImGui::TextWrapped("Click a destination on the ground (Esc to cancel).");
          break;
        case InputMode::AwaitingShootTarget:
          ImGui::TextWrapped("Click an enemy figure to shoot (Esc to cancel).");
          break;
        default:
          break;
      }
      ImGui::End();

      if (const auto selectedId = game.SelectedUnitId()) {
        const Unit* selected = game.FindUnit(*selectedId);
        if (selected && (game.Mode() == InputMode::ActionMenu ||
                          game.Mode() == InputMode::AwaitingMoveDestination ||
                          game.Mode() == InputMode::AwaitingShootTarget)) {
          const glm::vec3 headTop = selected->position + glm::vec3(0.0f, 1.9f, 0.0f);
          const glm::vec3 screenPos =
              glm::project(headTop, view, proj, glm::vec4(viewport.x, viewport.y, viewport.z, viewport.w));
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
            if (ImGui::Button("Pass")) game.ChoosePass();
          } else {
            if (ImGui::Button("Cancel")) game.CancelAction();
          }
          ImGui::End();
        }
      }
    }

    // --- Dispatch deferred input, now that WantCaptureMouse reflects the UI
    // actually built this frame. ---
    if (escapePending) game.CancelAction();

    const bool worldInputAllowed = !ImGui::GetIO().WantCaptureMouse;
    if (worldInputAllowed) {
      if (game.Mode() == InputMode::AwaitingMoveDestination) {
        const gfx::Ray hoverRay = camera.ScreenPointToRay(
            static_cast<float>(mouseX), static_cast<float>(mouseY),
            static_cast<float>(windowWidth), static_cast<float>(windowHeight));
        glm::vec3 hoverPoint;
        if (IntersectGroundOrClimbTop(hoverRay, game.GetScene().obstacles, &hoverPoint)) {
          hoveredGroundPoint = hoverPoint;
          hasHoveredGroundPoint = true;
          game.HoverGround(hoverPoint);
        }
      }
      if (leftClickPending) {
        const gfx::Ray clickRay = camera.ScreenPointToRay(
            static_cast<float>(leftClickX), static_cast<float>(leftClickY),
            static_cast<float>(windowWidth), static_cast<float>(windowHeight));
        // Only figures actually rendered this frame (own team, or enemies
        // currently inside the viewing team's FOV) are pickable -- a hidden
        // enemy's collision box must not be clickable just because it
        // happens to sit behind something that is drawn.
        std::vector<Unit> pickableUnits;
        for (const Unit& unit : game.GetScene().units) {
          if (unit.alive && IsUnitVisibleForRender(unit, viewingTeam, fogActive,
                                                    viewingTeamVisibility)) {
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

    // --- Shadow pass: render casters (obstacles + currently-visible figures)
    // depth-only from the light's point of view. Units hidden by fog-of-war
    // must not cast a shadow either, or their position would leak through
    // it. ---
    const auto& obstacles = game.GetScene().obstacles;
    glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo);
    glViewport(0, 0, kShadowMapSize, kShadowMapSize);
    glClear(GL_DEPTH_BUFFER_BIT);
    depthShader.Use();
    for (const auto& obstacle : obstacles) {
      const AABB& bounds = obstacle.bounds;
      DrawBoxDepth(depthShader, cubeMesh, lightSpaceMatrix, bounds.min, bounds.max - bounds.min);
    }
    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive) continue;
      if (!IsUnitVisibleForRender(unit, viewingTeam, fogActive, viewingTeamVisibility)) continue;
      DrawUnitDepth(depthShader, cubeMesh, lightSpaceMatrix, unit);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // --- Render ---
    glViewport(0, 0, windowWidth, windowHeight);
    glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    litShader.Use();
    litShader.SetVec3("uLightDir", lightDir);
    litShader.SetVec3("uViewPos", camera.Position());
    litShader.SetInt("uShadowMap", 0);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, shadowDepthTex);

    const float mapHalfExtent = tactics::constants::kMapHalfExtent;
    DrawBoxLit(litShader, cubeMesh, viewProj, lightSpaceMatrix,
               glm::vec3(-mapHalfExtent, -0.05f, -mapHalfExtent),
               glm::vec3(mapHalfExtent * 2.0f, 0.05f, mapHalfExtent * 2.0f),
               glm::vec4(0.16f, 0.18f, 0.20f, 1.0f));

    for (size_t i = 0; i < obstacles.size(); ++i) {
      const bool obstacleVisible = !fogActive || viewingTeamVisibility.ObstacleVisible(i);
      const AABB& bounds = obstacles[i].bounds;
      const glm::vec4 baseColor = obstacles[i].climbable ? glm::vec4(0.55f, 0.48f, 0.3f, 1.0f)
                                                          : glm::vec4(0.55f, 0.55f, 0.6f, 1.0f);
      const glm::vec4 color = obstacleVisible ? baseColor : glm::vec4(0.22f, 0.22f, 0.24f, 1.0f);
      DrawBoxLit(litShader, cubeMesh, viewProj, lightSpaceMatrix, bounds.min,
                 bounds.max - bounds.min, color);
    }

    for (const Unit& unit : game.GetScene().units) {
      if (!unit.alive) continue;
      if (!IsUnitVisibleForRender(unit, viewingTeam, fogActive, viewingTeamVisibility)) continue;
      DrawUnit(litShader, cubeMesh, viewProj, lightSpaceMatrix, unit);
    }

    unlitShader.Use();
    if (const auto actorId = game.CurrentActorId(); actorId && game.Mode() != InputMode::GameOver) {
      if (const Unit* actor = game.FindUnit(*actorId)) {
        DrawHighlight(unlitShader, cubeMesh, viewProj, actor->position,
                      glm::vec4(1.0f, 1.0f, 1.0f, 1.0f));
      }
    }
    if (const auto selectedId = game.SelectedUnitId()) {
      if (const Unit* selected = game.FindUnit(*selectedId)) {
        DrawHighlight(unlitShader, cubeMesh, viewProj, selected->position,
                      glm::vec4(1.0f, 0.9f, 0.15f, 1.0f));
      }
    }

    if (game.Mode() == InputMode::AwaitingMoveDestination) {
      if (game.MovePreviewValid() && game.MovePreviewPath().size() >= 2) {
        pathLine.SetPoints(game.MovePreviewPath());
        unlitShader.SetMat4("uMVP", viewProj);
        unlitShader.SetVec4("uColor", glm::vec4(1.0f, 0.85f, 0.2f, 1.0f));
        pathLine.Draw();
      } else if (hasHoveredGroundPoint) {
        DrawHighlight(unlitShader, cubeMesh, viewProj, hoveredGroundPoint,
                      glm::vec4(0.9f, 0.15f, 0.15f, 1.0f));
      }
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    SDL_GL_SwapWindow(window);

    if (isSmokeTest && ++frameCount >= kSmokeTestMaxFrames) {
      quit = true;
    }
  }

  pathLine.Destroy();
  cubeMesh.Destroy();
  glDeleteTextures(1, &shadowDepthTex);
  glDeleteFramebuffers(1, &shadowFbo);
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplSDL2_Shutdown();
  ImGui::DestroyContext();
  SDL_GL_DeleteContext(glContext);
  SDL_DestroyWindow(window);
  SDL_Quit();
  return 0;
}
