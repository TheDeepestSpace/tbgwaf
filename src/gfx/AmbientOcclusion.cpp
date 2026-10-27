#include "gfx/AmbientOcclusion.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>

#include "gfx/Mesh.h"
#include "gfx/SceneGeometry.h"
#include "game/Geometry.h"

#include <glm/gtc/matrix_transform.hpp>

namespace gfx {
namespace {
const char* kVertex = R"(#version 300 es
out vec2 vUV;
void main() {
  vUV = vec2(float((gl_VertexID << 1) & 2), float(gl_VertexID & 2));
  gl_Position = vec4(vUV * 2.0 - 1.0, 0.0, 1.0);
}
)";

// Both passes address the pane in pane-relative UVs (vUV, uPaneSize) and
// scale by uUvScale only when sampling: the targets may be larger than the
// pane (grow-only allocation shared by panes of different sizes), and the
// texels beyond the pane hold stale data that must never be sampled.
const char* kOcclusion = R"(#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
in vec2 vUV;
uniform sampler2D uDepth;
uniform sampler2D uNormal;
uniform mat4 uInverseViewProjection;
uniform mat4 uProjection;
uniform mat4 uView;
uniform vec2 uPaneSize;  // Pane size in pixels.
uniform vec2 uUvScale;   // Pane size / target texture size.
uniform float uRadius;   // World-space reach of the shading.
uniform int uSteps;      // Depth samples marched per ray.
out vec4 FragColor;

vec3 WorldPosition(vec2 uv, float depth) {
  vec4 p = uInverseViewProjection * vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
  return p.xyz / p.w;
}

void main() {
  float depth = texture(uDepth, vUV * uUvScale).r;
  if (depth >= 1.0) { FragColor = vec4(0.0); return; }
  vec3 p = WorldPosition(vUV, depth);
  vec3 normal = normalize(texture(uNormal, vUV * uUvScale).xyz * 2.0 - 1.0);
  // World-space radius: the contact shading keeps its scale as we zoom.
  float radius = uRadius;
  float viewDepth = -(uView * vec4(p, 1.0)).z;
  vec2 uvRadius = 0.5 * radius * vec2(uProjection[0][0], uProjection[1][1]) / viewDepth;
  vec2 size = uPaneSize;
  // Stable rotation breaks up radial bands without temporal shimmer.
  uvec2 pixel = uvec2(gl_FragCoord.xy);
  uint hash = (pixel.x * 1664525u) ^ (pixel.y * 1013904223u);
  hash = (hash ^ (hash >> 16u)) * 2246822519u;
  float rotation = float(hash & 65535u) / 65535.0 * 6.2831853;
  float occlusion = 0.0;
  for (int direction = 0; direction < 8; ++direction) {
    float angle = rotation + float(direction) * 0.78539816;
    vec2 ray = vec2(cos(angle), sin(angle)) * uvRadius;
    float horizon = 0.0;
    for (int step = 0; step < uSteps; ++step) {
      float fraction = (float(step) + 0.5) / float(uSteps);
      vec2 uv = (floor((vUV + ray * fraction * fraction) * size) + 0.5) / size;
      // Missing/offscreen geometry contributes no occlusion, never a
      // clamped silhouette or information from the other player's pane.
      if (any(lessThan(uv, vec2(0.0))) || any(greaterThanEqual(uv, vec2(1.0)))) continue;
      float sampleDepth = texture(uDepth, uv * uUvScale).r;
      if (sampleDepth >= 1.0) continue;
      vec3 delta = WorldPosition(uv, sampleDepth) - p;
      float distance = length(delta);
      if (distance < 0.02) continue;
      float falloff = 1.0 - smoothstep(0.0, radius, distance);
      // A small normal bias avoids self-occlusion on flat planes. Only
      // nearby geometry above the receiver's tangent plane can shade it.
      float obscurance = max(dot(normal, delta / distance) - 0.08, 0.0) * falloff;
      horizon = max(horizon, obscurance);
    }
    occlusion += horizon;
  }
  FragColor = vec4(vec3(clamp(occlusion / 8.0, 0.0, 1.0)), 1.0);
}
)";

// PR #178: full-sphere directional visibility, accumulated into an ambient
// cube in RGBA8. Explicit highp samplers avoid fp16 depth quantization on
// llvmpipe (docs/fov-shadow-map.md). No float attachments or blending.
const char* kWorldOcclusion = R"(#version 300 es
precision highp float;
precision highp int;
precision highp sampler2DArray;
precision highp sampler3D;
uniform sampler2DArray uDepthMaps;
uniform sampler3D uPositive;
uniform sampler3D uNegative;
uniform mat4 uVP[16];
uniform vec3 uDirection[16];
uniform int uCount;
uniform int uSlice;
uniform vec3 uOrigin;
uniform float uSpacing;
uniform float uDiameter;
uniform float uRadius;
uniform vec3 uPositiveWeight;
uniform vec3 uNegativeWeight;
layout(location = 0) out vec4 Positive;
layout(location = 1) out vec4 Negative;

float Visibility(vec3 p, int layer) {
  vec3 c = (uVP[layer] * vec4(p, 1.0)).xyz * 0.5 + 0.5;
  if (any(lessThan(c, vec3(0.0))) || any(greaterThan(c, vec3(1.0)))) return 1.0;
  ivec2 size = textureSize(uDepthMaps, 0).xy;
  vec2 f = c.xy * vec2(size) - 0.5;
  ivec2 base = ivec2(floor(f));
  vec2 t = fract(f);
  // 1.5 depth texels in world units, divided by the depth range (2R).
  float bias = 1.5 / float(size.x);
  float occ = 0.0;
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 2; ++x) {
      ivec2 tc = clamp(base + ivec2(x, y), ivec2(0), size - 1);
      float delta = c.z - texelFetch(uDepthMaps, ivec3(tc, layer), 0).r;
      float distance = delta * uDiameter;
      float weight = (x == 0 ? 1.0 - t.x : t.x) * (y == 0 ? 1.0 - t.y : t.y);
      occ += weight * step(bias, delta) * (1.0 - smoothstep(0.0, uRadius, distance));
    }
  }
  return 1.0 - occ;
}

void main() {
  ivec3 cell = ivec3(ivec2(gl_FragCoord.xy), uSlice);
  Positive = texelFetch(uPositive, cell, 0);
  Negative = texelFetch(uNegative, cell, 0);
  if (Positive.a == 0.0) return;
  vec3 probe = uOrigin + (vec3(cell) + 0.5) * uSpacing;
  vec3 posSum = vec3(0.0), negSum = vec3(0.0);
  for (int i = 0; i < uCount; ++i) {
    for (int a = 0; a < 3; ++a) {
      float d = uDirection[i][a];
      vec3 axis = vec3(0.0);
      axis[a] = d >= 0.0 ? 1.0 : -1.0;
      float v = Visibility(probe + axis * (0.75 * uSpacing), i);
      posSum[a] += max(d, 0.0) * v;
      negSum[a] += max(-d, 0.0) * v;
    }
  }
  // batchSum/N divided by the CPU's mean axis weight = batchSum/sum(w).
  // Only the per-batch store rounds to 8 bits (six batches at default quality).
  Positive.rgb += posSum * uPositiveWeight;
  Negative.rgb += negSum * uNegativeWeight;
}
)";

const char* kWorldFill = R"(#version 300 es
precision highp float;
precision highp int;
precision highp sampler3D;
uniform sampler3D uPositive;
uniform sampler3D uNegative;
uniform int uSlice;
layout(location = 0) out vec4 Positive;
layout(location = 1) out vec4 Negative;
void main() {
  ivec3 cell = ivec3(ivec2(gl_FragCoord.xy), uSlice);
  Positive = texelFetch(uPositive, cell, 0);
  Negative = texelFetch(uNegative, cell, 0);
  if (Positive.a > 0.5) return;
  ivec3 size = textureSize(uPositive, 0);
  // All six immediate neighbours are equally near. Copy one, rather than
  // averaging across both sides of a solid wall. Two passes reach two cells.
  for (int a = 0; a < 3; ++a) {
    for (int sign = -1; sign <= 1; sign += 2) {
      ivec3 next = cell;
      next[a] += sign;
      if (any(lessThan(next, ivec3(0))) || any(greaterThanEqual(next, size))) continue;
      vec4 candidate = texelFetch(uPositive, next, 0);
      if (candidate.a > 0.5) {
        Positive = candidate;
        Negative = texelFetch(uNegative, next, 0);
        return;
      }
    }
  }
}
)";

const char* kComposite = R"(#version 300 es
precision highp float;
precision highp sampler2D;
in vec2 vUV;
uniform sampler2D uScene;
uniform sampler2D uNormal;
uniform sampler2D uDepth;
uniform sampler2D uOcclusion;
uniform mat4 uInverseViewProjection;
uniform vec2 uPaneSize;  // Pane size in pixels.
uniform vec2 uUvScale;   // Pane size / target texture size.
uniform int uBlackHot;
uniform float uStrength;  // Occlusion-to-darkening multiplier.
uniform int uBlurRadius;  // Bilateral blur half-width in pixels.
uniform int uBaked;       // Look shading up from the world bake instead.
uniform highp sampler3D uWorldPositive;
uniform highp sampler3D uWorldNegative;
uniform vec3 uWorldOrigin;
uniform vec3 uWorldSize;
uniform float uWorldSpacing;
uniform int uContactCount;
uniform vec4 uContacts[12];  // World center, radius; visible units only.
out vec4 FragColor;

// PR #178: orientation-aware lookup works at eye level and below a deck,
// independent of the camera. Figures receive no self-shading.
float BakedOcclusion(vec3 p, vec3 n) {
  vec3 q = p + n * (0.5 * uWorldSpacing);
  vec3 uvw = (q - uWorldOrigin) / uWorldSize;
  vec3 positive = texture(uWorldPositive, uvw).rgb;
  vec3 negative = texture(uWorldNegative, uvw).rgb;
  float ao = 1.0 - dot(n * n, mix(negative, positive, step(vec3(0.0), n)));
  for (int i = 0; i < uContactCount; ++i) {
    vec3 L = uContacts[i].xyz - p;
    float d2 = max(dot(L, L), 0.0001);
    float occ = (uContacts[i].w * uContacts[i].w / d2) * max(dot(n, L * inversesqrt(d2)), 0.0);
    ao += clamp(occ, 0.0, 1.0);
  }
  return clamp(ao, 0.0, 1.0);
}

vec3 WorldPosition(vec2 uv, float depth) {
  vec4 p = uInverseViewProjection * vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
  return p.xyz / p.w;
}

void main() {
  float depth = texture(uDepth, vUV * uUvScale).r;
  vec3 rgb = texture(uScene, vUV * uUvScale).rgb;
  if (depth < 1.0) {
    vec3 p = WorldPosition(vUV, depth);
    vec4 normalSample = texture(uNormal, vUV * uUvScale);
    vec3 n = normalize(normalSample.xyz * 2.0 - 1.0);
    // Alpha 0 marks a surface drawn in its own color (team-colored figures):
    // it keeps its chroma and, being absent from the bake, gets no baked
    // self-shading (see the lit shader's WorldNormal output).
    bool keepsColor = normalSample.a < 0.5;
    float ao;
    if (uBaked != 0) {
      ao = keepsColor ? 0.0 : BakedOcclusion(p, n);
    } else {
    vec2 texel = 1.0 / uPaneSize;
    float total = 0.0, weightSum = 0.0;
    // Bilateral blur removes sampling grain without smearing contact
    // shading across roof silhouettes, thin parapets or distant surfaces.
    for (int y = -uBlurRadius; y <= uBlurRadius; ++y) {
      for (int x = -uBlurRadius; x <= uBlurRadius; ++x) {
        vec2 uv = clamp(vUV + vec2(x, y) * texel, texel * 0.5, 1.0 - texel * 0.5);
        float d = texture(uDepth, uv * uUvScale).r;
        if (d >= 1.0) continue;
        vec3 q = WorldPosition(uv, d);
        vec3 sampleNormal = normalize(texture(uNormal, uv * uUvScale).xyz * 2.0 - 1.0);
        float weight = pow(max(dot(n, sampleNormal), 0.0), 8.0) *
                       exp(-abs(dot(q - p, n)) * 20.0) / float(1 + x*x + y*y);
        total += texture(uOcclusion, uv * uUvScale).r * weight;
        weightSum += weight;
      }
    }
    ao = total / max(weightSum, 0.001);
    }
    float shade = 1.0 - min(ao * uStrength, 0.78);
    if (keepsColor) {
      // Contact shading darkens a figure's color in both polarities.
      rgb *= shade;
    } else {
      // Work in black-hot space so switching polarity inverts the complete
      // gray image, including contact shading.
      float black = uBlackHot != 0 ? rgb.r : 1.0 - rgb.r;
      black *= shade;
      rgb = vec3(uBlackHot != 0 ? black : 1.0 - black);
    }
  }
  FragColor = vec4(rgb, 1.0);
  // Preserve opaque depth for see-through buildings and gameplay overlays.
  // Writing it avoids format-dependent depth blits to the browser canvas.
  gl_FragDepth = depth;
}
)";

void BindTexture(GLuint texture, int unit, GLenum target = GL_TEXTURE_2D) {
  glActiveTexture(GL_TEXTURE0 + unit);
  glBindTexture(target, texture);
}

void Allocate(GLuint& texture, GLint format, GLenum channels, GLenum type, int width, int height) {
  if (!texture) glGenTextures(1, &texture);
  glBindTexture(GL_TEXTURE_2D, texture);
  glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, channels, type, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}
}  // namespace

bool AmbientOcclusion::Init() {
  return occlusionShader_.Compile(kVertex, kOcclusion) &&
         worldOcclusionShader_.Compile(kVertex, kWorldOcclusion) &&
         worldFillShader_.Compile(kVertex, kWorldFill) &&
         compositeShader_.Compile(kVertex, kComposite);
}

void AmbientOcclusion::DestroyWorldBake() {
  const GLuint textures[] = {volume_.positive, volume_.negative};
  glDeleteTextures(2, textures);
  if (worldFbo_) glDeleteFramebuffers(1, &worldFbo_);
  for (auto& mesh : worldMeshes_) mesh.Destroy();
  worldFbo_ = 0;
  volume_ = {};
  worldKey_ = bakeKey_ = 0;
}

void AmbientOcclusion::Destroy() {
  DestroyWorldBake();
  const GLuint textures[] = {color_, normal_, depth_, occlusion_};
  glDeleteTextures(4, textures);
  if (sceneFbo_) glDeleteFramebuffers(1, &sceneFbo_);
  if (occlusionFbo_) glDeleteFramebuffers(1, &occlusionFbo_);
  if (vao_) glDeleteVertexArrays(1, &vao_);
  color_ = normal_ = depth_ = occlusion_ = sceneFbo_ = occlusionFbo_ = vao_ = 0;
  width_ = height_ = allocatedWidth_ = allocatedHeight_ = 0;
}

bool AmbientOcclusion::Begin(int width, int height, float background) {
  // Grow-only: the two panes of a split window can differ by a pixel, and
  // re-specifying four targets twice per frame would be the thermal view's
  // biggest cost. A smaller pane renders into the lower-left corner.
  if (width > allocatedWidth_ || height > allocatedHeight_) {
    const int allocWidth = std::max(width, allocatedWidth_);
    const int allocHeight = std::max(height, allocatedHeight_);
    if (!sceneFbo_) {
      glGenFramebuffers(1, &sceneFbo_);
      glGenFramebuffers(1, &occlusionFbo_);
      glGenVertexArrays(1, &vao_);
    }
    glActiveTexture(GL_TEXTURE0);
    Allocate(color_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, allocWidth, allocHeight);
    Allocate(normal_, GL_RGBA8, GL_RGBA, GL_UNSIGNED_BYTE, allocWidth, allocHeight);
    Allocate(depth_, GL_DEPTH_COMPONENT24, GL_DEPTH_COMPONENT, GL_UNSIGNED_INT, allocWidth,
             allocHeight);
    Allocate(occlusion_, GL_R8, GL_RED, GL_UNSIGNED_BYTE, allocWidth, allocHeight);
    glBindTexture(GL_TEXTURE_2D, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, occlusionFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, occlusion_, 0);
    bool complete = glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, color_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, GL_TEXTURE_2D, normal_, 0);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, depth_, 0);
    const GLenum buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    glDrawBuffers(2, buffers);
    complete = complete && glCheckFramebufferStatus(GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE;
    if (!complete) {
      if (!warned_) {
        std::fprintf(stderr, "Thermal AO framebuffer incomplete; using unoccluded thermal view\n");
        warned_ = true;
      }
      Destroy();
      return false;
    }
    allocatedWidth_ = allocWidth;
    allocatedHeight_ = allocHeight;
  }
  width_ = width;
  height_ = height;
  glBindFramebuffer(GL_FRAMEBUFFER, sceneFbo_);
  glViewport(0, 0, width, height);
  glScissor(0, 0, width, height);
  const GLfloat color[] = {background, background, background, 1.0f};
  const GLfloat normal[] = {0.5f, 1.0f, 0.5f, 1.0f};
  glClearBufferfv(GL_COLOR, 0, color);
  glClearBufferfv(GL_COLOR, 1, normal);
  glClear(GL_DEPTH_BUFFER_BIT);
  return true;
}

namespace {
void AllocateVolume(GLuint texture, const glm::ivec3& dims, const void* pixels) {
  glBindTexture(GL_TEXTURE_3D, texture);
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGBA8, dims.x, dims.y, dims.z, 0,
               GL_RGBA, GL_UNSIGNED_BYTE, pixels);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
}

// Only visit the columns within each solid's bounds. A city has thousands
// of prisms; testing every prism at every 3D probe would dominate the bake.
std::vector<unsigned char> ProbeValidity(const tactics::Scene& scene,
                                        const AmbientOcclusionVolume& volume) {
  const glm::ivec3 dims = volume.dims;
  const glm::vec3 origin = volume.origin;
  const float spacing = volume.spacing;
  std::vector<unsigned char> pixels(static_cast<size_t>(dims.x) * dims.y * dims.z * 4, 0);
  for (size_t i = 3; i < pixels.size(); i += 4) pixels[i] = 255;
  const auto index = [&](int x, int y, int z) {
    return ((static_cast<size_t>(z) * dims.y + y) * dims.x + x) * 4 + 3;
  };
  const auto invalidateColumn = [&](int x, int z, float bottom, float top) {
    const int y0 = std::max(0, static_cast<int>(std::ceil((bottom - origin.y) / spacing - 0.5f)));
    const int y1 = std::min(dims.y - 1,
                          static_cast<int>(std::floor((top - origin.y) / spacing - 0.5f)));
    for (int y = y0; y <= y1; ++y) pixels[index(x, y, z)] = 0;
  };
  for (int z = 0; z < dims.z; ++z) {
    for (int x = 0; x < dims.x; ++x) {
      const float px = origin.x + (x + 0.5f) * spacing;
      const float pz = origin.z + (z + 0.5f) * spacing;
      if (std::abs(px) > scene.mapHalfExtent || std::abs(pz) > scene.mapHalfExtent) continue;
      invalidateColumn(x, z, origin.y, scene.ground.MeshHeightAt(px, pz));
    }
  }
  const auto visitColumns = [&](glm::vec3 lo, glm::vec3 hi, const auto& visit) {
    glm::ivec3 first = glm::max(glm::ivec3(glm::ceil((lo - origin) / spacing - 0.5f)), glm::ivec3(0));
    glm::ivec3 last = glm::min(glm::ivec3(glm::floor((hi - origin) / spacing - 0.5f)), dims - 1);
    for (int z = first.z; z <= last.z; ++z) {
      for (int x = first.x; x <= last.x; ++x) {
        visit(x, z, origin.x + (x + 0.5f) * spacing, origin.z + (z + 0.5f) * spacing);
      }
    }
  };
  for (const auto& obstacle : tactics::SightObstacles(scene)) {
    if (obstacle.navOnly) continue;
    const auto footprint = tactics::ObstacleFootprint(obstacle);
    visitColumns(obstacle.bounds.min, obstacle.bounds.max, [&](int x, int z, float px, float pz) {
      if (tactics::PointInConvexPolygon({px, pz}, footprint)) {
        invalidateColumn(x, z, obstacle.bounds.min.y, obstacle.bounds.max.y);
      }
    });
  }
  for (const auto& surface : scene.walkSurfaces) {
    if (surface.vertices.size() < 3) continue;
    glm::vec3 lo = surface.vertices.front(), hi = lo;
    for (const auto& p : surface.vertices) { lo = glm::min(lo, p); hi = glm::max(hi, p); }
    visitColumns(lo, hi, [&](int x, int z, float px, float pz) {
      if (tactics::SurfaceContainsXZ(surface, px, pz)) {
        const float top = tactics::SurfaceHeightAt(surface, px, pz);
        invalidateColumn(x, z, top - 0.45f, top);
      }
    });
  }
  return pixels;
}
}  // namespace

bool AmbientOcclusion::EnsureWorldBake(const tactics::Scene& scene, const Shader& depthShader,
                                       const AmbientOcclusionSettings& settings) {
  const uint64_t sceneKey = WorldSceneFingerprint(scene);
  uint64_t key = sceneKey;
  const auto mix = [&key](const auto& value) {
    const auto* p = reinterpret_cast<const unsigned char*>(&value);
    for (size_t i = 0; i < sizeof(value); ++i) key = (key ^ p[i]) * 1099511628211ull;
  };
  mix(settings.radius); mix(settings.quality);
  mix(settings.bake.directions); mix(settings.bake.depthResolution);
  mix(settings.bake.batchLayers); mix(settings.bake.spacing);
  if (volume_.positive && key == bakeKey_) return true;

  const auto started = std::chrono::steady_clock::now();
  const float radius = std::max(settings.radius, 0.01f);
  const int directions = settings.bake.directions > 0
      ? std::clamp(settings.bake.directions, 16, 256)
      : std::clamp(settings.quality, 1, 16) * 16;
  GLint maxResolution = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxResolution);
  const int resolution = std::clamp(settings.bake.depthResolution, 16, maxResolution);
  const int layers = std::clamp(settings.bake.batchLayers, 1, 16);

  // Include the full map with a four-unit XZ margin and normal-lookup room
  // above/below all ground, obstacle and deck heights (including massing).
  float bottom = -0.05f, top = 0.0f;
  for (float h : scene.ground.heights) { bottom = std::min(bottom, h); top = std::max(top, h); }
  for (const auto& obstacle : tactics::SightObstacles(scene)) {
    if (obstacle.navOnly) continue;
    bottom = std::min(bottom, obstacle.bounds.min.y);
    top = std::max(top, obstacle.bounds.max.y);
  }
  for (const auto& surface : scene.walkSurfaces) {
    for (const auto& p : surface.vertices) {
      bottom = std::min(bottom, p.y - 0.45f); top = std::max(top, p.y);
    }
  }
  const float extent = scene.mapHalfExtent + 4.0f;
  AmbientOcclusionVolume next;
  next.origin = {-extent, bottom - 2.0f, -extent};
  const glm::vec3 span(2.0f * extent, top - bottom + 4.0f, 2.0f * extent);
  next.spacing = std::max({settings.bake.spacing, 0.1f, span.x / 256.0f,
                           span.y / 64.0f, span.z / 256.0f});
  next.dims = glm::min(glm::ivec3(glm::ceil(span / next.spacing)), glm::ivec3(256, 64, 256));
  const glm::vec3 center = next.origin + glm::vec3(next.dims) * (0.5f * next.spacing);
  const float R = glm::length(glm::vec3(next.dims) * (0.5f * next.spacing));
  std::printf("Thermal AO bake (PR #178): %d directions, %dx%d depth, %d layers, radius %.2f; "
              "grid %dx%dx%d, spacing %.3f (requested %.3f); center (%.2f, %.2f, %.2f), R %.2f\n",
              directions, resolution, resolution, layers, radius, next.dims.x, next.dims.y,
              next.dims.z, next.spacing, settings.bake.spacing, center.x, center.y, center.z, R);
  std::fflush(stdout);

  if (!worldFbo_) {
    glGenFramebuffers(1, &worldFbo_);
    for (auto& mesh : worldMeshes_) mesh.Init();
  }
  if (!volume_.positive || sceneKey != worldKey_) {
    BuildWorldSceneMeshes(scene, &worldMeshes_[0], &worldMeshes_[1], &worldMeshes_[2]);
    worldKey_ = sceneKey;
  }
  const auto validity = ProbeValidity(scene, next);
  std::vector<glm::vec3> direction(directions);
  std::vector<glm::mat4> vp(directions);
  glm::vec3 positiveWeight(0.0f), negativeWeight(0.0f);
  const glm::mat4 projection = glm::ortho(-R, R, -R, R, 0.0f, 2.0f * R);
  for (int i = 0; i < directions; ++i) {
    const float y = 1.0f - 2.0f * (i + 0.5f) / directions;
    const float angle = i * 2.39996323f;  // Fibonacci sphere, both hemispheres.
    const float r = std::sqrt(std::max(1.0f - y * y, 0.0f));
    const glm::vec3 d(r * std::cos(angle), y, r * std::sin(angle));
    direction[i] = d;
    positiveWeight += glm::max(d, glm::vec3(0.0f));
    negativeWeight += glm::max(-d, glm::vec3(0.0f));
    const glm::vec3 ad = glm::abs(d);
    glm::vec3 up(0.0f);
    up[ad.x <= ad.y && ad.x <= ad.z ? 0 : (ad.y <= ad.z ? 1 : 2)] = 1.0f;
    vp[i] = projection * glm::lookAt(center + d * R, center, up);
  }

  GLuint maps = 0, ping[2][2] = {};
  glGenTextures(1, &maps);
  glGenTextures(2, ping[0]);
  glGenTextures(2, ping[1]);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D_ARRAY, maps);
  glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_DEPTH_COMPONENT32F, resolution, resolution, layers,
               0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D_ARRAY, GL_TEXTURE_COMPARE_MODE, GL_NONE);
  for (int p = 0; p < 2; ++p) {
    for (GLuint texture : ping[p]) AllocateVolume(texture, next.dims, p == 0 ? validity.data() : nullptr);
  }
  glBindTexture(GL_TEXTURE_3D, 0);
  glBindTexture(GL_TEXTURE_2D_ARRAY, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, worldFbo_);
  glDisable(GL_BLEND);
  glDisable(GL_CULL_FACE);
  glDisable(GL_STENCIL_TEST);
  const GLboolean dither = glIsEnabled(GL_DITHER);
  glDisable(GL_DITHER);  // Deterministic 8-bit rounding across batches.
  bool complete = true;
  int read = 0;
  const GLenum buffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
  const auto drawSlices = [&](const Shader& shader, int write) {
    glViewport(0, 0, next.dims.x, next.dims.y);
    glScissor(0, 0, next.dims.x, next.dims.y);
    glDisable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE);
    glBindVertexArray(vao_);
    BindTexture(ping[read][0], 1, GL_TEXTURE_3D);
    BindTexture(ping[read][1], 2, GL_TEXTURE_3D);
    shader.SetInt("uPositive", 1);
    shader.SetInt("uNegative", 2);
    for (int z = 0; z < next.dims.z; ++z) {
      glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, ping[write][0], 0, z);
      glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, ping[write][1], 0, z);
      glDrawBuffers(2, buffers);
      if (z == 0 && glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        complete = false;
        return;
      }
      shader.SetInt("uSlice", z);
      glDrawArrays(GL_TRIANGLES, 0, 3);
      RecordDraw(3, 1);
    }
    read = write;
  };
  for (int batch = 0; batch < directions && complete; batch += layers) {
    const int count = std::min(layers, directions - batch);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 0, 0, 0);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, 0, 0, 0);
    const GLenum none = GL_NONE;
    glDrawBuffers(1, &none);
    glReadBuffer(GL_NONE);
    // A depth attachment must never also be bound for sampling in WebGL2.
    BindTexture(0, 0, GL_TEXTURE_2D_ARRAY);
    glViewport(0, 0, resolution, resolution);
    glScissor(0, 0, resolution, resolution);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    // PR #178: use the exit surface toward the receiver. Entry depth on
    // the far side of a thick building measures its thickness as empty
    // distance and erases finite-radius contact at its near wall.
    glFrontFace(GL_CW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(1.0f, 1.0f);
    depthShader.Use();
    for (int i = 0; i < count; ++i) {
      glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, maps, 0, i);
      if (i == 0 && glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
        complete = false;
        break;
      }
      glClear(GL_DEPTH_BUFFER_BIT);
      depthShader.SetMat4("uLightMVP", vp[batch + i]);
      worldMeshes_[0].Draw();
      // Heightfields/urban tiers are open underneath: keep both sides of
      // their terrain faces so a neighbouring hill can still occlude sky.
      glDisable(GL_CULL_FACE);
      worldMeshes_[1].Draw();
      worldMeshes_[2].Draw();
      glEnable(GL_CULL_FACE);
    }
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDisable(GL_CULL_FACE);
    glFrontFace(GL_CCW);
    glCullFace(GL_BACK);
    glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, 0, 0, 0);
    if (!complete) break;
    BindTexture(maps, 0, GL_TEXTURE_2D_ARRAY);
    worldOcclusionShader_.Use();
    worldOcclusionShader_.SetInt("uDepthMaps", 0);
    worldOcclusionShader_.SetInt("uCount", count);
    worldOcclusionShader_.SetVec3("uOrigin", next.origin);
    worldOcclusionShader_.SetFloat("uSpacing", next.spacing);
    worldOcclusionShader_.SetFloat("uDiameter", 2.0f * R);
    worldOcclusionShader_.SetFloat("uRadius", radius);
    worldOcclusionShader_.SetVec3("uPositiveWeight", 1.0f / positiveWeight);
    worldOcclusionShader_.SetVec3("uNegativeWeight", 1.0f / negativeWeight);
    for (int i = 0; i < count; ++i) {
      const std::string index = "[" + std::to_string(i) + "]";
      worldOcclusionShader_.SetMat4(("uVP" + index).c_str(), vp[batch + i]);
      worldOcclusionShader_.SetVec3(("uDirection" + index).c_str(), direction[batch + i]);
    }
    drawSlices(worldOcclusionShader_, 1 - read);
  }
  if (complete) {
    worldFillShader_.Use();
    for (int pass = 0; pass < 2 && complete; ++pass) drawSlices(worldFillShader_, 1 - read);
  }
  // Detach before returning these textures to the composite's samplers.
  glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, 0, 0, 0);
  glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, 0, 0, 0);
  BindTexture(0, 0, GL_TEXTURE_2D_ARRAY);
  BindTexture(0, 1, GL_TEXTURE_3D);
  BindTexture(0, 2, GL_TEXTURE_3D);
  if (dither) glEnable(GL_DITHER);
  glDeleteTextures(1, &maps);
  glDeleteTextures(2, ping[1 - read]);
  if (!complete) {
    glDeleteTextures(2, ping[read]);
    std::fprintf(stderr, "Thermal AO volume framebuffer incomplete; using screen-space AO\n");
    return false;
  }
  glDeleteTextures(1, &volume_.positive);
  glDeleteTextures(1, &volume_.negative);
  next.positive = ping[read][0];
  next.negative = ping[read][1];
  next.bakeCount = volume_.bakeCount + 1;
  volume_ = next;
  bakeKey_ = key;
  // Synchronous by design; report completed GPU work, not submission time.
  glFinish();
  const double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - started).count();
  std::printf("Thermal AO bake complete: %.1f ms; %.1f MiB volume, %.1f MiB transient depth\n",
              ms, double(next.dims.x) * next.dims.y * next.dims.z * 8 / 1048576.0,
              double(resolution) * resolution * layers * 4 / 1048576.0);
  return true;
}

void AmbientOcclusion::Composite(GLuint target, int x, int y, const glm::mat4& view,
                                 const glm::mat4& projection, bool blackHot,
                                 const AmbientOcclusionSettings& settings,
                                 const tactics::Scene* world, const Shader* depthShader,
                                 const std::vector<glm::vec4>& contactSpheres) {
  const glm::mat4 inverse = glm::inverse(projection * view);
  const glm::vec2 paneSize(static_cast<float>(width_), static_cast<float>(height_));
  const glm::vec2 uvScale(paneSize.x / static_cast<float>(allocatedWidth_),
                          paneSize.y / static_cast<float>(allocatedHeight_));
  const float radius = std::max(settings.radius, 0.01f);
  const int quality = std::max(settings.quality, 1);
  const bool baked =
      settings.baked && world && depthShader && EnsureWorldBake(*world, *depthShader, settings);
  glBindVertexArray(vao_);

  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  BindTexture(depth_, 0);
  BindTexture(normal_, 1);
  if (!baked) {
    glBindFramebuffer(GL_FRAMEBUFFER, occlusionFbo_);
    glViewport(0, 0, width_, height_);
    glScissor(0, 0, width_, height_);
    occlusionShader_.Use();
    occlusionShader_.SetInt("uDepth", 0);
    occlusionShader_.SetInt("uNormal", 1);
    occlusionShader_.SetMat4("uInverseViewProjection", inverse);
    occlusionShader_.SetMat4("uProjection", projection);
    occlusionShader_.SetMat4("uView", view);
    occlusionShader_.SetVec2("uPaneSize", paneSize);
    occlusionShader_.SetVec2("uUvScale", uvScale);
    occlusionShader_.SetFloat("uRadius", radius);
    occlusionShader_.SetInt("uSteps", quality);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    RecordDraw(3, 1);
  }

  glBindFramebuffer(GL_FRAMEBUFFER, target);
  glViewport(x, y, width_, height_);
  glScissor(x, y, width_, height_);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_ALWAYS);
  BindTexture(color_, 2);
  BindTexture(occlusion_, 3);
  if (baked) {
    BindTexture(volume_.positive, 4, GL_TEXTURE_3D);
    BindTexture(volume_.negative, 5, GL_TEXTURE_3D);
  }
  compositeShader_.Use();
  compositeShader_.SetInt("uDepth", 0);
  compositeShader_.SetInt("uNormal", 1);
  compositeShader_.SetInt("uScene", 2);
  compositeShader_.SetInt("uOcclusion", 3);
  compositeShader_.SetInt("uWorldPositive", 4);
  compositeShader_.SetInt("uWorldNegative", 5);
  compositeShader_.SetInt("uBaked", baked ? 1 : 0);
  if (baked) {
    compositeShader_.SetVec3("uWorldOrigin", volume_.origin);
    compositeShader_.SetVec3("uWorldSize", glm::vec3(volume_.dims) * volume_.spacing);
    compositeShader_.SetFloat("uWorldSpacing", volume_.spacing);
    const int count = std::min(static_cast<int>(contactSpheres.size()), 12);
    compositeShader_.SetInt("uContactCount", count);
    for (int i = 0; i < count; ++i) {
      const std::string name = "uContacts[" + std::to_string(i) + "]";
      compositeShader_.SetVec4(name.c_str(), contactSpheres[i]);
    }
  }
  compositeShader_.SetInt("uBlackHot", blackHot ? 1 : 0);
  compositeShader_.SetMat4("uInverseViewProjection", inverse);
  compositeShader_.SetVec2("uPaneSize", paneSize);
  compositeShader_.SetVec2("uUvScale", uvScale);
  compositeShader_.SetFloat("uStrength", std::max(settings.strength, 0.0f));
  compositeShader_.SetInt("uBlurRadius", std::max(settings.softness, 0));
  glDrawArrays(GL_TRIANGLES, 0, 3);
  RecordDraw(3, 1);
  glBindVertexArray(0);
  glDepthFunc(GL_LESS);
  // No AO attachments stay bound when the next pane starts capturing.
  for (int unit = 5; unit >= 0; --unit) {
    BindTexture(0, unit, GL_TEXTURE_3D);
    BindTexture(0, unit);
  }
}

}  // namespace gfx
