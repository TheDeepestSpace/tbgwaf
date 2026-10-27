#include "gfx/AmbientOcclusion.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "gfx/Mesh.h"

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

// Horizon occlusion on the baked top-down heightmap (texel = world cell).
// Same obscurance model as the screen-space pass with an up-facing normal,
// but with fixed directions: nothing here depends on the camera.
const char* kWorldOcclusion = R"(#version 300 es
precision highp float;
precision highp int;
precision highp sampler2D;
uniform sampler2D uHeight;
uniform float uTexel;   // World size of one texel.
uniform float uRadius;  // World-space reach of the shading.
uniform int uSteps;
uniform int uRes;
out vec4 FragColor;

void main() {
  ivec2 c = ivec2(gl_FragCoord.xy);
  float h = texelFetch(uHeight, c, 0).r;
  float occlusion = 0.0;
  for (int direction = 0; direction < 16; ++direction) {
    float angle = (float(direction) + 0.5) * 0.39269908;
    vec2 dir = vec2(cos(angle), sin(angle));
    float horizon = 0.0;
    for (int step = 0; step < uSteps; ++step) {
      float fraction = (float(step) + 0.5) / float(uSteps);
      float reach = uRadius * fraction * fraction;
      ivec2 tc = c + ivec2(floor(dir * reach / uTexel + 0.5));
      if (tc == c || any(lessThan(tc, ivec2(0))) || any(greaterThanEqual(tc, ivec2(uRes)))) continue;
      float rise = texelFetch(uHeight, tc, 0).r - h;
      if (rise <= 0.0) continue;
      // A heightmap flattens a wall into one tall step, but the screen-space
      // pass sees the wall's lower, nearer points too: score the step at a
      // few heights up its face so tall buildings still shade their base.
      float run = length(vec2(tc - c)) * uTexel;
      for (int k = 0; k < 3; ++k) {
        float dh = min(rise, uRadius * (0.2 + 0.3 * float(k)));
        float distance = length(vec2(run, dh));
        float falloff = 1.0 - smoothstep(0.0, uRadius, distance);
        horizon = max(horizon, max(dh / distance - 0.08, 0.0) * falloff);
      }
    }
    occlusion += horizon;
  }
  FragColor = vec4(vec3(clamp(occlusion / 16.0, 0.0, 1.0)), 1.0);
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
uniform sampler2D uWorldAo;
uniform sampler2D uWorldHeight;
uniform vec2 uWorldOrigin;
uniform float uWorldTexel;
uniform int uWorldRes;
uniform float uWorldRadius;
out vec4 FragColor;

// Shading from the world bake: a depth-aware bilinear lookup at the surface
// point (pushed out along a wall's normal), keeping only texels whose baked
// surface is not above the point, and fading with the point's height over
// that surface so walls and figures darken near their feet.
float BakedOcclusion(vec3 p, vec3 n) {
  vec2 xz = p.xz + n.xz * 0.3;
  vec2 f = (xz - uWorldOrigin) / uWorldTexel - 0.5;
  ivec2 i0 = ivec2(floor(f));
  vec2 t = f - vec2(i0);
  float sum = 0.0, weightSum = 0.0;
  for (int k = 0; k < 4; ++k) {
    ivec2 o = ivec2(k & 1, k >> 1);
    ivec2 tc = i0 + o;
    if (any(lessThan(tc, ivec2(0))) || any(greaterThanEqual(tc, ivec2(uWorldRes)))) continue;
    float rise = p.y - texelFetch(uWorldHeight, tc, 0).r;
    float w = (o.x == 0 ? 1.0 - t.x : t.x) * (o.y == 0 ? 1.0 - t.y : t.y) *
              exp(-max(-rise - 0.1, 0.0) * 8.0);
    sum += w * texelFetch(uWorldAo, tc, 0).r *
           (1.0 - smoothstep(0.0, uWorldRadius, max(rise, 0.0)));
    weightSum += w;
  }
  return sum / max(weightSum, 0.3);
}

vec3 WorldPosition(vec2 uv, float depth) {
  vec4 p = uInverseViewProjection * vec4(uv * 2.0 - 1.0, depth * 2.0 - 1.0, 1.0);
  return p.xyz / p.w;
}

void main() {
  float depth = texture(uDepth, vUV * uUvScale).r;
  float gray = texture(uScene, vUV * uUvScale).r;
  if (depth < 1.0) {
    vec3 p = WorldPosition(vUV, depth);
    vec3 n = normalize(texture(uNormal, vUV * uUvScale).xyz * 2.0 - 1.0);
    float ao;
    if (uBaked != 0) {
      ao = BakedOcclusion(p, n);
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
    // Work in black-hot space so switching polarity inverts the complete
    // image, including contact shading; figures retain their hot signal.
    float black = uBlackHot != 0 ? gray : 1.0 - gray;
    black *= 1.0 - min(ao * uStrength, 0.78);
    gray = uBlackHot != 0 ? black : 1.0 - black;
  }
  FragColor = vec4(vec3(gray), 1.0);
  // Preserve opaque depth for see-through buildings and gameplay overlays.
  // Writing it avoids format-dependent depth blits to the browser canvas.
  gl_FragDepth = depth;
}
)";

void BindTexture(GLuint texture, int unit) {
  glActiveTexture(GL_TEXTURE0 + unit);
  glBindTexture(GL_TEXTURE_2D, texture);
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
         compositeShader_.Compile(kVertex, kComposite);
}

void AmbientOcclusion::DestroyWorldBake() {
  const GLuint textures[] = {worldHeight_, worldAo_};
  glDeleteTextures(2, textures);
  if (worldFbo_) glDeleteFramebuffers(1, &worldFbo_);
  worldHeight_ = worldAo_ = worldFbo_ = 0;
  worldKey_ = 0;
  worldRes_ = worldQuality_ = 0;
  worldRadius_ = 0.0f;
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
uint64_t HashScene(const tactics::Scene& scene) {
  uint64_t hash = 1469598103934665603ull;
  const auto mix = [&hash](float value) {
    uint32_t bits;
    std::memcpy(&bits, &value, sizeof bits);
    hash = (hash ^ bits) * 1099511628211ull;
  };
  mix(scene.mapHalfExtent);
  for (const tactics::Obstacle& o : scene.obstacles) {
    if (o.navOnly) continue;
    mix(o.bounds.min.x); mix(o.bounds.min.y); mix(o.bounds.min.z);
    mix(o.bounds.max.x); mix(o.bounds.max.y); mix(o.bounds.max.z);
    for (const glm::vec2& v : o.footprint) { mix(v.x); mix(v.y); }
  }
  for (const tactics::WalkSurface& surface : scene.walkSurfaces) {
    for (const glm::vec3& v : surface.vertices) { mix(v.x); mix(v.y); mix(v.z); }
  }
  mix(scene.ground.minX); mix(scene.ground.minZ); mix(scene.ground.step);
  mix(static_cast<float>(scene.ground.nx)); mix(static_cast<float>(scene.ground.nz));
  for (float h : scene.ground.heights) mix(h);
  return hash;
}
}  // namespace

bool AmbientOcclusion::EnsureWorldBake(const tactics::Scene& scene, float radius, int quality) {
  const uint64_t key = HashScene(scene);
  const bool geometryChanged = !worldHeight_ || key != worldKey_;
  if (!geometryChanged && radius == worldRadius_ && quality == worldQuality_) return true;

  if (geometryChanged) {
    // Top-down heightmap of everything opaque and static, ~4 texels a unit.
    const float extent = 2.0f * (scene.mapHalfExtent + 2.0f);
    const int res = std::clamp(static_cast<int>(std::ceil(extent * 4.0f)), 64, 2048);
    const float texel = extent / static_cast<float>(res);
    const glm::vec2 origin(-scene.mapHalfExtent - 2.0f);
    std::vector<float> height(static_cast<size_t>(res) * res, 0.0f);
    const auto center = [&](int i, int j) {
      return origin + glm::vec2((static_cast<float>(i) + 0.5f) * texel,
                                (static_cast<float>(j) + 0.5f) * texel);
    };
    const auto range = [&](float lo, float hi, int& a, int& b) {
      a = std::max(static_cast<int>(std::floor(lo)), 0);
      b = std::min(static_cast<int>(std::ceil(hi)), res - 1);
    };
    if (!scene.ground.Empty()) {
      for (int j = 0; j < res; ++j) {
        for (int i = 0; i < res; ++i) {
          const glm::vec2 c = center(i, j);
          height[static_cast<size_t>(j) * res + i] = scene.ground.MeshHeightAt(c.x, c.y);
        }
      }
    }
    for (const tactics::Obstacle& o : scene.obstacles) {
      if (o.navOnly) continue;
      const std::vector<glm::vec2> fp = tactics::ObstacleFootprint(o);
      int i0, i1, j0, j1;
      range((o.bounds.min.x - origin.x) / texel - 0.5f, (o.bounds.max.x - origin.x) / texel - 0.5f,
            i0, i1);
      range((o.bounds.min.z - origin.y) / texel - 0.5f, (o.bounds.max.z - origin.y) / texel - 0.5f,
            j0, j1);
      for (int j = j0; j <= j1; ++j) {
        for (int i = i0; i <= i1; ++i) {
          const glm::vec2 c = center(i, j);
          bool inside = true;
          for (size_t k = 0; k < fp.size() && inside; ++k) {
            const glm::vec2 a = fp[k], b = fp[(k + 1) % fp.size()];
            inside = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) >= -1e-4f;
          }
          if (inside) {
            float& h = height[static_cast<size_t>(j) * res + i];
            h = std::max(h, o.bounds.max.y);
          }
        }
      }
    }
    for (const tactics::WalkSurface& surface : scene.walkSurfaces) {
      const auto& v = surface.vertices;
      for (size_t k = 1; k + 1 < v.size(); ++k) {
        const glm::vec3 a = v[0], b = v[k], c = v[k + 1];
        const float det = (b.z - c.z) * (a.x - c.x) + (c.x - b.x) * (a.z - c.z);
        if (std::abs(det) < 1e-8f) continue;
        int i0, i1, j0, j1;
        range((std::min({a.x, b.x, c.x}) - origin.x) / texel - 0.5f,
              (std::max({a.x, b.x, c.x}) - origin.x) / texel - 0.5f, i0, i1);
        range((std::min({a.z, b.z, c.z}) - origin.y) / texel - 0.5f,
              (std::max({a.z, b.z, c.z}) - origin.y) / texel - 0.5f, j0, j1);
        for (int j = j0; j <= j1; ++j) {
          for (int i = i0; i <= i1; ++i) {
            const glm::vec2 p = center(i, j);
            const float l0 = ((b.z - c.z) * (p.x - c.x) + (c.x - b.x) * (p.y - c.z)) / det;
            const float l1 = ((c.z - a.z) * (p.x - c.x) + (a.x - c.x) * (p.y - c.z)) / det;
            const float l2 = 1.0f - l0 - l1;
            if (l0 < -1e-3f || l1 < -1e-3f || l2 < -1e-3f) continue;
            float& h = height[static_cast<size_t>(j) * res + i];
            h = std::max(h, l0 * a.y + l1 * b.y + l2 * c.y);
          }
        }
      }
    }

    if (!worldFbo_) glGenFramebuffers(1, &worldFbo_);
    glActiveTexture(GL_TEXTURE0);
    if (!worldHeight_) glGenTextures(1, &worldHeight_);
    glBindTexture(GL_TEXTURE_2D, worldHeight_);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_R32F, res, res, 0, GL_RED, GL_FLOAT, height.data());
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    Allocate(worldAo_, GL_R8, GL_RED, GL_UNSIGNED_BYTE, res, res);
    glBindFramebuffer(GL_FRAMEBUFFER, worldFbo_);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, worldAo_, 0);
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE) {
      DestroyWorldBake();
      return false;
    }
    worldKey_ = key;
    worldRes_ = res;
    worldTexel_ = texel;
    worldOrigin_ = origin;
  }

  glBindFramebuffer(GL_FRAMEBUFFER, worldFbo_);
  glViewport(0, 0, worldRes_, worldRes_);
  glScissor(0, 0, worldRes_, worldRes_);
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  BindTexture(worldHeight_, 0);
  glBindVertexArray(vao_);
  worldOcclusionShader_.Use();
  worldOcclusionShader_.SetInt("uHeight", 0);
  worldOcclusionShader_.SetFloat("uTexel", worldTexel_);
  worldOcclusionShader_.SetFloat("uRadius", radius);
  worldOcclusionShader_.SetInt("uSteps", quality);
  worldOcclusionShader_.SetInt("uRes", worldRes_);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  RecordDraw(3, 1);
  worldRadius_ = radius;
  worldQuality_ = quality;
  return true;
}

void AmbientOcclusion::Composite(GLuint target, int x, int y, const glm::mat4& view,
                                 const glm::mat4& projection, bool blackHot,
                                 const AmbientOcclusionSettings& settings,
                                 const tactics::Scene* world) {
  const glm::mat4 inverse = glm::inverse(projection * view);
  const glm::vec2 paneSize(static_cast<float>(width_), static_cast<float>(height_));
  const glm::vec2 uvScale(paneSize.x / static_cast<float>(allocatedWidth_),
                          paneSize.y / static_cast<float>(allocatedHeight_));
  const float radius = std::max(settings.radius, 0.01f);
  const int quality = std::max(settings.quality, 1);
  glBindVertexArray(vao_);
  const bool baked =
      settings.baked && world && EnsureWorldBake(*world, radius, quality);

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
    BindTexture(worldAo_, 4);
    BindTexture(worldHeight_, 5);
  }
  compositeShader_.Use();
  compositeShader_.SetInt("uDepth", 0);
  compositeShader_.SetInt("uNormal", 1);
  compositeShader_.SetInt("uScene", 2);
  compositeShader_.SetInt("uOcclusion", 3);
  compositeShader_.SetInt("uWorldAo", 4);
  compositeShader_.SetInt("uWorldHeight", 5);
  compositeShader_.SetInt("uBaked", baked ? 1 : 0);
  if (baked) {
    compositeShader_.SetVec2("uWorldOrigin", worldOrigin_);
    compositeShader_.SetFloat("uWorldTexel", worldTexel_);
    compositeShader_.SetInt("uWorldRes", worldRes_);
    compositeShader_.SetFloat("uWorldRadius", radius);
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
  for (int unit = 5; unit >= 0; --unit) BindTexture(0, unit);
}

}  // namespace gfx
