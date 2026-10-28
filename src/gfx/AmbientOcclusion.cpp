#include "gfx/AmbientOcclusion.h"

#include <algorithm>
#include <cstdio>

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
out vec4 FragColor;

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
    // Alpha 0 marks a surface drawn in its own color (team-colored figures).
    bool keepsColor = normalSample.a < 0.5;
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
    float ao = total / max(weightSum, 0.001);
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
         compositeShader_.Compile(kVertex, kComposite);
}

void AmbientOcclusion::Destroy() {
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

void AmbientOcclusion::Composite(GLuint target, int x, int y, const glm::mat4& view,
                                 const glm::mat4& projection, bool blackHot,
                                 const AmbientOcclusionSettings& settings) {
  const glm::mat4 inverse = glm::inverse(projection * view);
  const glm::vec2 paneSize(static_cast<float>(width_), static_cast<float>(height_));
  const glm::vec2 uvScale(paneSize.x / static_cast<float>(allocatedWidth_),
                          paneSize.y / static_cast<float>(allocatedHeight_));
  glBindFramebuffer(GL_FRAMEBUFFER, occlusionFbo_);
  glViewport(0, 0, width_, height_);
  glScissor(0, 0, width_, height_);
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  BindTexture(depth_, 0);
  BindTexture(normal_, 1);
  occlusionShader_.Use();
  occlusionShader_.SetInt("uDepth", 0);
  occlusionShader_.SetInt("uNormal", 1);
  occlusionShader_.SetMat4("uInverseViewProjection", inverse);
  occlusionShader_.SetMat4("uProjection", projection);
  occlusionShader_.SetMat4("uView", view);
  occlusionShader_.SetVec2("uPaneSize", paneSize);
  occlusionShader_.SetVec2("uUvScale", uvScale);
  occlusionShader_.SetFloat("uRadius", std::max(settings.radius, 0.01f));
  occlusionShader_.SetInt("uSteps", std::max(settings.quality, 1));
  glBindVertexArray(vao_);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  RecordDraw(3, 1);

  glBindFramebuffer(GL_FRAMEBUFFER, target);
  glViewport(x, y, width_, height_);
  glScissor(x, y, width_, height_);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glDepthFunc(GL_ALWAYS);
  BindTexture(color_, 2);
  BindTexture(occlusion_, 3);
  compositeShader_.Use();
  compositeShader_.SetInt("uDepth", 0);
  compositeShader_.SetInt("uNormal", 1);
  compositeShader_.SetInt("uScene", 2);
  compositeShader_.SetInt("uOcclusion", 3);
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
  for (int unit = 3; unit >= 0; --unit) BindTexture(0, unit);
}

}  // namespace gfx
