#pragma once

#include <string>

#include <GLES3/gl3.h>
#include <glm/glm.hpp>

namespace gfx {

// Minimal GLSL ES 3.0 shader program wrapper.
class Shader {
 public:
  Shader() = default;
  ~Shader();

  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  bool Compile(const std::string& vertexSrc, const std::string& fragmentSrc);
  void Use() const;

  void SetMat4(const char* name, const glm::mat4& value) const;
  void SetVec4(const char* name, const glm::vec4& value) const;
  void SetVec3(const char* name, const glm::vec3& value) const;
  void SetInt(const char* name, int value) const;

  GLuint Program() const { return program_; }

 private:
  GLuint program_ = 0;
};

}  // namespace gfx
