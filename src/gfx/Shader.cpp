#include "gfx/Shader.h"

#include <cstdio>
#include <vector>

namespace gfx {
namespace {

GLuint CompileStage(GLenum stage, const std::string& src) {
  GLuint shader = glCreateShader(stage);
  const char* srcPtr = src.c_str();
  glShaderSource(shader, 1, &srcPtr, nullptr);
  glCompileShader(shader);

  GLint ok = GL_FALSE;
  glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    GLint logLength = 0;
    glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &logLength);
    std::vector<char> log(logLength + 1, '\0');
    glGetShaderInfoLog(shader, logLength, nullptr, log.data());
    std::fprintf(stderr, "Shader compile error: %s\n", log.data());
    glDeleteShader(shader);
    return 0;
  }
  return shader;
}

}  // namespace

Shader::~Shader() {
  if (program_) glDeleteProgram(program_);
}

bool Shader::Compile(const std::string& vertexSrc, const std::string& fragmentSrc) {
  GLuint vs = CompileStage(GL_VERTEX_SHADER, vertexSrc);
  GLuint fs = CompileStage(GL_FRAGMENT_SHADER, fragmentSrc);
  if (!vs || !fs) return false;

  program_ = glCreateProgram();
  glAttachShader(program_, vs);
  glAttachShader(program_, fs);
  glLinkProgram(program_);

  GLint ok = GL_FALSE;
  glGetProgramiv(program_, GL_LINK_STATUS, &ok);
  glDeleteShader(vs);
  glDeleteShader(fs);
  if (!ok) {
    GLint logLength = 0;
    glGetProgramiv(program_, GL_INFO_LOG_LENGTH, &logLength);
    std::vector<char> log(logLength + 1, '\0');
    glGetProgramInfoLog(program_, logLength, nullptr, log.data());
    std::fprintf(stderr, "Program link error: %s\n", log.data());
    glDeleteProgram(program_);
    program_ = 0;
    return false;
  }
  return true;
}

void Shader::Use() const { glUseProgram(program_); }

void Shader::SetMat4(const char* name, const glm::mat4& value) const {
  const GLint loc = glGetUniformLocation(program_, name);
  glUniformMatrix4fv(loc, 1, GL_FALSE, &value[0][0]);
}

void Shader::SetVec4(const char* name, const glm::vec4& value) const {
  const GLint loc = glGetUniformLocation(program_, name);
  glUniform4fv(loc, 1, &value[0]);
}

void Shader::SetVec3(const char* name, const glm::vec3& value) const {
  const GLint loc = glGetUniformLocation(program_, name);
  glUniform3fv(loc, 1, &value[0]);
}

void Shader::SetInt(const char* name, int value) const {
  const GLint loc = glGetUniformLocation(program_, name);
  glUniform1i(loc, value);
}

}  // namespace gfx
