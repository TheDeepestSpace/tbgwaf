cmake_minimum_required(VERSION 3.16)
project(tbgwaf CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

include(FetchContent)
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)
pkg_check_modules(GLESV2 REQUIRED IMPORTED_TARGET glesv2)

add_executable(tactics_app src/main.cpp)
target_link_libraries(tactics_app PRIVATE
  PkgConfig::SDL2
  PkgConfig::EGL
  PkgConfig::GLESV2
  glm::glm
)

enable_testing()
find_program(XVFB_RUN_EXECUTABLE xvfb-run)
if(XVFB_RUN_EXECUTABLE)
  add_test(
    NAME smoke_test
    COMMAND ${XVFB_RUN_EXECUTABLE} --auto-servernum --server-args=-screen\ 0\ 1280x1024x24
            $<TARGET_FILE:tactics_app>
  )
else()
  add_test(NAME smoke_test COMMAND tactics_app)
endif()
set_tests_properties(smoke_test PROPERTIES
  TIMEOUT 30
  ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1;SDL_VIDEODRIVER=x11"
)
