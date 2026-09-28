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

FetchContent_Declare(
  imgui
  GIT_REPOSITORY https://github.com/ocornut/imgui.git
  GIT_TAG v1.91.9
)
FetchContent_MakeAvailable(imgui)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
pkg_check_modules(EGL REQUIRED IMPORTED_TARGET egl)
pkg_check_modules(GLESV2 REQUIRED IMPORTED_TARGET glesv2)

add_library(imgui STATIC
  ${imgui_SOURCE_DIR}/imgui.cpp
  ${imgui_SOURCE_DIR}/imgui_draw.cpp
  ${imgui_SOURCE_DIR}/imgui_tables.cpp
  ${imgui_SOURCE_DIR}/imgui_widgets.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
  ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
)
target_include_directories(imgui PUBLIC
  ${imgui_SOURCE_DIR}
  ${imgui_SOURCE_DIR}/backends
)
target_compile_definitions(imgui PUBLIC IMGUI_IMPL_OPENGL_ES3)
target_link_libraries(imgui PUBLIC PkgConfig::SDL2 PkgConfig::GLESV2)

# Pure game-logic sources: no SDL/GL/ImGui dependency, so they can be linked
# into both the playable app and the headless logic test binary.
set(GAME_LOGIC_SOURCES
  src/game/Scene.cpp
  src/game/NavMesh.cpp
  src/game/Raycast.cpp
  src/game/TurnManager.cpp
  src/game/GameLogic.cpp
)

add_library(tactics_logic STATIC ${GAME_LOGIC_SOURCES})
target_include_directories(tactics_logic PUBLIC src)
target_link_libraries(tactics_logic PUBLIC glm::glm)

add_executable(tactics_app
  src/main.cpp
  src/gfx/Shader.cpp
  src/gfx/Mesh.cpp
  src/gfx/Camera.cpp
)
target_include_directories(tactics_app PRIVATE src)
target_link_libraries(tactics_app PRIVATE
  tactics_logic
  imgui
  PkgConfig::SDL2
  PkgConfig::EGL
  PkgConfig::GLESV2
  glm::glm
)

enable_testing()

add_executable(tactics_logic_tests tests/logic_tests.cpp)
target_include_directories(tactics_logic_tests PRIVATE src)
target_link_libraries(tactics_logic_tests PRIVATE tactics_logic)
add_test(NAME logic_tests COMMAND tactics_logic_tests)

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
  ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1;SDL_VIDEODRIVER=x11;TBGWAF_SMOKE_TEST=1"
)
