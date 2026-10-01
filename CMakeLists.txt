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

# Pure game-logic sources: no SDL/GL/ImGui dependency, so they can be linked
# into both the playable app and the headless logic test binary.
set(GAME_LOGIC_SOURCES
  src/game/Scene.cpp
  src/game/MapGenerator.cpp
  src/game/NavMesh.cpp
  src/game/Raycast.cpp
  src/game/Visibility.cpp
  src/game/GameLogic.cpp
)

add_library(tactics_logic STATIC ${GAME_LOGIC_SOURCES})
target_include_directories(tactics_logic PUBLIC src)
target_link_libraries(tactics_logic PUBLIC glm::glm)

set(APP_GFX_SOURCES
  src/main.cpp
  src/gfx/Shader.cpp
  src/gfx/Mesh.cpp
  src/gfx/Camera.cpp
  src/gfx/SceneRenderer.cpp
)

if(EMSCRIPTEN)
  # Stage D: WASM/WebGL2 build, deployed to GitHub Pages. Emscripten ships
  # its own SDL2 "port" and GL emulation, so this skips the native
  # pkg-config SDL2/EGL/GLESV2 discovery entirely and instead compiles
  # ImGui's sources directly into the app (rather than reusing the native
  # `imgui` static-lib target, which is linked against those pkg-config
  # libraries).
  add_executable(tactics_app_web
    ${APP_GFX_SOURCES}
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_sdl2.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )
  target_include_directories(tactics_app_web PRIVATE
    src
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )
  target_compile_definitions(tactics_app_web PRIVATE IMGUI_IMPL_OPENGL_ES3)
  target_link_libraries(tactics_app_web PRIVATE tactics_logic glm::glm)

  # -sUSE_SDL=2 must apply at compile time too (it changes where SDL
  # headers are found); the rest are link-only settings.
  target_compile_options(tactics_app_web PRIVATE -sUSE_SDL=2)
  # MODULARIZE: the page (web/index.html) instantiates the module twice --
  # one client per team, each with its own canvas -- so the output is a
  # factory-function .js instead of an emcc-generated HTML page.
  target_link_options(tactics_app_web PRIVATE
    -sUSE_SDL=2
    -sUSE_WEBGL2=1
    -sFULL_ES3=1
    -sALLOW_MEMORY_GROWTH=1
    -sNO_EXIT_RUNTIME=1
    -sMODULARIZE=1
    -sEXPORT_NAME=createTbgwafModule
  )
  set_target_properties(tactics_app_web PROPERTIES
    OUTPUT_NAME "tactics_app"
    SUFFIX ".js"
  )
  # The page is static; keep a copy next to the build output so the build
  # dir can be served as-is for local testing.
  add_custom_command(
    OUTPUT ${CMAKE_BINARY_DIR}/index.html
    COMMAND ${CMAKE_COMMAND} -E copy
            ${CMAKE_SOURCE_DIR}/web/index.html ${CMAKE_BINARY_DIR}/index.html
    DEPENDS ${CMAKE_SOURCE_DIR}/web/index.html
  )
  add_custom_target(tactics_web_page ALL DEPENDS ${CMAKE_BINARY_DIR}/index.html)
else()
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

  # SDL-free GL rendering code (issue #14 stage 2): shared between the
  # interactive app and the headless visual scenario runner so both draw the
  # per-team fog-of-war panes through the exact same code path.
  add_library(tactics_gfx STATIC
    src/gfx/Shader.cpp
    src/gfx/Mesh.cpp
    src/gfx/Camera.cpp
    src/gfx/SceneRenderer.cpp
  )
  target_include_directories(tactics_gfx PUBLIC src)
  target_link_libraries(tactics_gfx PUBLIC tactics_logic PkgConfig::GLESV2 glm::glm)

  add_executable(tactics_app src/main.cpp)
  target_include_directories(tactics_app PRIVATE src)
  target_link_libraries(tactics_app PRIVATE
    tactics_logic
    tactics_gfx
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

  add_executable(tactics_map_generator_tests tests/map_generator_tests.cpp)
  target_include_directories(tactics_map_generator_tests PRIVATE src)
  target_link_libraries(tactics_map_generator_tests PRIVATE tactics_logic)
  add_test(NAME map_generator_tests COMMAND tactics_map_generator_tests)

  add_executable(tactics_camera_tests tests/camera_tests.cpp src/gfx/Camera.cpp)
  target_include_directories(tactics_camera_tests PRIVATE src)
  target_link_libraries(tactics_camera_tests PRIVATE glm::glm)
  add_test(NAME camera_tests COMMAND tactics_camera_tests)

  # YAML gameplay scenarios (issue #14): a native-only, test-only dependency,
  # so it's kept out of tactics_logic (which the WASM build also links) to
  # avoid cross-compiling yaml-cpp for Emscripten.
  set(YAML_CPP_BUILD_TESTS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_TOOLS OFF CACHE BOOL "" FORCE)
  set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG 0.8.0
  )
  FetchContent_MakeAvailable(yaml-cpp)

  add_library(tactics_scenario STATIC tests/scenario/Scenario.cpp)
  target_include_directories(tactics_scenario PUBLIC src tests)
  target_link_libraries(tactics_scenario PUBLIC tactics_logic yaml-cpp::yaml-cpp)

  add_executable(tactics_scenario_tests tests/scenario_tests.cpp)
  target_include_directories(tactics_scenario_tests PRIVATE src tests)
  target_link_libraries(tactics_scenario_tests PRIVATE tactics_scenario)
  add_test(
    NAME scenario_tests
    COMMAND tactics_scenario_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  # Visual scenario runner (issue #14 stages 2+3): PNG golden capture/diff
  # and per-team scenario videos. stb supplies single-header PNG read/write;
  # header-only, so only its source dir is needed (no CMakeLists to add).
  FetchContent_Declare(
    stb
    GIT_REPOSITORY https://github.com/nothings/stb.git
    GIT_TAG 2c980bb59875b0d32144a71867fbdebb2f77cd20
  )
  FetchContent_MakeAvailable(stb)

  add_executable(tactics_visual_tests tests/visual/visual_main.cpp)
  target_include_directories(tactics_visual_tests PRIVATE src tests ${stb_SOURCE_DIR})
  target_link_libraries(tactics_visual_tests PRIVATE
    tactics_scenario
    tactics_gfx
    PkgConfig::SDL2
    PkgConfig::EGL
  )

  # Map-only golden renders of seeded generated cities (no units/teams).
  add_executable(tactics_map_visual_tests tests/visual/map_visual_main.cpp)
  target_include_directories(tactics_map_visual_tests PRIVATE src tests ${stb_SOURCE_DIR})
  target_link_libraries(tactics_map_visual_tests PRIVATE
    tactics_logic
    tactics_gfx
    PkgConfig::SDL2
    PkgConfig::EGL
  )

  find_program(XVFB_RUN_EXECUTABLE xvfb-run)
  if(XVFB_RUN_EXECUTABLE)
    # Golden-screenshot regression run (no --video: ffmpeg isn't needed for
    # the ctest pass, only for the PR review page built in CI).
    add_test(
      NAME visual_tests
      COMMAND ${XVFB_RUN_EXECUTABLE} --auto-servernum --server-args=-screen\ 0\ 1280x1024x24
              $<TARGET_FILE:tactics_visual_tests> --out-dir ${CMAKE_BINARY_DIR}/visual_out
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    set_tests_properties(visual_tests PROPERTIES
      TIMEOUT 300
      ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1;SDL_VIDEODRIVER=x11"
    )
    add_test(
      NAME map_visual_tests
      COMMAND ${XVFB_RUN_EXECUTABLE} --auto-servernum --server-args=-screen\ 0\ 1280x1024x24
              $<TARGET_FILE:tactics_map_visual_tests> --out-dir ${CMAKE_BINARY_DIR}/map_visual_out
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
    set_tests_properties(map_visual_tests PROPERTIES
      TIMEOUT 300
      ENVIRONMENT "LIBGL_ALWAYS_SOFTWARE=1;SDL_VIDEODRIVER=x11"
    )
  endif()
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
endif()
