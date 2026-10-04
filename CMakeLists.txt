cmake_minimum_required(VERSION 3.16)
project(tbgwaf CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build selection (native only; the Emscripten build is always the web client).
#   TBGWAF_BUILD_CLIENT: SDL2/GLES desktop app + GL-based tests. Needs the
#     SDL2/EGL/GLESv2 dev packages; turn OFF for a headless/server-only build.
#   TBGWAF_BUILD_SERVER: the authoritative WebSocket server (POSIX sockets).
option(TBGWAF_BUILD_CLIENT "Build the SDL2/GLES client app and its GL tests" ON)
option(TBGWAF_BUILD_SERVER "Build the WebSocket game server" ON)
# Default WebSocket server for the WASM page (empty = in-page two-canvas demo
# unless ?server=ws://host:port is given in the page URL).
set(TBGWAF_SERVER_URL "" CACHE STRING "Default server URL baked into the web page")

include(FetchContent)
FetchContent_Declare(
  glm
  GIT_REPOSITORY https://github.com/g-truc/glm.git
  GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

if(EMSCRIPTEN OR TBGWAF_BUILD_CLIENT)
  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.9
  )
  FetchContent_MakeAvailable(imgui)
endif()

# App flow (Splash -> Map Select -> gameplay): flow/app_flow.yaml is the source
# of truth. Build-time only -- a host Python script emits the C++ state enum +
# transition table (so no YAML parser is linked into the app/WASM) and a
# Mermaid diagram, persisted as flow/app_flow.mmd (committed; plus .svg via
# mermaid-cli (mmdc) when available).
find_package(Python3 REQUIRED COMPONENTS Interpreter)
set(FLOW_YAML ${CMAKE_SOURCE_DIR}/flow/app_flow.yaml)
set(FLOW_TEMPLATES ${CMAKE_SOURCE_DIR}/flow/templates)
set(FLOW_GEN_DIR ${CMAKE_BINARY_DIR}/generated)
# The diagram is persisted next to the YAML that declares it (committed).
set(FLOW_MMD ${CMAKE_SOURCE_DIR}/flow/app_flow.mmd)
set(FLOW_SVG ${CMAKE_SOURCE_DIR}/flow/app_flow.svg)
add_custom_command(
  OUTPUT ${FLOW_GEN_DIR}/AppFlow.h ${FLOW_MMD}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${FLOW_GEN_DIR}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/scripts/gen_flow.py ${FLOW_YAML}
          --header ${FLOW_GEN_DIR}/AppFlow.h --mermaid ${FLOW_MMD}
          --templates ${FLOW_TEMPLATES}
  DEPENDS ${FLOW_YAML} ${CMAKE_SOURCE_DIR}/scripts/gen_flow.py
          ${FLOW_TEMPLATES}/AppFlow.h.j2 ${FLOW_TEMPLATES}/app_flow.mmd.j2
)
add_custom_target(app_flow_gen DEPENDS ${FLOW_GEN_DIR}/AppFlow.h ${FLOW_MMD})
find_program(MMDC_EXECUTABLE mmdc)
if(MMDC_EXECUTABLE)
  add_custom_command(
    OUTPUT ${FLOW_SVG}
    COMMAND ${MMDC_EXECUTABLE} -i ${FLOW_MMD} -o ${FLOW_SVG}
    DEPENDS ${FLOW_MMD}
  )
  add_custom_target(app_flow_diagram ALL DEPENDS ${FLOW_SVG})
else()
  message(STATUS "mmdc (mermaid-cli) not found: skipping app_flow.svg (Mermaid source still generated)")
endif()

# Pure game-logic sources: no SDL/GL/ImGui dependency, so they can be linked
# into both the playable app and the headless logic test binary.
set(GAME_LOGIC_SOURCES
  src/game/Scene.cpp
  src/game/Geometry.cpp
  src/game/MapGenerator.cpp
  src/game/NavMesh.cpp
  src/game/Raycast.cpp
  src/game/Visibility.cpp
  src/game/GameLogic.cpp
)

add_library(tactics_logic STATIC ${GAME_LOGIC_SOURCES})
target_include_directories(tactics_logic PUBLIC src)
target_link_libraries(tactics_logic PUBLIC glm::glm)

# Wire protocol + client-side protocol logic (JSON, discrete actions, remote
# mirror): shared by the server, the WASM client, and the headless tests.
add_library(tactics_net STATIC
  src/net/Json.cpp
  src/net/Protocol.cpp
  src/net/ActionApply.cpp
  src/net/RemoteClient.cpp
  src/net/SceneSpec.cpp
)
target_include_directories(tactics_net PUBLIC src)
target_link_libraries(tactics_net PUBLIC tactics_logic glm::glm)

set(APP_GFX_SOURCES
  src/main.cpp
  src/ui/Hud.cpp
  src/ui/Menu.cpp
  src/gfx/Shader.cpp
  src/gfx/Mesh.cpp
  src/gfx/Camera.cpp
  src/gfx/FigureRig.cpp
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
  target_include_directories(tactics_app_web PRIVATE ${FLOW_GEN_DIR})
  add_dependencies(tactics_app_web app_flow_gen)
  target_link_libraries(tactics_app_web PRIVATE tactics_logic tactics_net glm::glm)

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
  # Keep a copy of the page next to the build output so the build dir can be
  # served as-is for local testing. TBGWAF_SERVER_URL is substituted in.
  configure_file(${CMAKE_SOURCE_DIR}/web/index.html ${CMAKE_BINARY_DIR}/index.html @ONLY)

  # Asset & animation gallery (issue #126): a second, much smaller WASM
  # module for web/gallery.html. No ImGui and no game logic -- just the
  # shared figure/weapon rig and the gallery renderer; the page supplies
  # the item list and playback UI through the exported C API.
  add_executable(tactics_gallery_web
    src/gallery/gallery_main.cpp
    src/gallery/GalleryScene.cpp
    src/gfx/Shader.cpp
    src/gfx/Mesh.cpp
    src/gfx/FigureRig.cpp
  )
  target_include_directories(tactics_gallery_web PRIVATE src)
  target_link_libraries(tactics_gallery_web PRIVATE glm::glm)
  target_compile_options(tactics_gallery_web PRIVATE -sUSE_SDL=2)
  target_link_options(tactics_gallery_web PRIVATE
    -sUSE_SDL=2
    -sUSE_WEBGL2=1
    -sFULL_ES3=1
    -sALLOW_MEMORY_GROWTH=1
    -sNO_EXIT_RUNTIME=1
    -sMODULARIZE=1
    -sEXPORT_NAME=createTbgwafGalleryModule
    -sEXPORTED_RUNTIME_METHODS=ccall
  )
  set_target_properties(tactics_gallery_web PROPERTIES
    OUTPUT_NAME "tactics_gallery"
    SUFFIX ".js"
  )
  configure_file(${CMAKE_SOURCE_DIR}/web/gallery.html ${CMAKE_BINARY_DIR}/gallery.html COPYONLY)
  # Map-seed goldens (issue #133): the gallery's "Map seeds" section is
  # driven by whatever PNGs sit under tests/map_goldens/. The script globs
  # at build time, so re-run it every build and a new golden shows up in
  # the gallery without touching the page.
  add_custom_target(gallery_map_seeds ALL
    COMMAND ${CMAKE_COMMAND}
            -DGOLDENS_DIR=${CMAKE_SOURCE_DIR}/tests/map_goldens
            -DOUT_DIR=${CMAKE_BINARY_DIR}
            -P ${CMAKE_SOURCE_DIR}/scripts/gen_gallery_map_seeds.cmake
    COMMENT "Collecting map-seed goldens for the gallery")
else()
  enable_testing()

  # ---- Headless targets (no SDL2/EGL/GLESv2 needed) ----

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
  target_link_libraries(tactics_scenario PUBLIC tactics_logic tactics_net yaml-cpp::yaml-cpp)

  add_executable(tactics_scenario_tests tests/scenario_tests.cpp)
  target_include_directories(tactics_scenario_tests PRIVATE src tests)
  target_link_libraries(tactics_scenario_tests PRIVATE tactics_scenario)
  add_test(
    NAME scenario_tests
    COMMAND tactics_scenario_tests
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  add_executable(tactics_net_tests tests/net_tests.cpp)
  target_link_libraries(tactics_net_tests PRIVATE tactics_net)
  add_test(NAME net_tests COMMAND tactics_net_tests)

  if(TBGWAF_BUILD_SERVER)
    add_library(tactics_server_lib STATIC
      src/server/GameSession.cpp
      src/server/Lobby.cpp
      src/server/WebSocket.cpp
      src/server/Server.cpp
    )
    target_include_directories(tactics_server_lib PUBLIC src)
    target_link_libraries(tactics_server_lib PUBLIC tactics_net tactics_logic)

    add_executable(tbgwaf_server src/server/server_main.cpp)
    target_link_libraries(tbgwaf_server PRIVATE tactics_server_lib)

    # Same server with the loopback-only test control tap compiled in (never
    # shipped): drives the networked scenario tests.
    add_executable(tbgwaf_server_testctl src/server/server_main.cpp)
    target_compile_definitions(tbgwaf_server_testctl PRIVATE TBGWAF_TEST_CONTROL)
    target_link_libraries(tbgwaf_server_testctl PRIVATE tactics_server_lib)

    add_executable(tactics_net_scenario_tests tests/net_scenario_tests.cpp tests/net/WsClient.cpp)
    target_include_directories(tactics_net_scenario_tests PRIVATE src tests tests/net)
    target_link_libraries(tactics_net_scenario_tests PRIVATE tactics_scenario tactics_server_lib)
    add_dependencies(tactics_net_scenario_tests tbgwaf_server_testctl)
    add_test(
      NAME net_scenario_tests
      COMMAND tactics_net_scenario_tests $<TARGET_FILE:tbgwaf_server_testctl> tests/scenarios
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )

    add_executable(tactics_server_tests tests/server_tests.cpp)
    target_link_libraries(tactics_server_tests PRIVATE tactics_server_lib)
    add_test(NAME server_tests COMMAND tactics_server_tests)
  endif()

  if(TBGWAF_BUILD_CLIENT)
    # ---- Client app + GL-dependent tests ----
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
      src/gfx/FigureRig.cpp
      src/gfx/SceneRenderer.cpp
    )
    target_include_directories(tactics_gfx PUBLIC src)
    target_link_libraries(tactics_gfx PUBLIC tactics_logic PkgConfig::GLESV2 glm::glm)

    # Per-team HUD (Turn panel, action menu, team label), shared by the
    # interactive app and the visual scenario runner.
    add_library(tactics_ui STATIC src/ui/Hud.cpp src/ui/Menu.cpp)
    target_include_directories(tactics_ui PUBLIC src ${FLOW_GEN_DIR})
    add_dependencies(tactics_ui app_flow_gen)
    target_link_libraries(tactics_ui PUBLIC tactics_logic tactics_gfx imgui glm::glm)

    # Asset & animation gallery (issue #126): shared catalog/renderer, a
    # native interactive viewer.
    add_library(tactics_gallery STATIC src/gallery/GalleryScene.cpp)
    target_include_directories(tactics_gallery PUBLIC src)
    target_link_libraries(tactics_gallery PUBLIC tactics_gfx glm::glm)

    add_executable(tactics_gallery_app src/gallery/gallery_main.cpp)
    target_include_directories(tactics_gallery_app PRIVATE src)
    target_link_libraries(tactics_gallery_app PRIVATE
      tactics_gallery
      PkgConfig::SDL2
      PkgConfig::EGL
      PkgConfig::GLESV2
    )

    add_executable(tactics_app src/main.cpp)
    target_include_directories(tactics_app PRIVATE src ${FLOW_GEN_DIR})
    add_dependencies(tactics_app app_flow_gen)
    target_link_libraries(tactics_app PRIVATE
      tactics_logic
      tactics_net
      tactics_gfx
      tactics_ui
      imgui
      PkgConfig::SDL2
      PkgConfig::EGL
      PkgConfig::GLESV2
      glm::glm
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
      tactics_ui
      imgui
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
endif()
