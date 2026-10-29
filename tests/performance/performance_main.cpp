// Headless renderer performance suite (issue #202).
//
// Renders the same two-pane, six-figure urban scene as the native app for a
// committed set of seeds. Startup work is warmed out before a fixed sample
// window. The JSON output intentionally contains every sample as well as
// summaries so hosted-runner variance remains visible instead of becoming a
// brittle pass/fail threshold.

#include <SDL.h>
#include <GLES3/gl3.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <sys/utsname.h>

#include "game/GameLogic.h"
#include "game/MapGenerator.h"
#include "game/Visibility.h"
#include "gfx/Camera.h"
#include "gfx/SceneRenderer.h"

namespace fs = std::filesystem;

namespace {

constexpr int kWindowWidth = 1280;
constexpr int kWindowHeight = 720;
constexpr int kPaneCount = 2;
constexpr int kRequiredSeedCount = 10;

struct Options {
  fs::path seedsPath = "tests/performance/seeds.txt";
  fs::path outputPath = "performance-results.json";
  int warmupFrames = 30;
  int sampleFrames = 120;
  std::string commit;
  std::string targetRef;
  std::string runId;
  std::string timestamp;
};

struct Sample {
  double fps = 0.0;
  double frameMs = 0.0;
  double fovMs = 0.0;
  int drawCalls = 0;
  long long triangles = 0;
  long long vertices = 0;
};

struct MapResult {
  uint32_t seed = 0;
  std::vector<Sample> samples;
};

struct Summary {
  double minimum = 0.0;
  double p05 = 0.0;
  double median = 0.0;
  double p95 = 0.0;
  double maximum = 0.0;
  double mean = 0.0;
};

void Usage() {
  std::fprintf(stderr,
               "usage: tactics_performance_suite [--seeds FILE] [--output FILE] "
               "[--warmup-frames N] [--sample-frames N] [--commit SHA] "
               "[--target-ref REF] [--run-id ID] [--timestamp ISO8601]\n");
}

bool PositiveInt(const char* text, int* value) {
  try {
    size_t consumed = 0;
    const int parsed = std::stoi(text, &consumed);
    if (consumed != std::string(text).size() || parsed <= 0) return false;
    *value = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool ParseOptions(int argc, char** argv, Options* options) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    const auto value = [&]() -> const char* {
      if (i + 1 >= argc) return nullptr;
      return argv[++i];
    };
    if (arg == "--seeds") {
      const char* v = value();
      if (!v) return false;
      options->seedsPath = v;
    } else if (arg == "--output") {
      const char* v = value();
      if (!v) return false;
      options->outputPath = v;
    } else if (arg == "--warmup-frames") {
      const char* v = value();
      if (!v || !PositiveInt(v, &options->warmupFrames)) return false;
    } else if (arg == "--sample-frames") {
      const char* v = value();
      if (!v || !PositiveInt(v, &options->sampleFrames)) return false;
    } else if (arg == "--commit") {
      const char* v = value();
      if (!v) return false;
      options->commit = v;
    } else if (arg == "--target-ref") {
      const char* v = value();
      if (!v) return false;
      options->targetRef = v;
    } else if (arg == "--run-id") {
      const char* v = value();
      if (!v) return false;
      options->runId = v;
    } else if (arg == "--timestamp") {
      const char* v = value();
      if (!v) return false;
      options->timestamp = v;
    } else {
      return false;
    }
  }
  return true;
}

bool LoadSeeds(const fs::path& path, std::vector<uint32_t>* seeds) {
  std::ifstream input(path);
  if (!input) {
    std::fprintf(stderr, "could not open seed list: %s\n", path.string().c_str());
    return false;
  }
  std::string line;
  int lineNumber = 0;
  while (std::getline(input, line)) {
    ++lineNumber;
    const size_t first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') continue;
    const size_t last = line.find_last_not_of(" \t\r");
    const std::string value = line.substr(first, last - first + 1);
    try {
      size_t consumed = 0;
      const unsigned long long parsed = std::stoull(value, &consumed);
      if (consumed != value.size() || parsed > 0xffffffffULL) throw std::out_of_range("seed");
      seeds->push_back(static_cast<uint32_t>(parsed));
    } catch (...) {
      std::fprintf(stderr, "invalid seed at %s:%d: %s\n", path.string().c_str(), lineNumber,
                   value.c_str());
      return false;
    }
  }
  if (seeds->size() != kRequiredSeedCount) {
    std::fprintf(stderr, "performance suite requires exactly %d seeds; found %zu in %s\n",
                 kRequiredSeedCount, seeds->size(), path.string().c_str());
    return false;
  }
  return true;
}

std::string JsonString(const std::string& value) {
  std::ostringstream out;
  out << '"';
  for (const unsigned char c : value) {
    switch (c) {
      case '"': out << "\\\""; break;
      case '\\': out << "\\\\"; break;
      case '\b': out << "\\b"; break;
      case '\f': out << "\\f"; break;
      case '\n': out << "\\n"; break;
      case '\r': out << "\\r"; break;
      case '\t': out << "\\t"; break;
      default:
        if (c < 0x20) {
          out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<int>(c) << std::dec << std::setfill(' ');
        } else {
          out << c;
        }
    }
  }
  out << '"';
  return out.str();
}

std::string GlString(GLenum name) {
  const GLubyte* value = glGetString(name);
  return value ? reinterpret_cast<const char*>(value) : "unknown";
}

std::string Environment(const char* name) {
  const char* value = std::getenv(name);
  return value ? value : "";
}

std::string CpuModel() {
  std::ifstream cpuInfo("/proc/cpuinfo");
  std::string line;
  while (std::getline(cpuInfo, line)) {
    const std::string key = "model name";
    if (line.compare(0, key.size(), key) != 0) continue;
    const size_t colon = line.find(':');
    if (colon == std::string::npos) break;
    const size_t value = line.find_first_not_of(" \t", colon + 1);
    return value == std::string::npos ? "" : line.substr(value);
  }
  return "";
}

double Percentile(const std::vector<double>& sorted, double percentile) {
  if (sorted.empty()) return 0.0;
  const double index = percentile * static_cast<double>(sorted.size() - 1);
  const size_t lower = static_cast<size_t>(std::floor(index));
  const size_t upper = static_cast<size_t>(std::ceil(index));
  const double fraction = index - static_cast<double>(lower);
  return sorted[lower] + (sorted[upper] - sorted[lower]) * fraction;
}

Summary Summarize(std::vector<double> values) {
  if (values.empty()) return {};
  const double total = std::accumulate(values.begin(), values.end(), 0.0);
  std::sort(values.begin(), values.end());
  return {values.front(), Percentile(values, 0.05), Percentile(values, 0.50),
          Percentile(values, 0.95), values.back(), total / static_cast<double>(values.size())};
}

using SampleValue = std::function<double(const Sample&)>;

Summary SummarizeSamples(const std::vector<Sample>& samples, const SampleValue& value) {
  std::vector<double> values;
  values.reserve(samples.size());
  for (const Sample& sample : samples) values.push_back(value(sample));
  return Summarize(std::move(values));
}

void WriteSummary(std::ostream& out, const Summary& summary, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  out << "{\n"
      << pad << "  \"min\": " << summary.minimum << ",\n"
      << pad << "  \"p05\": " << summary.p05 << ",\n"
      << pad << "  \"p50\": " << summary.median << ",\n"
      << pad << "  \"p95\": " << summary.p95 << ",\n"
      << pad << "  \"max\": " << summary.maximum << ",\n"
      << pad << "  \"mean\": " << summary.mean << "\n"
      << pad << "}";
}

void WriteAggregates(std::ostream& out, const std::vector<Sample>& samples, int indent) {
  const std::string pad(static_cast<size_t>(indent), ' ');
  const struct Metric {
    const char* name;
    SampleValue value;
  } metrics[] = {
      {"fps", [](const Sample& s) { return s.fps; }},
      {"frame_ms", [](const Sample& s) { return s.frameMs; }},
      {"fov_ms", [](const Sample& s) { return s.fovMs; }},
      {"draw_calls", [](const Sample& s) { return static_cast<double>(s.drawCalls); }},
      {"triangles", [](const Sample& s) { return static_cast<double>(s.triangles); }},
      {"vertices", [](const Sample& s) { return static_cast<double>(s.vertices); }},
  };
  out << "{\n";
  for (size_t i = 0; i < std::size(metrics); ++i) {
    out << pad << "  " << JsonString(metrics[i].name) << ": ";
    WriteSummary(out, SummarizeSamples(samples, metrics[i].value), indent + 2);
    out << (i + 1 == std::size(metrics) ? "\n" : ",\n");
  }
  out << pad << "}";
}

bool WriteResults(const Options& options, const std::vector<uint32_t>& seeds,
                  const std::vector<MapResult>& maps, const std::string& platform,
                  const std::string& glVendor, const std::string& glRenderer,
                  const std::string& glVersion) {
  if (!options.outputPath.parent_path().empty()) {
    std::error_code error;
    fs::create_directories(options.outputPath.parent_path(), error);
    if (error) {
      std::fprintf(stderr, "could not create output directory: %s\n", error.message().c_str());
      return false;
    }
  }
  std::ofstream out(options.outputPath);
  if (!out) {
    std::fprintf(stderr, "could not write results: %s\n", options.outputPath.string().c_str());
    return false;
  }
  std::vector<Sample> allSamples;
  for (const MapResult& map : maps) {
    allSamples.insert(allSamples.end(), map.samples.begin(), map.samples.end());
  }
  SDL_version sdlVersion;
  SDL_GetVersion(&sdlVersion);
  out << std::fixed << std::setprecision(4);
  out << "{\n"
      << "  \"schema_version\": 1,\n"
      << "  \"metadata\": {\n"
      << "    \"commit\": " << JsonString(options.commit) << ",\n"
      << "    \"target_ref\": " << JsonString(options.targetRef) << ",\n"
      << "    \"run_id\": " << JsonString(options.runId) << ",\n"
      << "    \"timestamp\": " << JsonString(options.timestamp) << "\n"
      << "  },\n"
      << "  \"configuration\": {\n"
      << "    \"map_type\": \"urban-auto\",\n"
      << "    \"feature_modes\": \"auto\",\n"
      << "    \"seed_file\": " << JsonString(options.seedsPath.generic_string()) << ",\n"
      << "    \"warmup_frames\": " << options.warmupFrames << ",\n"
      << "    \"sample_frames\": " << options.sampleFrames << ",\n"
      << "    \"pane_count\": " << kPaneCount << ",\n"
      << "    \"window_width\": " << kWindowWidth << ",\n"
      << "    \"window_height\": " << kWindowHeight << ",\n"
      << "    \"fov_overlay\": \"shadow_map\",\n"
      << "    \"fog_active\": true,\n"
      << "    \"seeds\": [";
  for (size_t i = 0; i < seeds.size(); ++i) out << (i ? ", " : "") << seeds[i];
  out << "]\n"
      << "  },\n"
      << "  \"environment\": {\n"
      << "    \"platform\": " << JsonString(platform) << ",\n"
      << "    \"runner_os\": " << JsonString(Environment("RUNNER_OS")) << ",\n"
      << "    \"runner_arch\": " << JsonString(Environment("RUNNER_ARCH")) << ",\n"
      << "    \"runner_name\": " << JsonString(Environment("RUNNER_NAME")) << ",\n"
      << "    \"runner_image_os\": " << JsonString(Environment("ImageOS")) << ",\n"
      << "    \"runner_image_version\": " << JsonString(Environment("ImageVersion")) << ",\n"
      << "    \"cpu_model\": " << JsonString(CpuModel()) << ",\n"
      << "    \"cpu_threads\": " << std::thread::hardware_concurrency() << ",\n"
      << "    \"video_driver\": "
      << JsonString(SDL_GetCurrentVideoDriver() ? SDL_GetCurrentVideoDriver() : "") << ",\n"
      << "    \"software_gl_requested\": " << JsonString(Environment("LIBGL_ALWAYS_SOFTWARE"))
      << ",\n"
      << "    \"gl_vendor\": " << JsonString(glVendor) << ",\n"
      << "    \"gl_renderer\": " << JsonString(glRenderer) << ",\n"
      << "    \"gl_version\": " << JsonString(glVersion) << ",\n"
      << "    \"sdl_version\": " << JsonString(std::to_string(sdlVersion.major) + "." +
                                                   std::to_string(sdlVersion.minor) + "." +
                                                   std::to_string(sdlVersion.patch)) << ",\n"
#ifdef NDEBUG
      << "    \"build_type\": \"Release\",\n"
#else
      << "    \"build_type\": \"Debug\",\n"
#endif
      << "    \"swap_interval\": " << SDL_GL_GetSwapInterval() << "\n"
      << "  },\n"
      << "  \"metrics\": [\"fps\", \"frame_ms\", \"fov_ms\", \"draw_calls\", "
         "\"triangles\", \"vertices\"],\n"
      << "  \"aggregate\": ";
  WriteAggregates(out, allSamples, 2);
  out << ",\n  \"maps\": [\n";
  for (size_t mapIndex = 0; mapIndex < maps.size(); ++mapIndex) {
    const MapResult& map = maps[mapIndex];
    out << "    {\n"
        << "      \"seed\": " << map.seed << ",\n"
        << "      \"sample_count\": " << map.samples.size() << ",\n"
        << "      \"aggregate\": ";
    WriteAggregates(out, map.samples, 6);
    out << ",\n      \"samples\": [\n";
    for (size_t i = 0; i < map.samples.size(); ++i) {
      const Sample& sample = map.samples[i];
      out << "        {\"frame\": " << i + 1 << ", \"fps\": " << sample.fps
          << ", \"frame_ms\": " << sample.frameMs << ", \"fov_ms\": " << sample.fovMs
          << ", \"draw_calls\": " << sample.drawCalls << ", \"triangles\": "
          << sample.triangles << ", \"vertices\": " << sample.vertices << "}"
          << (i + 1 == map.samples.size() ? "\n" : ",\n");
    }
    out << "      ]\n    }" << (mapIndex + 1 == maps.size() ? "\n" : ",\n");
  }
  out << "  ]\n}\n";
  return static_cast<bool>(out);
}

Sample RenderFrame(gfx::SceneRenderer* renderer, const tactics::GameLogic& game,
                   const gfx::OrbitCamera* cameras, bool collectStats) {
  std::array<tactics::TeamVisibility, kPaneCount> visibility;
  glFinish();
  const Uint64 frameStart = SDL_GetPerformanceCounter();
  const Uint64 fovStart = frameStart;
  visibility[0] = game.ComputeVisibility(tactics::Team::Blue);
  visibility[1] = game.ComputeVisibility(tactics::Team::Red);
  const Uint64 fovEnd = SDL_GetPerformanceCounter();

  gfx::RenderFrameStats stats;
  gfx::RenderDebugOptions debug;
  debug.stats = collectStats ? &stats : nullptr;
  // Benchmark the default AO view, like the live app.
  debug.thermal = true;
  const tactics::Team teams[kPaneCount] = {tactics::Team::Blue, tactics::Team::Red};
  for (int pane = 0; pane < kPaneCount; ++pane) {
    renderer->RenderPane(game, teams[pane], /*fogActive=*/true, visibility[pane], cameras[pane],
                         pane * (kWindowWidth / kPaneCount), 0, kWindowWidth / kPaneCount,
                         kWindowHeight, {}, 0, debug);
  }
  SDL_GL_SwapWindow(SDL_GL_GetCurrentWindow());
  glFinish();
  const Uint64 frameEnd = SDL_GetPerformanceCounter();
  const double frequency = static_cast<double>(SDL_GetPerformanceFrequency());
  const double frameMs = 1000.0 * static_cast<double>(frameEnd - frameStart) / frequency;
  return {frameMs > 0.0 ? 1000.0 / frameMs : 0.0,
          frameMs,
          1000.0 * static_cast<double>(fovEnd - fovStart) / frequency,
          stats.drawCalls,
          stats.triangles,
          stats.vertices};
}

}  // namespace

int main(int argc, char** argv) {
  Options options;
  if (!ParseOptions(argc, argv, &options)) {
    Usage();
    return 2;
  }
  std::vector<uint32_t> seeds;
  if (!LoadSeeds(options.seedsPath, &seeds)) return 2;

  if (SDL_Init(SDL_INIT_VIDEO) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
  SDL_SetHint(SDL_HINT_VIDEO_X11_FORCE_EGL, "1");
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
  SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
  SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
  SDL_Window* window = SDL_CreateWindow("tbgwaf performance suite", SDL_WINDOWPOS_CENTERED,
                                        SDL_WINDOWPOS_CENTERED, kWindowWidth, kWindowHeight,
                                        SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN);
  if (!window) {
    std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_GLContext context = SDL_GL_CreateContext(window);
  if (!context) {
    std::fprintf(stderr, "SDL_GL_CreateContext failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  SDL_GL_SetSwapInterval(0);
  int drawableWidth = 0, drawableHeight = 0;
  SDL_GL_GetDrawableSize(window, &drawableWidth, &drawableHeight);
  if (drawableWidth != kWindowWidth || drawableHeight != kWindowHeight) {
    std::fprintf(stderr, "drawable size %dx%d != required %dx%d\n", drawableWidth,
                 drawableHeight, kWindowWidth, kWindowHeight);
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }

  const std::string glVendor = GlString(GL_VENDOR);
  const std::string glRenderer = GlString(GL_RENDERER);
  const std::string glVersion = GlString(GL_VERSION);
  struct utsname systemInfo {};
  const std::string platform = uname(&systemInfo) == 0
                                   ? std::string(systemInfo.sysname) + " " + systemInfo.release +
                                         " " + systemInfo.machine
                                   : "unknown";

  gfx::SceneRenderer renderer;
  if (!renderer.Init()) {
    SDL_GL_DeleteContext(context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 1;
  }
  renderer.SetFovOverlayMode(gfx::FovOverlayMode::ShadowMap);

  std::vector<MapResult> results;
  results.reserve(seeds.size());
  for (size_t mapIndex = 0; mapIndex < seeds.size(); ++mapIndex) {
    const uint32_t seed = seeds[mapIndex];
    tactics::GameLogic game(tactics::GenerateUrbanMap(seed));
    gfx::OrbitCamera cameras[kPaneCount];
    for (gfx::OrbitCamera& camera : cameras) camera.FitToExtent(game.GetScene().mapHalfExtent);

    for (int frame = 0; frame < options.warmupFrames; ++frame) {
      SDL_PumpEvents();
      RenderFrame(&renderer, game, cameras, /*collectStats=*/false);
    }
    MapResult result;
    result.seed = seed;
    result.samples.reserve(static_cast<size_t>(options.sampleFrames));
    for (int frame = 0; frame < options.sampleFrames; ++frame) {
      SDL_PumpEvents();
      result.samples.push_back(RenderFrame(&renderer, game, cameras, /*collectStats=*/true));
    }
    const Summary fps = SummarizeSamples(result.samples, [](const Sample& s) { return s.fps; });
    const Summary frame =
        SummarizeSamples(result.samples, [](const Sample& s) { return s.frameMs; });
    std::fprintf(stderr, "[%zu/%zu] seed %u: %.1f FPS, %.2f ms (p50)\n", mapIndex + 1,
                 seeds.size(), seed, fps.median, frame.median);
    results.push_back(std::move(result));
  }

  const bool wrote = WriteResults(options, seeds, results, platform, glVendor, glRenderer, glVersion);
  renderer.Destroy();
  SDL_GL_DeleteContext(context);
  SDL_DestroyWindow(window);
  SDL_Quit();
  if (!wrote) return 1;
  std::fprintf(stderr, "wrote %s\n", options.outputPath.string().c_str());
  return 0;
}
