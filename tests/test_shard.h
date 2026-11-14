// Shared runner for multi-test binaries: --list, --only NAME, --shard i/N,
// --timings. Sharding is derived at runtime from the registered list
// (index % N), so adding a test needs no CI/CMake edits.
#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

struct TestCase {
  const char* name;
  void (*fn)();
};
#define TBG_TEST(fn) \
  TestCase { #fn, fn }

// Returns false on usage error. Runs selected tests (all by default).
inline bool RunSelectedTests(int argc, char** argv, const std::vector<TestCase>& cases) {
  int shardIndex = 0, shardCount = 1;
  const char* only = nullptr;
  bool list = false, timings = false;
  for (int i = 1; i < argc; ++i) {
    if (!std::strcmp(argv[i], "--list")) list = true;
    else if (!std::strcmp(argv[i], "--timings")) timings = true;
    else if (!std::strcmp(argv[i], "--only") && i + 1 < argc) only = argv[++i];
    else if (!std::strcmp(argv[i], "--shard") && i + 1 < argc) {
      if (std::sscanf(argv[++i], "%d/%d", &shardIndex, &shardCount) != 2 ||
          shardCount < 1 || shardIndex < 0 || shardIndex >= shardCount) {
        std::fprintf(stderr, "bad --shard (want i/N, 0 <= i < N)\n");
        return false;
      }
    } else {
      std::fprintf(stderr, "usage: %s [--list] [--only NAME] [--shard i/N] [--timings]\n", argv[0]);
      return false;
    }
  }
  if (list) {
    for (const auto& c : cases) std::printf("%s\n", c.name);
    std::exit(0);
  }
  bool matched = only == nullptr;
  for (size_t i = 0; i < cases.size(); ++i) {
    if (only && std::strcmp(only, cases[i].name) != 0) continue;
    if (!only && static_cast<int>(i % shardCount) != shardIndex) continue;
    matched = true;
    auto t0 = std::chrono::steady_clock::now();
    cases[i].fn();
    if (timings) {
      double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
      std::fprintf(stderr, "TIMING %8.2fs %s\n", s, cases[i].name);
    }
  }
  if (!matched) { std::fprintf(stderr, "no test named %s\n", only); return false; }
  return true;
}
