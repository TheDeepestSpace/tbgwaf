// Runs every YAML gameplay scenario under tests/scenarios/ (or a directory
// passed as argv[1]) through the cheap/logic-only mode described in
// issue #14: load scenario, replay its scripted actions through the same
// GameLogic click/choose API a player uses, and check state-only assertions
// (no rendering). See tests/scenario/Scenario.h for the file format.

#include <algorithm>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <string>
#include <vector>

#include "scenario/Scenario.h"

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  const fs::path scenarioDir = argc > 1 ? fs::path(argv[1]) : fs::path("tests/scenarios");

  if (!fs::is_directory(scenarioDir)) {
    std::fprintf(stderr, "scenario directory not found: %s\n", scenarioDir.string().c_str());
    return 1;
  }

  std::vector<fs::path> files;
  for (const auto& entry : fs::directory_iterator(scenarioDir)) {
    if (!entry.is_regular_file()) continue;
    const auto ext = entry.path().extension();
    if (ext == ".yaml" || ext == ".yml") files.push_back(entry.path());
  }
  std::sort(files.begin(), files.end());

  if (files.empty()) {
    std::fprintf(stderr, "no scenario YAML files found under %s\n", scenarioDir.string().c_str());
    return 1;
  }

  int scenarioFailures = 0;
  for (const auto& file : files) {
    tactics::scenario::Scenario scenario;
    try {
      scenario = tactics::scenario::LoadScenarioFromFile(file.string());
    } catch (const std::exception& e) {
      std::fprintf(stderr, "FAIL %s: %s\n", file.string().c_str(), e.what());
      ++scenarioFailures;
      continue;
    }

    const auto result = tactics::scenario::RunScenario(scenario);
    if (result.Passed()) {
      std::printf("PASS %s (%s)\n", scenario.name.c_str(), file.string().c_str());
    } else {
      std::fprintf(stderr, "FAIL %s (%s):\n", scenario.name.c_str(), file.string().c_str());
      for (const auto& msg : result.failures) {
        std::fprintf(stderr, "  %s\n", msg.c_str());
      }
      ++scenarioFailures;
    }
  }

  if (scenarioFailures == 0) {
    std::printf("All %zu scenario(s) passed.\n", files.size());
    return 0;
  }
  std::fprintf(stderr, "%d of %zu scenario(s) failed.\n", scenarioFailures, files.size());
  return 1;
}
