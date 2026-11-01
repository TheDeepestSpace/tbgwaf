// Headless replay of a demo-scene or Builder repro log (issue #240):
//
//   tactics_repro_replay <log.jsonl> [--quiet]
//
// Rebuilds the log's scene (a registry demo scene, or the Builder scene
// embedded in its header), feeds every recorded input back through
// GameLogic and checks the state hash after each one against the recorded
// value. Exit 0 when every hash matched, 1 on a mismatch or a malformed /
// unknown-scene log. Prints the sim log lines and the first few mismatches
// so a developer can see where a replay diverges.

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>

#include "game/BuilderScene.h"
#include "game/ReproLog.h"

int main(int argc, char** argv) {
  const char* path = nullptr;
  bool quiet = false;
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--quiet") == 0) {
      quiet = true;
    } else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
      std::printf("usage: tactics_repro_replay <log.jsonl> [--quiet]\n");
      return 0;
    } else if (!path) {
      path = argv[i];
    } else {
      std::fprintf(stderr, "unexpected argument %s\n", argv[i]);
      return 2;
    }
  }
  if (!path) {
    std::fprintf(stderr, "usage: tactics_repro_replay <log.jsonl> [--quiet]\n");
    return 2;
  }
  std::ifstream in(path);
  if (!in) {
    std::fprintf(stderr, "cannot read %s\n", path);
    return 2;
  }
  std::stringstream buffer;
  buffer << in.rdbuf();

  tactics::GameLogic game;
  tactics::ReproLog log;
  const tactics::ReplayReport report = tactics::ReplayAnyLog(buffer.str(), &game, &log);
  if (!report.error.empty()) {
    std::fprintf(stderr, "FAIL %s: %s\n", path, report.error.c_str());
    return 1;
  }
  if (!quiet) {
    std::printf("scene=%s seed=%u build=%s (this build: %s)%s\n", log.header.scene.c_str(),
                static_cast<unsigned>(log.header.seed), log.header.build.c_str(),
                tactics::ReproBuildId(),
                log.header.build == tactics::ReproBuildId() ? "" : "  [different build]");
    for (const tactics::ReproEntry& entry : log.entries) {
      if (entry.kind == tactics::ReproEntry::Kind::Log) {
        std::printf("  [seq %d round %d t=%llums] %s\n", entry.seq, entry.round,
                    static_cast<unsigned long long>(entry.timeMs), entry.message.c_str());
      }
    }
  }
  std::printf("%s: %d entries (%d inputs, %d frames), final state %s, round %d, %zu mismatch(es)\n",
              report.ok ? "PASS" : "FAIL", report.entriesApplied, report.inputsApplied,
              report.updatesApplied, tactics::HashToHex(report.finalHash).c_str(),
              game.RoundNumber(), report.mismatches.size());
  size_t shown = 0;
  for (const auto& m : report.mismatches) {
    if (shown++ >= 10) {
      std::printf("  ...\n");
      break;
    }
    std::printf("  seq %d: recorded %s, replayed %s\n", m.seq, tactics::HashToHex(m.expected).c_str(),
                tactics::HashToHex(m.actual).c_str());
  }
  return report.ok ? 0 : 1;
}
