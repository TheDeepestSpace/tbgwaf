#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "ai/JevPlanner.h"
#include "game/GameLogic.h"

namespace tactics::server {

// Server-side Jev gaming client. For one bot-controlled team it builds the
// bounded legal-action request from a copy of the authoritative game
// (ai::JevRequestBuilder), asks the Jev adapter for a choice per figure, and
// hands the answer back to the poll loop, which replays it through the normal
// click/plan path (ai::ApplyJevChoice). Jev never generates commands; the
// adapter (server/jev-adapter) holds the Jev API key and talks to Jev's API.
//
// All slow work (request building is ~100 ms per candidate, the upstream call
// up to seconds) happens on worker threads so the single-threaded poll loop
// never blocks. The poll loop only calls Submit() and TakeResults().
struct JevJob {
  uint64_t roomId = 0;
  std::string roomName;
  Team team = Team::Blue;
  GameLogic game;        // Private copy; the builder works on it.
  unsigned nonce = 0;
  bool fallback = false;  // Deterministic non-Jev choice (explicit user opt-in); no network.
};

struct JevResult {
  uint64_t roomId = 0;
  std::string roomName;
  Team team = Team::Blue;
  bool nothingToAsk = false;  // No unplanned figure / no legal action.
  ai::JevRequest request;
  std::vector<std::string> choices;  // One option id per request.actorIds entry.
  std::string error;                 // Non-empty: the request failed (message is client-safe).
  bool fallback = false;
};

struct JevBotConfig {
  std::string adapterUrl;  // "http://host:port[/base]"; empty = bots disabled.
  int timeoutMs = 30000;
  int workers = 2;
};

class JevBotRunner {
 public:
  // Sends the request JSON, returns the adapter's HTTP status and body (or
  // false + error on transport failure). Injectable so tests need no network.
  using Poster = std::function<bool(const std::string& requestJson, int* status,
                                    std::string* body, std::string* error)>;

  explicit JevBotRunner(JevBotConfig config, Poster poster = nullptr);
  ~JevBotRunner();
  JevBotRunner(const JevBotRunner&) = delete;
  JevBotRunner& operator=(const JevBotRunner&) = delete;

  bool enabled() const { return enabled_; }
  void Submit(JevJob job);
  std::vector<JevResult> TakeResults();
  // True while any job is queued or running (the poll loop shortens its wait).
  bool Busy() const { return pending_.load() > 0; }

  // Maps the adapter's reply to one option id per figure, in
  // request.actorIds order. Empty *error on success.
  static std::vector<std::string> ParseReply(const ai::JevRequest& request, int status,
                                             const std::string& body, std::string* error);

 private:
  void Work();
  JevResult Run(JevJob& job);

  JevBotConfig config_;
  Poster poster_;
  bool enabled_ = false;
  std::atomic<bool> stopping_{false};
  std::atomic<int> pending_{0};
  std::mutex mutex_;
  std::condition_variable wake_;
  std::deque<JevJob> jobs_;
  std::vector<JevResult> results_;
  std::vector<std::thread> threads_;
};

}  // namespace tactics::server
