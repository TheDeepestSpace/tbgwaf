#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "game/GameLogic.h"
#include "net/Json.h"
#include "net/Protocol.h"

namespace tactics::net {

// Client-side protocol logic for server-authoritative play, kept free of any
// socket / emscripten / SDL code so it is testable natively. The embedding
// layer (src/main.cpp in the WASM build) shuttles text frames between a
// WebSocket and OnMessage()/TakeOutgoing().
//
// The local GameLogic is a presentation mirror: the click/choose flow plans
// into it as usual (so previews, overlays and HUD work unchanged), SyncPlans()
// turns plan changes into discrete actions for the server, and the server's
// replies are applied back -- rule state directly, and a round's fog-filtered
// timeline as locally-synthesized playback (walk cycles, shot beats and falls
// are derived here; none of that crosses the wire). Enemy figures the team
// cannot currently see are parked far off-map, never reconstructed.
class RemoteClient {
 public:
  enum class Status { Connecting, Waiting, Playing, OpponentLeft, Disconnected };

  RemoteClient(tactics::GameLogic* game, std::string room);

  // Transport events.
  void OnOpen();
  void OnMessage(const std::string& text);
  void OnClose();

  // Frames to send, in order (drains the queue).
  std::vector<std::string> TakeOutgoing();

  // Call every frame: advances round playback by `dt` and, while planning,
  // queues actions for any of our plans that differ from what the server has.
  void Update(float dt);

  void RequestCommit();
  void RequestNewMatch();

  Status status() const { return status_; }
  std::optional<tactics::Team> team() const { return team_; }
  bool playingBack() const { return playback_.has_value(); }
  bool selfReady() const { return selfReady_; }
  bool peerReady() const { return peerReady_; }
  const std::string& lastError() const { return lastError_; }
  // True once after a (new) match started so the host can re-frame cameras.
  bool ConsumeMatchStarted() {
    const bool v = matchStarted_;
    matchStarted_ = false;
    return v;
  }
  std::string StatusText() const;

 private:
  struct Sample {
    int id = -1;
    glm::vec3 pos{0.0f};
    float yaw = 0.0f;
    bool alive = true;
    bool moving = false;
  };
  struct Frame {
    float t = 0.0f;
    std::vector<Sample> units;
  };
  struct Shot {
    float t = 0.0f;
    int unit = -1;
    float yaw = 0.0f;
  };
  struct Playback {
    std::vector<Frame> frames;
    std::vector<Shot> shots;
    float clock = 0.0f;
    float duration = 0.0f;
    Json finalState;
  };

  void Queue(const Json& message) { outgoing_.push_back(message.Dump()); }
  void QueueAction(Action action);
  void HandleStart(const Json& m);
  void HandlePlans(const Json& m);
  void HandleRound(const Json& m);
  void ApplyPlans(const Json& m);
  void ApplyState(const Json& state);
  void AdvancePlayback(float dt);
  void SyncPlans();
  void NormalizeEnemyPlans();

  tactics::GameLogic* game_;
  std::string room_;
  Status status_ = Status::Connecting;
  std::optional<tactics::Team> team_;
  std::vector<std::string> outgoing_;
  std::optional<Playback> playback_;
  std::optional<Json> pendingPlans_;  // Arrived mid-playback; applied once it ends.
  std::map<int, std::string> syncedPlan_;   // Unit id -> last plan signature the server has.
  std::optional<tactics::SquadPlaybook> syncedPlaybook_;
  bool selfReady_ = false;
  bool peerReady_ = false;
  bool matchStarted_ = false;
  int nextSeq_ = 1;
  std::string lastError_;
};

// Where unseen enemies are parked in the local mirror (beyond any shot range).
constexpr float kHiddenCoord = 10000.0f;

}  // namespace tactics::net
