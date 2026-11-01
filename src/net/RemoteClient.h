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
  enum class Status { Connecting, Waiting, Playing, OpponentLeft, Disconnected, Rejected };

  // `intent` is "create", "join" or empty (see Lobby::Join). With "create" and
  // no room, the server picks the code (room_code() once waiting).
  // `spectate` attaches read-only to an existing room: every figure and both
  // teams' plans are mirrored, and nothing is ever sent but the join.
  RemoteClient(tactics::GameLogic* game, std::string room, std::string intent = "",
               bool spectate = false);

  // Ask the server to seat Jev: "player-v-ai" (we play Blue against Jev's Red)
  // or, with `spectate`, "ai-v-ai" (watch two Jevs). Empty = a normal match.
  // Call before OnOpen().
  void SetBotMode(std::string mode) { botMode_ = std::move(mode); }
  // Splash map ("urban"|"urban-x4"|"hills") and seed for the Jev room the
  // server opens; unset = the server's default random city.
  void SetMatchSetup(std::string mapKind, std::optional<uint32_t> seed) {
    mapKind_ = std::move(mapKind);
    mapSeed_ = seed;
  }

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
  bool spectating() const { return spectate_; }
  std::optional<tactics::Team> team() const { return team_; }
  bool playingBack() const { return playback_.has_value(); }
  bool selfReady() const { return selfReady_; }
  bool peerReady() const { return peerReady_; }
  // True from the moment RequestCommit() queues the commit until the server
  // either refuses it or runs the round: the plans are locked in, so planning
  // input (and the Commit button) must be disabled.
  bool CommitLocked() const { return commitPending_ || selfReady_; }
  const std::string& lastError() const { return lastError_; }
  const std::string& room_code() const { return room_; }
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
  struct TracerEvent {
    float t = 0.0f;
    Team team = Team::Blue;
    glm::vec3 from{0.0f}, to{0.0f};
  };
  struct Playback {
    std::vector<Frame> frames;
    std::vector<Shot> shots;
    std::vector<TracerEvent> tracers;
    float clock = 0.0f;
    float duration = 0.0f;
    Json finalState;
  };

  void Queue(const Json& message) { outgoing_.push_back(message.Dump()); }
  // Assigns the next seq, queues the action, and returns that seq.
  int QueueAction(Action action);
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
  std::string intent_;
  std::string botMode_;
  std::string mapKind_;
  std::optional<uint32_t> mapSeed_;
  bool spectate_ = false;
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
  // In-flight window: a "plans" echo received while lastAckedSeq_ <
  // lastSentSeq_ describes a server state older than the local mirror (our
  // actions it hasn't processed yet), so it is held, not applied.
  int lastSentSeq_ = 0;
  int lastAckedSeq_ = 0;
  // Commit lock: set when RequestCommit() queues the commit, cleared when the
  // server refuses it (nack of commitSeq_), when a round runs, or on restart.
  bool commitPending_ = false;
  int commitSeq_ = 0;
  std::string lastError_;
};

// Where unseen enemies are parked in the local mirror (beyond any shot range).
constexpr float kHiddenCoord = 10000.0f;

}  // namespace tactics::net
