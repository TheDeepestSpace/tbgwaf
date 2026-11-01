#pragma once

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "server/GameSession.h"
#include "server/JevBot.h"
#include "server/SceneStore.h"

namespace tactics::server {

// Pairs connections into games: "a session is just two sockets, one game".
// The first connection to join a room waits (Blue); the second is paired
// (Red) and the authoritative GameSession starts. Transport-agnostic: the
// socket layer feeds it messages and supplies a send callback. A third kind of
// connection, a spectator ({"t":"join","room":..,"spectate":true}), watches a
// room read-only with unfiltered state; it is not one of the two players.
class Lobby {
 public:
  using SendFn = std::function<void(int conn, const std::string& text)>;
  using SeedFn = std::function<uint32_t()>;
  Lobby(SendFn send, SeedFn seed) : send_(std::move(send)), seed_(std::move(seed)) {}

  // Enables the server-side Jev modes (join with "mode":"player-v-ai" or
  // "ai-v-ai"). `bots` must outlive the Lobby. `pacePlayback` holds a
  // spectator-only match's next round until the previous one has been played
  // back (turn off in tests).
  void SetBots(JevBotRunner* bots, int maxBotRooms = 4, bool pacePlayback = true) {
    bots_ = bots;
    maxBotRooms_ = maxBotRooms;
    pacePlayback_ = pacePlayback;
  }
  // Enables Builder scene storage (scenes_list / scene_put / scene_delete,
  // usable by any connection, joined or not). `store` must outlive the Lobby.
  void SetSceneStore(SceneStore* store) { sceneStore_ = store; }
  // Call every loop iteration: applies finished Jev answers and starts new
  // requests. Cheap when no room has a bot.
  void Tick();
  bool BotsBusy() const { return bots_ && bots_->Busy(); }

  // One inbound text message from `conn`. Never throws on bad input.
  void OnMessage(int conn, const std::string& text);
  void OnDisconnect(int conn);

  // Test-only control tap (see Server). `load_scenario` pre-registers a room
  // whose match runs on an explicit scene; `dump_state` returns the room's
  // unfiltered server-side state. Always returns the reply to send.
  net::Json OnControl(const net::Json& message);

  size_t RoomCount() const { return rooms_.size(); }

 private:
  // One Jev-controlled seat (a team with no connection).
  struct BotSeat {
    bool active = false;
    bool inFlight = false;
    bool fallback = false;  // User opted into the deterministic non-Jev planner.
    std::string error;      // Held until a retry; the seat does not re-ask on its own.
  };
  struct Room {
    std::string name;
    uint64_t id = 0;  // Unique per room instance, so a late Jev answer can't hit a successor.
    int conns[2] = {-1, -1};  // Indexed by Team; -1 = bot seat.
    std::array<BotSeat, 2> bot;
    std::string botMode;      // "", "player-v-ai" or "ai-v-ai".
    bool botsRunning = true;  // ai-v-ai starts paused.
    int64_t nextBotMs = 0;    // Playback pacing for rooms with no human player.
    std::vector<int> spectators;  // Read-only, unfiltered viewers.
    std::unique_ptr<GameSession> session;
    std::optional<Scene> scene;  // Set by load_scenario.
    net::Json sceneSpec;         // The same scene, sent to clients in `start`.
    bool friendlyFire = true;    // Test knobs from load_scenario.
    std::vector<float> shotRolls;
  };
  struct Member {
    std::string room;
    Team team = Team::Blue;
    bool spectator = false;
  };

  std::string NewRoomCode();  // A code no existing room uses.
  void Join(int conn, const net::Json& message);
  void HandleAction(int conn, Member& member, const net::Json& message);
  // scenes_list / scene_put / scene_delete; false if `type` is not one of them.
  bool HandleSceneMessage(int conn, const std::string& type, const net::Json& message);
  void StartMatch(Room& room);
  void SpectatorJoin(int conn, const net::Json& message);
  void CreateBotRoom(int conn, const std::string& name, const std::string& mode,
                     const std::string& mapKind = "", std::optional<uint32_t> seed = std::nullopt);
  void HandleBotControl(int conn, Room& room, const net::Json& message);
  void MaybeRunRound(Room& room);
  void ScheduleBots(Room& room);
  void BroadcastBots(const Room& room);
  void AnnounceBotCommit(Room& room, Team bot);
  void SendSeat(const Room& room, Team team, const net::Json& message) {
    if (room.conns[static_cast<int>(team)] >= 0) Send(room.conns[static_cast<int>(team)], message);
  }
  static int64_t NowMs();
  void SendSpectatorView(int conn, const Room& room);  // start + plans + state.
  void Send(int conn, const net::Json& message) { send_(conn, message.Dump()); }
  void SendError(int conn, const std::string& why);

  SendFn send_;
  SeedFn seed_;
  JevBotRunner* bots_ = nullptr;
  SceneStore* sceneStore_ = nullptr;
  int maxBotRooms_ = 4;
  bool pacePlayback_ = true;
  uint64_t nextRoomId_ = 1;
  unsigned botNonce_ = 0;
  std::map<std::string, Room> rooms_;
  std::map<int, Member> members_;
  std::optional<std::mt19937> codeRng_;
};

}  // namespace tactics::server
