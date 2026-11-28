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
#include "server/GameStore.h"
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
  // Enables persistence of human-v-human and human-v-Jev matches (see GameStore): a match is
  // stored when both players joined with a device `token`, commits and
  // resolved turns are written as they happen, and matches still active in
  // the store are restored now (players rejoin with their token). `store`
  // must outlive the Lobby. Returns how many active games were restored.
  int SetGameStore(GameStore* store);
  // New human matches end as a draw after this many rounds (0 = unlimited;
  // test-control servers use a short limit to reach game over quickly).
  void SetRoundLimit(int rounds) { roundLimit_ = rounds; }
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
    std::string mapKind;      // Blue's splash pick for a human room ("" = default).
    std::optional<uint32_t> mapSeed;
    bool ctf = false;
    std::string botMode;      // "", "player-v-ai" or "ai-v-ai".
    bool botsRunning = true;  // ai-v-ai starts paused.
    int64_t nextBotMs = 0;    // Playback pacing for rooms with no human player.
    std::vector<int> spectators;  // Read-only, unfiltered viewers.
    std::unique_ptr<GameSession> session;
    std::optional<Scene> scene;  // Set by load_scenario.
    net::Json sceneSpec;         // The same scene, sent to clients in `start`.
    bool friendlyFire = true;    // Test knobs from load_scenario.
    bool figureOverlapCheck = true;
    std::vector<float> shotRolls;
    // Persistence (empty gameId = ephemeral room).
    std::string gameId;
    std::string tokens[2];   // Device tokens by Team, as sent on join.
    int turn = 1;            // Next round to resolve, from 1 (the store's round numbering).
    std::string lastRound[2];  // Latest resolved round message per team (resume).
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
  // my_games: the token's own stored games.
  void HandleMyGames(int conn, const net::Json& message);
  // Join by stored game id + token: reseats a player in an active game, or
  // sends a finished game's final state and full reveal for replay.
  void Resume(int conn, const std::string& gameId, const std::string& token);
  void ViewFinished(int conn, const GameStore::Game& game, Team team);
  // Rebuilds a stored game's session by replaying its resolved turns (and,
  // with `pending`, the commits of the round in progress).
  std::unique_ptr<GameSession> ReplayGame(const GameStore::Game& game, bool pending,
                                          int* nextTurn, std::string* error);
  void PersistNewGame(Room& room);
  void PersistCommit(Room& room, Team team);
  void PersistTurn(Room& room, const std::array<net::Json, 3>& reports);
  net::Json StartMessage(const Room& room, Team team) const;
  std::string NewGameId();
  void SpectatorJoin(int conn, const net::Json& message);
  void CreateBotRoom(int conn, const std::string& name, const std::string& mode,
                     const std::string& mapKind = "", std::optional<uint32_t> seed = std::nullopt,
                     bool ctf = false,
                     const std::string& token = "");
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
  GameStore* gameStore_ = nullptr;
  int roundLimit_ = 0;
  int maxBotRooms_ = 4;
  bool pacePlayback_ = true;
  uint64_t nextRoomId_ = 1;
  unsigned botNonce_ = 0;
  std::map<std::string, Room> rooms_;
  std::map<int, Member> members_;
  std::optional<std::mt19937> codeRng_;
};

}  // namespace tactics::server
