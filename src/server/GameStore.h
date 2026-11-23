#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

struct sqlite3;

namespace tactics::server {

// SQLite persistence for two-player matches (see docs/persistence.md).
//
// What is stored is exactly what is needed to rebuild a match: the game row
// (seed, map, status), one row per seat with the player's anonymous device
// token, each team's committed orders per round, and each resolved turn's
// round messages. Resolution is deterministic, so replaying the orders from
// the seed reproduces the state; the stored turns serve the post-game replay
// and a consistency check (state hash).
//
// There are no migrations: kSchemaVersion is kept in `PRAGMA user_version`,
// and a database written by a different version is deleted and recreated.
class GameStore {
 public:
  static constexpr int kSchemaVersion = 1;

  struct Game {
    std::string id;
    std::string room;
    uint32_t seed = 0;
    std::string mapKind;
    bool ctf = false;
    int roundLimit = 0;  // 0 = unlimited; a draw is declared after this many rounds.
    bool finished = false;
    std::string winner;  // "blue" | "red" | "none" once finished.
    int round = 1;       // Resolved rounds + 1 while active.
    int64_t createdMs = 0, updatedMs = 0;
  };
  struct Order {
    int round = 0;
    int team = 0;
    std::string json;  // {"plans":[<action>...],"playbook":[...]}
  };
  struct Turn {
    int round = 0;
    std::string messages[3];  // Blue's, Red's and the unfiltered (spectator) round message.
    std::string stateHash;
  };

  // Opens (creating if needed) the database at `path`; ":memory:" for a
  // throwaway one. Check ok().
  explicit GameStore(const std::string& path);
  ~GameStore();
  GameStore(const GameStore&) = delete;
  GameStore& operator=(const GameStore&) = delete;

  bool ok() const { return db_ != nullptr; }
  const std::string& error() const { return error_; }
  // True if opening found a database of another schema version and recreated it.
  bool wasReset() const { return wasReset_; }

  bool CreateGame(const Game& game, const std::string& blueToken, const std::string& redToken);
  bool PutOrders(const std::string& gameId, int round, int team, const std::string& json);
  // Stores a resolved turn and advances the game's round (or finishes it).
  bool PutTurn(const std::string& gameId, const Turn& turn, bool finished,
               const std::string& winner);

  std::optional<Game> Find(const std::string& id) const;
  std::vector<Game> ActiveGames() const;
  // The token's own games (any status), most recently updated first; `teams`
  // gets the seat (0 Blue / 1 Red) for each.
  std::vector<Game> GamesOf(const std::string& token, std::vector<int>* teams) const;
  std::optional<int> TeamOf(const std::string& gameId, const std::string& token) const;
  std::vector<Order> Orders(const std::string& gameId) const;
  std::vector<Turn> Turns(const std::string& gameId) const;
  bool RoomCodeUsed(const std::string& room) const;

 private:
  bool Exec(const char* sql);
  bool Open(const std::string& path);
  void Close();
  struct Stmt;

  sqlite3* db_ = nullptr;
  std::string path_;
  std::string error_;
  bool wasReset_ = false;
};

}  // namespace tactics::server
