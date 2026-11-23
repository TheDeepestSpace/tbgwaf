#include "server/GameStore.h"

#include <sqlite3.h>

#include <chrono>
#include <cstdio>

namespace tactics::server {

namespace {
int64_t NowMs() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}
}  // namespace

// RAII prepared statement with positional binds.
struct GameStore::Stmt {
  sqlite3_stmt* s = nullptr;
  bool good = false;
  Stmt(sqlite3* db, const char* sql) { good = sqlite3_prepare_v2(db, sql, -1, &s, nullptr) == SQLITE_OK; }
  ~Stmt() { sqlite3_finalize(s); }
  Stmt(const Stmt&) = delete;
  Stmt& operator=(const Stmt&) = delete;
  Stmt& Text(int i, const std::string& v) {
    sqlite3_bind_text(s, i, v.c_str(), static_cast<int>(v.size()), SQLITE_TRANSIENT);
    return *this;
  }
  Stmt& Int(int i, int64_t v) {
    sqlite3_bind_int64(s, i, v);
    return *this;
  }
  bool Step() { return good && sqlite3_step(s) == SQLITE_ROW; }
  bool Run() { return good && sqlite3_step(s) == SQLITE_DONE; }
  std::string Col(int i) const {
    const unsigned char* t = sqlite3_column_text(s, i);
    return t ? std::string(reinterpret_cast<const char*>(t),
                           static_cast<size_t>(sqlite3_column_bytes(s, i)))
             : std::string();
  }
  int64_t I(int i) const { return sqlite3_column_int64(s, i); }
};

namespace {
constexpr const char* kGameColumns =
    "id, room, seed, map_kind, ctf, finished, winner, round, created_ms, updated_ms, round_limit";
}  // namespace

GameStore::GameStore(const std::string& path) : path_(path) {
  if (!Open(path)) return;
  sqlite3_stmt* version = nullptr;
  int found = 0;
  if (sqlite3_prepare_v2(db_, "PRAGMA user_version", -1, &version, nullptr) == SQLITE_OK &&
      sqlite3_step(version) == SQLITE_ROW) {
    found = sqlite3_column_int(version, 0);
  }
  sqlite3_finalize(version);
  if (found != 0 && found != kSchemaVersion) {
    // Other schema: drop everything instead of migrating (finished games and
    // all). A file-backed DB is deleted outright; an in-memory one is reopened.
    Close();
    if (path != ":memory:") {
      std::remove(path.c_str());
      std::remove((path + "-wal").c_str());
      std::remove((path + "-shm").c_str());
    }
    wasReset_ = true;
    if (!Open(path)) return;
  }
  const bool created =
      Exec("PRAGMA journal_mode=WAL") && Exec("PRAGMA synchronous=FULL") &&
      Exec("PRAGMA foreign_keys=ON") &&
      Exec("CREATE TABLE IF NOT EXISTS games ("
           " id TEXT PRIMARY KEY, room TEXT NOT NULL, seed INTEGER NOT NULL,"
           " map_kind TEXT NOT NULL, ctf INTEGER NOT NULL, round_limit INTEGER NOT NULL DEFAULT 0, finished INTEGER NOT NULL DEFAULT 0,"
           " winner TEXT NOT NULL DEFAULT '', round INTEGER NOT NULL DEFAULT 1,"
           " created_ms INTEGER NOT NULL, updated_ms INTEGER NOT NULL)") &&
      Exec("CREATE TABLE IF NOT EXISTS players ("
           " game_id TEXT NOT NULL REFERENCES games(id), team INTEGER NOT NULL,"
           " token TEXT NOT NULL, PRIMARY KEY (game_id, team))") &&
      Exec("CREATE INDEX IF NOT EXISTS players_token ON players(token)") &&
      Exec("CREATE TABLE IF NOT EXISTS orders ("
           " game_id TEXT NOT NULL REFERENCES games(id), round INTEGER NOT NULL,"
           " team INTEGER NOT NULL, json TEXT NOT NULL, PRIMARY KEY (game_id, round, team))") &&
      Exec("CREATE TABLE IF NOT EXISTS turns ("
           " game_id TEXT NOT NULL REFERENCES games(id), round INTEGER NOT NULL,"
           " blue TEXT NOT NULL, red TEXT NOT NULL, spectator TEXT NOT NULL,"
           " state_hash TEXT NOT NULL, PRIMARY KEY (game_id, round))") &&
      Exec(("PRAGMA user_version=" + std::to_string(kSchemaVersion)).c_str());
  if (!created) Close();
}

GameStore::~GameStore() { Close(); }

bool GameStore::Open(const std::string& path) {
  if (sqlite3_open(path.c_str(), &db_) != SQLITE_OK) {
    error_ = std::string("cannot open database: ") + (db_ ? sqlite3_errmsg(db_) : "out of memory");
    Close();
    return false;
  }
  sqlite3_busy_timeout(db_, 2000);
  return true;
}

void GameStore::Close() {
  if (db_) sqlite3_close(db_);
  db_ = nullptr;
}

bool GameStore::Exec(const char* sql) {
  char* message = nullptr;
  if (sqlite3_exec(db_, sql, nullptr, nullptr, &message) == SQLITE_OK) return true;
  error_ = message ? message : "sqlite error";
  sqlite3_free(message);
  return false;
}

bool GameStore::CreateGame(const Game& game, const std::string& blueToken,
                           const std::string& redToken) {
  if (!db_ || !Exec("BEGIN")) return false;
  const int64_t now = NowMs();
  Stmt g(db_,
         "INSERT INTO games (id, room, seed, map_kind, ctf, round_limit, round, created_ms, updated_ms)"
         " VALUES (?1, ?2, ?3, ?4, ?5, ?7, 1, ?6, ?6)");
  g.Text(1, game.id).Text(2, game.room).Int(3, game.seed).Text(4, game.mapKind).Int(5, game.ctf).Int(6, now)
      .Int(7, game.roundLimit);
  Stmt p(db_, "INSERT INTO players (game_id, team, token) VALUES (?1, ?2, ?3)");
  bool ok = g.Run();
  for (int team = 0; ok && team < 2; ++team) {
    sqlite3_reset(p.s);
    p.Text(1, game.id).Int(2, team).Text(3, team == 0 ? blueToken : redToken);
    ok = p.Run();
  }
  if (!ok) error_ = sqlite3_errmsg(db_);
  if (!Exec(ok ? "COMMIT" : "ROLLBACK")) ok = false;
  return ok;
}

bool GameStore::PutOrders(const std::string& gameId, int round, int team, const std::string& json) {
  if (!db_) return false;
  Stmt s(db_, "INSERT OR REPLACE INTO orders (game_id, round, team, json) VALUES (?1, ?2, ?3, ?4)");
  s.Text(1, gameId).Int(2, round).Int(3, team).Text(4, json);
  if (s.Run()) return true;
  error_ = sqlite3_errmsg(db_);
  return false;
}

bool GameStore::PutTurn(const std::string& gameId, const Turn& turn, bool finished,
                        const std::string& winner) {
  if (!db_ || !Exec("BEGIN")) return false;
  Stmt t(db_,
         "INSERT OR REPLACE INTO turns (game_id, round, blue, red, spectator, state_hash)"
         " VALUES (?1, ?2, ?3, ?4, ?5, ?6)");
  t.Text(1, gameId).Int(2, turn.round).Text(3, turn.messages[0]).Text(4, turn.messages[1])
      .Text(5, turn.messages[2]).Text(6, turn.stateHash);
  Stmt g(db_, "UPDATE games SET round = ?2, finished = ?3, winner = ?4, updated_ms = ?5 WHERE id = ?1");
  g.Text(1, gameId).Int(2, turn.round + 1).Int(3, finished).Text(4, winner).Int(5, NowMs());
  const bool ok = t.Run() && g.Run();
  if (!ok) error_ = sqlite3_errmsg(db_);
  if (!Exec(ok ? "COMMIT" : "ROLLBACK")) return false;
  return ok;
}

namespace {
template <typename S>
GameStore::Game ReadGame(const S& s, int base = 0) {
  GameStore::Game g;
  g.id = s.Col(base + 0);
  g.room = s.Col(base + 1);
  g.seed = static_cast<uint32_t>(s.I(base + 2));
  g.mapKind = s.Col(base + 3);
  g.ctf = s.I(base + 4) != 0;
  g.finished = s.I(base + 5) != 0;
  g.winner = s.Col(base + 6);
  g.round = static_cast<int>(s.I(base + 7));
  g.createdMs = s.I(base + 8);
  g.updatedMs = s.I(base + 9);
  g.roundLimit = static_cast<int>(s.I(base + 10));
  return g;
}
}  // namespace

std::optional<GameStore::Game> GameStore::Find(const std::string& id) const {
  if (!db_) return std::nullopt;
  Stmt s(db_, (std::string("SELECT ") + kGameColumns + " FROM games WHERE id = ?1").c_str());
  s.Text(1, id);
  if (!s.Step()) return std::nullopt;
  return ReadGame(s);
}

std::vector<GameStore::Game> GameStore::ActiveGames() const {
  std::vector<Game> out;
  if (!db_) return out;
  Stmt s(db_, (std::string("SELECT ") + kGameColumns +
               " FROM games WHERE finished = 0 ORDER BY created_ms, id").c_str());
  while (s.Step()) out.push_back(ReadGame(s));
  return out;
}

std::vector<GameStore::Game> GameStore::GamesOf(const std::string& token,
                                                std::vector<int>* teams) const {
  std::vector<Game> out;
  if (teams) teams->clear();
  if (!db_) return out;
  Stmt s(db_,
         "SELECT g.id, g.room, g.seed, g.map_kind, g.ctf, g.finished, g.winner, g.round,"
         " g.created_ms, g.updated_ms, g.round_limit, p.team FROM games g JOIN players p ON p.game_id = g.id"
         " WHERE p.token = ?1 ORDER BY g.updated_ms DESC, g.id");
  s.Text(1, token);
  while (s.Step()) {
    out.push_back(ReadGame(s));
    if (teams) teams->push_back(static_cast<int>(s.I(11)));
  }
  return out;
}

std::optional<int> GameStore::TeamOf(const std::string& gameId, const std::string& token) const {
  if (!db_) return std::nullopt;
  Stmt s(db_, "SELECT team FROM players WHERE game_id = ?1 AND token = ?2");
  s.Text(1, gameId).Text(2, token);
  if (!s.Step()) return std::nullopt;
  return static_cast<int>(s.I(0));
}

std::vector<GameStore::Order> GameStore::Orders(const std::string& gameId) const {
  std::vector<Order> out;
  if (!db_) return out;
  Stmt s(db_, "SELECT round, team, json FROM orders WHERE game_id = ?1 ORDER BY round, team");
  s.Text(1, gameId);
  while (s.Step()) out.push_back({static_cast<int>(s.I(0)), static_cast<int>(s.I(1)), s.Col(2)});
  return out;
}

std::vector<GameStore::Turn> GameStore::Turns(const std::string& gameId) const {
  std::vector<Turn> out;
  if (!db_) return out;
  Stmt s(db_, "SELECT round, blue, red, spectator, state_hash FROM turns WHERE game_id = ?1 ORDER BY round");
  s.Text(1, gameId);
  while (s.Step()) {
    Turn t;
    t.round = static_cast<int>(s.I(0));
    t.messages[0] = s.Col(1);
    t.messages[1] = s.Col(2);
    t.messages[2] = s.Col(3);
    t.stateHash = s.Col(4);
    out.push_back(std::move(t));
  }
  return out;
}

bool GameStore::RoomCodeUsed(const std::string& room) const {
  if (!db_) return false;
  Stmt s(db_, "SELECT 1 FROM games WHERE room = ?1 LIMIT 1");
  s.Text(1, room);
  return s.Step();
}

}  // namespace tactics::server
