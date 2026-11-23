# Game persistence, resume and "My games"

Two-player matches survive reconnects and server restarts. Finished matches
are kept forever and can be replayed.

## Identity

No accounts. Each browser makes an anonymous random **device token**
(`localStorage["tbgwaf.token"]`, 128 bits) and sends it on `join`. The token is
the only credential: a stored match is seated to the two tokens that started
it, and `my_games` returns only the games of the token it is asked with. The
last match played is remembered as `localStorage["tbgwaf.lastGame"]` (Continue).
A match is stored only if both players sent distinct tokens (Jev matches and
token-less clients stay in memory).

## Store (`src/server/GameStore`, SQLite)

| table | content |
| --- | --- |
| `games` | id, room, seed, map, ctf, `finished`, winner, next round, timestamps |
| `players` | the two seats' tokens (`game_id`, `team`, `token`) |
| `orders` | each team's committed plans + playbook per round (written on commit) |
| `turns` | the three round messages (Blue, Red, unfiltered) and the state hash per resolved round (written on resolve) |

Resolution is deterministic, so a game is rebuilt by re-simulating from the
seed and the stored orders; the state hash after every replayed round is
compared with the stored one and a mismatch refuses the restore. The stored
`turns` feed the post-game replay (`reveal`).

**No migrations.** The schema version is `PRAGMA user_version`
(`GameStore::kSchemaVersion`). If a database has another version it is deleted
and recreated: all stored games are dropped. Bump the version for any change
that would need a backfill.

## Configuration

`TBGWAF_DB_PATH` or `--db FILE` selects the SQLite file; unset keeps
everything in memory. The server refuses to start if the path cannot be
opened. On start every unfinished game is restored (pending commits included);
players rejoin with their token.

## Resume protocol

See the "Stored matches" block in `src/net/Protocol.h`. A dropped player's
seat is held (opponent sees `opponent_away`); the round waits for them.
Rejoining sends the snapshot, the latest resolved turn, and the plans with the
player's own commit flag.

## Client

Game Mode (online only) has **My games** (list: Resume for active, Replay for
finished) and **Continue** (resume the last match). Flow:
`flow/app_flow.yaml` (`my_games` state); logic: `net::GamesClient`,
`RemoteClient::SetToken/SetResume`.

## Tests

* `tests/server_tests.cpp`: store, restart/resume, seat holding, my games, finished replay.
* `tests/integration/*.yaml` run by `tactics_integration_tests` (ctest
  `integration_tests`): headless apps click the menu buttons against a real
  server with a temp DB: play, kill a client mid-turn, restart or SIGKILL the
  server, rejoin via Continue, commit survives, replay equals live, My games
  lists only own. See `tests/integration/README.md`. CI shards one scenario per
  job (`scripts/ci_matrix.py`).

## Fly.io

`server/fly.toml` mounts the volume `tbgwaf_data` at `/data` and sets
`TBGWAF_DB_PATH=/data/tbgwaf.db`; `server-deploy.yml` creates the volume if
missing and deploys with `--ha=false`.

**Single-machine constraint:** a Fly volume attaches to one machine and SQLite
needs a single writer, so never run more than one machine (no `fly scale
count`, no HA). Scale-to-zero is fine: the volume persists, and active games
are restored on the next start.

**Backup:** Fly takes daily volume snapshots, kept `snapshot_retention = 14`
days (`fly volumes snapshots list <volume-id>`; restore by creating a volume
`--snapshot-id`). Take one before risky changes with `fly volumes snapshots
create <volume-id>`. Litestream (continuous replication to object storage) is
the upgrade path if daily granularity is not enough.
