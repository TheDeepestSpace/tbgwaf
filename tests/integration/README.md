# Integration scenarios

`tactics_integration_tests` (ctest `integration_tests`) runs every `*.yaml`
here against a real `tbgwaf_server_testctl` with a temporary SQLite database
on localhost. Build target: `tactics_integration_tests`; run one scenario with
`--only <name>`.

A scenario is `name`, `description`, optional `round_limit: N` (the server ends matches as a draw after N rounds, so a match can finish quickly) and `steps`; each step is a one-key map:

| step | meaning |
| --- | --- |
| `server: start\|stop\|restart\|crash` | `restart` = SIGTERM + start, `crash` = SIGKILL + start; the DB file survives |
| `app: [alice, bob]` | headless apps (menu flow + device token + last game, as in the browser) |
| `click: {app, button, row?}` | press a menu button by its label (`src/ui/MenuButtons.h`); on My games, `Resume`/`Replay` + `row` |
| `type_room_code: {app, from}` | type `from`'s room code into `app`'s Room box |
| `round: {apps: [..], count?}` | plan every figure, commit, play the round out |
| `plan: app` / `commit: app` | plan (and commit) one app only |
| `reload: app` | the app is killed / the tab closed: socket dropped, back to the splash (token and last game stay) |
| `snapshot: {app, name}` | remember the app's visible state |
| `assert: {app, ...}` | `screen`, `status`, `round`, `self_ready`, `peer_ready`, `peer_away`, `game_over`, `has_reveal`, `error`, `games_total/active/finished`, `same_as` (a snapshot), `game_id_differs_from` (another app) |
