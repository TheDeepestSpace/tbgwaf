# Jev gameplay preview

The browser preview has three modes on the same page and uses the same WASM
`GameLogic` instance as human play:

- **Human vs Human** (`?mode=human`) is the original two-canvas mode.
- **Blue vs Jev** (`?mode=player-v-ai`) leaves Blue interactive and lets Jev
  plan Red. The Blue simulator commits automatically when both teams finish.
- **Jev vs Jev** (`?mode=ai-v-ai`) starts paused. Start, pause, restart, camera
  orbit, and camera zoom remain available to the spectator. It shows a single
  omniscient third-person pane: every Blue and Red figure is drawn (no fog of
  war) with both teams' translucent FOV cones. This is a mode-gated render path
  in the existing WASM build (`Module.tbgwafSpectator`, set only on the Blue
  instance; the Red instance keeps simulating in a hidden pane), not a separate
  binary. It is rendering-only: Jev's request state still contains only
  team-visible information. With a server configured, the Blue and Red Jev
  clients stay fog-limited and run in hidden panes; the visible pane is a
  third, read-only client that joins the room with `"spectate":true` and is
  sent unfiltered state, both teams' plans and unfiltered rounds. Other modes keep their two fog-limited panes.

Game code builds a bounded set of legal actions (about 25 per figure) for every
unplanned figure on the team (visible shots, navmesh-validated moves incl.
cover/hunt moves, and wait). Candidate ids are prefixed with the figure
(`f0_move_3`) and carry a `figure` field. The request state contains all
friendlies and only enemies in that team's current FOV. The adapter sends one
TypeSafe request with one `Choice` question per figure (`figure_<id>`), so Jev
plans the whole squad at once and can coordinate cover, focus fire and staggered
exposure. The returned ids (exactly one per figure) are checked against the
original set and replayed in turn on a copy of the game through the normal
click/plan path; if any one is invalid, none is applied. Jev does not generate
commands. A figure with a single legal option is not put to the model.

Shooting mechanics Jev is told: shot options fire a full burst (the weapon's
magazine, capped by shots that fit the 5 s round), each bullet scattered in a
cone; the option text carries the current per-shot hit chance, and the state
lists each ally's weapon, magazine, fire interval, scatter and max burst. The
objective text also covers FOV/LOS gating, friendly fire, and reaction fire.

Capture the flag: when the scene's flag is enabled, the objective text explains the
win condition (first figure to touch the flag wins when `winOnGrab`), `state.flag`
gives position/status/carrier, the grid marks it `F`, and each figure gets an `f<id>_flag`
move option toward it (`state.flag` is `null` otherwise).

Environment Jev is told: an ASCII top-down map (`state.map.grid`, 0.75-unit cells,
up to 96x96, sampled at cell centres against the exact obstacle footprints; `#`
obstacle, `+` climbable, own figures, visible enemies and ghosts marked, with
`cell_size` and the coordinate mapping in the legend), the nearest 48 obstacles with
exact footprints, FOV half-angle, shoot range, each figure's move budget, the team's
reaction playbook, and `ghosts` (last-known enemy positions, rounds ago,
movement direction). Move options say whether the destination is hidden from
or exposed to the currently visible enemies; `cover_N` moves hide behind
obstacle N away from the nearest threat and `hunt_N` moves toward a ghost.
The `flag`, `hunt_N` and `advance` (no flag) options are routed on the navmesh
rather than aimed in a straight line: each round's leg is one move budget along
the shortest legal route (around buildings, down graded roads, up stairs), and
the description states how much route remains, so a figure on a terrain level
is walked off it over several rounds. `wait` is offered only once an enemy is
visible or remembered; with no contact, a figure that can move must move.
Only information the team could see is sent.

## Server-hosted Jev (online play)

When the page is served with a game server (`?server=...`, see the README), Blue vs Jev
and Jev vs Jev run **on the server**, not in the browser:

- The toolbar still selects the mode; picking one reloads into a fresh room with
  `join {mode}` (`player-v-ai`: you are Blue, Jev is Red; `ai-v-ai`: a read-only
  spectator watches two Jevs, starting paused).
- The server (`src/server/JevBot.*`) builds the same bounded request from a copy of the
  authoritative game (`JevRequestBuilder`, team-visible information only), POSTs it to the
  Jev adapter (`TBGWAF_JEV_ADAPTER_URL`, plain HTTP), replays the chosen ids through
  `ApplyJevChoice` and commits for Jev. The browser never talks to Jev or the proxy, and
  the toolbar's **Jev proxy** box is hidden. All of that runs on worker threads; the poll
  loop only applies finished answers.
- Start/Pause/Retry/Fallback are relayed as `bot_control` messages; the server reports
  `bots` status (thinking, error, fallback) back. After an adapter error the seat stops
  asking until Retry; the deterministic fallback is still opt-in and labelled.
- In Jev vs Jev the next round waits until the previous one has played back. Concurrent
  Jev matches are capped by `TBGWAF_JEV_MAX_ROOMS` (default 4); the adapter's own rate and
  daily limits still apply, so a public server cannot burn the Jev budget unboundedly.
- The `server/Dockerfile` image bundles the adapter and starts it on `127.0.0.1:8787` when
  the `TBGWAF_JEV_API_KEY` secret is set (the deploy workflow stages it; `entrypoint.sh`),
  so Fly's server calls Jev directly and the key never leaves it. To use a separate adapter
  instead, set `TBGWAF_JEV_ADAPTER_URL` (`http://` only: the adapter terminates TLS to
  Jev). Without either, the Jev modes answer "Jev is not configured on this server".
- Server games have no flag (CTF is not part of the server's match setup), so Jev uses its
  no-flag `advance` option there.

## Run locally

Build the WASM app as usual, then start the adapter in another terminal. The
API key belongs only in the adapter process:

```sh
emcmake cmake -B build-web -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-web

TBGWAF_JEV_API_KEY=... node server/jev-adapter/server.mjs
python3 -m http.server 8000 -d build-web
```

Open:

```text
http://localhost:8000/?mode=player-v-ai&jev=http://localhost:8787
http://localhost:8000/?mode=ai-v-ai&jev=http://localhost:8787
```

The proxy URL can also be entered in the page's **Jev proxy** control and is
stored locally by the browser. Never put the TypeSafe key in this URL, page,
WASM build, or browser storage.

Run deterministic coverage without a key:

```sh
node --test web/tests/*.test.mjs server/jev-adapter/tests/*.test.mjs
ctest --test-dir build -R 'jev_planner_tests|logic_tests' --output-on-failure
```

## Hosted Pages and PR previews

The mode controls and `?mode=` entry points are included in both the production
Pages workflow and each PR preview. GitHub Pages is static and cannot safely
hold `TBGWAF_JEV_API_KEY`, so live decisions require a separately hosted
adapter. It defaults to `https://tbgwaf-jev-adapter.fly.dev` (the app name the deploy workflow uses); override it through `?jev=https://your-adapter.example` or the page
control.

`server/jev-adapter/Dockerfile` and `fly.toml.example` provide a concrete Fly.io
path:

```sh
cd server/jev-adapter
cp fly.toml.example fly.toml
# Edit the unique app name and exact Pages/preview origins first.
fly apps create YOUR_APP_NAME
fly secrets set TBGWAF_JEV_API_KEY=...
fly deploy
```

Set `TBGWAF_JEV_ALLOWED_ORIGINS` to a comma-separated exact origin list. For
this repository, production Pages and PR previews share the
`https://thedeepestspace.github.io` origin. Do not use `*`.

The adapter limits input to 128 KiB, 32 candidates per figure (200 total), defaults to 12 requests
per client per minute, two concurrent upstream requests, and a hard 500-request
in-memory daily budget. Each upstream attempt times out after 15 seconds and at
most one retry is made. These values are configurable with
`TBGWAF_JEV_RATE_PER_MINUTE`, `TBGWAF_JEV_DAILY_LIMIT`,
`TBGWAF_JEV_MAX_CONCURRENCY`, `TBGWAF_JEV_TIMEOUT_MS`, and
`TBGWAF_JEV_RETRIES`. `TBGWAF_JEV_PROXY_BEARER_TOKEN` can restrict non-public
deployments; a public static browser must not embed that token. Origin checks
are not authentication, so retain the hard request budget and place the
adapter behind a platform access gateway if the preview should be private.

`.github/workflows/jev-adapter-deploy.yml` (manual `workflow_dispatch`) deploys the
adapter using the `TBGWAF_FLYIO_TBGWAF_ORG_TOKEN` and `TBGWAF_JEV_API_KEY` Actions
secrets; it creates the app if missing.

The example Fly service auto-stops at zero machines. No hosted adapter is
deployed by this repository: deployment still requires the app owner's Fly.io
account and runtime secret. Until its URL is configured, the page reports the
missing proxy explicitly and offers retry or an opt-in deterministic fallback;
the fallback is always labelled as not live Jev.

## Actions smoke

`.github/workflows/jev-live-smoke.yml` reads the repository Actions secret
`TBGWAF_JEV_API_KEY` only inside a same-repository Actions run. It performs
exactly one upstream Choice request with retries disabled and reports
`api_requests=1`. Fork PRs do not receive the secret. The dispatcher or local
development environment is never expected to have it.
