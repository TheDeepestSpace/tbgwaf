# Jev gameplay preview

The browser preview has three modes on the same page and uses the same WASM
`GameLogic` instance as human play:

- **Human vs Human** (`?mode=human`) is the original two-canvas mode.
- **Blue vs Jev** (`?mode=player-v-ai`) leaves Blue interactive and lets Jev
  plan Red. The Blue simulator commits automatically when both teams finish.
- **Jev vs Jev** (`?mode=ai-v-ai`) starts paused. Start, pause, restart, camera
  orbit, and camera zoom remain available to the spectator.

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

Environment Jev is told: an ASCII top-down map (`state.map.grid`, 0.75-unit cells,
up to 96x96, sampled at cell centres against the exact obstacle footprints; `#`
obstacle, `+` climbable, own figures, visible enemies and ghosts marked, with
`cell_size` and the coordinate mapping in the legend), the nearest 48 obstacles with
exact footprints, FOV half-angle, shoot range, each figure's move budget, the team's
reaction playbook, and `ghosts` (last-known enemy positions, rounds ago,
movement direction). Move options say whether the destination is hidden from
or exposed to the currently visible enemies; `cover_N` moves hide behind
obstacle N away from the nearest threat and `hunt_N` moves toward a ghost.
Only information the team could see is sent.

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
