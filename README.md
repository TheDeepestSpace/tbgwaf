# tbgwaf

A turn-based tactics prototype: two 3-figure squads (Blue vs Red) fight on a
blocky obstacle field, each side fogged from the other's view, until one team
is eliminated.

**[Play it in your browser](https://thedeepestspace.github.io/tbgwaf/)** --
both players on one screen, no install required.

The page also includes **Blue vs Jev** and **Jev vs Jev** preview modes. They
use the same WASM game and require a server-side TypeSafe adapter; the API key
is never sent to the browser. See [Jev gameplay preview](docs/jev-preview.md)
for local setup, Pages/PR-preview URLs, deployment, limits, and smoke tests.

## Play in browser

The [live Pages build](https://thedeepestspace.github.io/tbgwaf/) is the same
native game compiled to WebAssembly/WebGL2. The page shows two canvases side
by side -- Blue on the left, Red on the right -- each with its own client
instance of the game (own camera, input, and UI); the WASM module is
instantiated once per canvas. Both teams plan at once: each instance sends
its own team's plans to the other over an in-page message bus, and Commit
Round (from either canvas) makes the Blue instance run the simulation and
broadcast it, which the Red instance mirrors. That in-page demo needs no
server; for real cross-device play see [Online play](#online-play-server)
below. See [Controls](#controls) below.

## Native build

```sh
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Run the game with `./build/tactics_app`. `ctest` also runs a headless
smoke test under `xvfb-run` if available.

Environment knobs: `TBGWAF_MAP_SEED=<n>` reseeds the procedural map.
The default map (also what the web build serves) is `urban-elevated`; the plain urban map lays an oblique boundary-to-boundary artery with
angled avenues/cross streets and polygon blocks whose buildings follow
their frontage; `TBGWAF_MAP=urban-merge` adds a wide branching avenue that
merges into the artery, `TBGWAF_MAP=urban-elevated` (default) turns the artery into a
true overpass (ramps up, crosses on pier bents with usable ground beneath,
ramps back down; the branch becomes an on-ramp merging mid-deck), and
`TBGWAF_MAP=hilly` selects rolling hills. Every elevated span carries
raised sides (a guard parapet that is cover on the deck), and each seed
draws its bent form (issue #223): one thick T-bent column flaring into a
hammerhead cap under the deck, or twin columns under a widened deck joined
by a crossbeam cap; shafts stand on stepped footings and are real obstacles
(collision, LOS, navmesh), the caps overhead only. Urban code/scenarios can also
tune artery count/width, local-street width/skew and elevation through
`MapGeneratorConfig` / `map.generate`. Some seeds (`extraArteries`, 0–2 drawn
per seed) widen one or two avenues into extra arteries beside the main one,
two or three in all; they run parallel to it, so they never cross its deck,
and each always ramps through the levels it meets. Press **N** in-game to toggle a debug
overlay of the navmesh's walkable-cell boundaries.

Every arterial city has **terrain levels** (issue #155): 1–3 map-spanning
raised levels, drawn per seed. Each level is a straight step across the whole
map, roughly across the artery: everything on one side rises 1–5 figure
heights behind a sheer concrete cliff. The levels are drawn like the
buildings: each tier is an extruded polygon (flat top, vertical sides), a
ramp through a level is a rectangular cutout exactly the width of its
pavement, and a stair is one extruded trapezoid (a straight flight and a flat
landing at the level's height) attached to the cliff face. A level edge is a
block boundary like a street: a 6.4-unit band centered on it is carved from
the blocks on both sides, so sidewalk rings and buildings keep a street's
setback from the cliff. Steps overlap into nested tiers at different
heights, with sharp edges (no rolling noise). The sampled heightfield only
drives movement (an unseen nav wall along each cliff keeps figures off the
edge). The terrain is planned first;
the highway profile is then chosen against it per seed: the artery stays at
grade and ramps through every wall (a main road cutting through the levels),
or climbs onto a deck before a run of walls, flies over them clear of the
highest tier and settles onto the level beyond, with either ramp optionally
replaced by an elevated map edge, up to one tall deck above every level. The
branch merges onto whatever the artery is doing and ramps through the walls
it meets at grade. A local street never runs into a level or across a road
that is ramping through one: an avenue may ramp through a level; any other
street whose pavement would reach the level band or a ramp corridor stops at
the last street it crossed before it (a T, not an X) and resumes at the first
one beyond it, and the blocks either side of the missing stretch stay one
block; where that last street itself ends at the junction (an L), the street
instead runs on as a dead end stopping just short of the band or corridor, so
no street ever pokes into a block. Elongated blocks get one of two seeded
treatments: a length-scaled row of short, angled apartment slabs with broad
gaps around them, or a split into two or three smaller blocks divided by a
short local street segment running from one side of the old block to the other. Sidewalk
rings are one flat plane per strip, on the tier beside the cliff. Switch the
levels off with `TBGWAF_TERRAIN=off`,
`MapGeneratorConfig::features.terrain`, or scenario YAML:

```yaml
map:
  generate:
    type: urban
    seed: 7
    elevated: false
    features: {terrain: on}
    terrain_levels: -1 # seeded 1–3; 0 keeps the city flat
    extra_arteries: -1 # seeded 0–2 more arteries; 0 pins the single one
```

The legacy orthogonal generator stays flat. `city_terrain_seed_*` goldens
cover seeds 7, 42 and 2024 at grade and `city_terrain_elevated_seed_16` a
deck over the levels; the flat-city goldens pin terrain off.

Urban buildings have an **archetype** (`Scene::buildings`, one `Building` per
parcel beside its obstacle): residential, mixed-use, office, commercial hall,
garage, or civic for the two landmark towers. Each parcel's kind is drawn
from a table weighted by the street it fronts (artery frontage leans
office/mixed-use, back streets residential/warehouse), biased by its block's
neighbourhood character (downtown, residential, industrial or mixed; drawn per
block, or forced with `TBGWAF_BLOCK_CHARACTER=downtown` / scenario
`map.generate.character`). Floor count and floor height per kind set the
building height; residential, mixed-use and office heights follow a smooth
per-block skyline field (up to `skylineMaxHeight`, 24; landmark towers reach
28–40), so tall and low districts cluster; the renderer colours facades per kind and draws
massing -- parapets, roof furniture, awnings over shopfronts, set-back office
top floors. Massing blocks sight lines and FOV (`SightObstacles`) but not
movement; the set-back top floor is full obstacle geometry.

Buildings also carry **facade openings** (issue #152, `Scene::facadeOpenings`):
a per-kind window grid per floor on every street facade (shopfront glazing on
mixed-use ground floors, a sparser grid on the courtyard wall) and the doors,
each recorded as data (wall edge, u-range, floor, kind) and rendered as pane
extrusions -- window frames extruded out of the wall (with exterior sills on
some residential buildings, flatter thinner frames on office ribbon glazing),
while doors and the factory-style grid windows of commercial halls are
recessed into the building body through a real hole in the drawn wall face --
so a later stage can turn the same records into gameplay holes without
re-rolling where they are. Nearly every building gets a street
entrance on its frontage: some entrances climb a short stoop walk-up while
the rest open straight off the ground, and some buildings add a rear
courtyard door, usually raised on a little platform whose access ramp lands
on one of its three free sides (seed-drawn). Stoops and platforms are real
`WalkSurface`s -- knee-high concrete a figure can stand on -- while the
windows and doors themselves stay visual for now. Switch the openings off
with `MapGeneratorConfig::features.facades` or scenario YAML
`map.generate.features: {facades: off}`.

Urban maps carry pre-built **ziplines** (0-2 per block, seed-deterministic,
`MapGeneratorConfig::maxZiplinesPerBlock`): two-way cables between anchor
posts along the block sidewalks. A move plan may walk to one anchor, ride,
and walk on from the other; a ride costs `length * kZiplineCostFactor`
(0.25, in `game/Types.h`) of the figure's per-round move budget, so a longer
line spends more of the turn. The move frontier draws that reach as a second
(cyan) region with the line highlighted. One rider per line at a time (a
second rider waits at the anchor), and a rider can't shoot. Scenarios add
lines with `map.ziplines: [{from: [x,y,z], to: [x,y,z]}]` and assert an
out-of-reach move with `expect_unreachable: true`.
The app defaults to the prototype per-unit shadow-map FOV mask (tints
walls, roofs and deck sides too; see
[docs/fov-shadow-map.md](docs/fov-shadow-map.md)); `TBGWAF_FOV_SHADOW_MAP=0`
restores the analytic FOV-cone overlay. Visual scenarios still default to
the analytic overlay and opt in
with a `render: {fov_overlay: shadow_map}` block.

The debug panel's **AO** view (formerly Thermal view) is on by default and uses black-hot polarity by default, based
on the references in PR #178: pale clay-like surroundings, gray streets and
figures in their team color. Screen-space ambient occlusion adds soft contact
shading at building bases, parapets, roof furniture and feet. It samples
nearby visible geometry, so hidden/offscreen surfaces cannot contribute.
Uncheck **Black hot** to invert the gray world (figures keep their team
color). Apparent heat is assigned by material; this is a visual
approximation, not a temperature simulation. Light sensor blur and grain
affect the shaded world before the gameplay overlays are drawn in their
normal in-game colors (selection, paths, ropes, FOV); only the HUD switches
to green. The **FOV opacity** slider (while AO is on) sets the team FOV tint strength over
the thermal world (default 0.25; the normal view uses 0.15). The `thermal_urban_black_hot` and
`thermal_urban_white_hot` scenarios show both polarities on the same city;
scenario YAML selects them with `render: {thermal: true, thermal_black_hot: false}`
for white-hot (omit `thermal_black_hot` for black-hot).

The AO sliders in the debug panel tune the contact shading: radius (default
6.0), strength (1.0), quality (4 samples per ray) and softness (3 px blur).

### Flag objective (CTF part 1)

A scene (or scenario YAML `flag:` block) can enable a neutral flag at the map
center (or the nearest free spot; `flag: {position: [x, z]}` overrides). A
living figure whose move passes through the flag's spot picks it up for free
and its team wins at once; `win_on_grab: false` instead just carries it
(stowed on the figure's back) while play continues. A carrier acts as normal;
if it dies the flag drops there and anyone passing picks it up. If several
figures reach it in the same tick the earliest arrival wins, with an exact tie
going to the lowest unit id. The flag is a neutral objective, so fog never
hides it (a carried one shows with its carrier). `round_limit: N` ends the
match as a draw after N rounds with no winner. Flag state travels in the
snapshot text protocol (`ExportState`/`ImportState`).

### Builder

Game Mode → **Builder** shows one card per scene you have built (a rendered
thumbnail and its name; scenes live in memory for the session, and the first
card is a sample) plus a **New Scene** card. Opening one enters the editor: a
medium flat field (40 x 40) in 3D with a side panel to rename the scene, add
cubes and edit their position, size and height, and add Blue/Red figures and
pick each one's gun (Desert Eagle, Assault Rifle, Sniper Rifle). Drag a
figure in the 3D view to move it; it snaps onto the ground or a cube top
(every cube is climbable). **Start** (enabled once both teams have a figure
and every figure stands clear) plays the scene **hotseat** on one screen:
both teams' figures and FOV cones are shown, any figure can be clicked and
planned, and the HUD follows the selected figure's team; **Commit Round** once
every figure has a plan. **Download log** (during or after the game) exports
the session as a repro log -- the scene itself, the RNG seed, every input with
timestamps, each frame's `dt`, the simulation's own log lines and a state hash
after every entry ([docs/repro-log.md](docs/repro-log.md)); **Stop** returns to
the Builder (whose screen keeps the last game's log downloadable).
`build/tactics_repro_replay <log.jsonl>` replays a log headlessly and verifies
every hash; `TBGWAF_REPRO_LOG=<log.jsonl> build/tactics_app` loads it into the
app onto the turn timeline. On the web each canvas reaches the Builder from
its own menu; a builder session is local to that canvas (it neither sends nor
applies the two-canvas sync messages until it returns to Game Mode).
The fixed demo scenes of issue #240 (registry in `src/game/DemoScenes.cpp`)
are no longer listed in the app, but their logs still replay and scenario
YAML can still pin one with `map: {demo_scene: <id>}`
(`tests/scenarios/demo_odd_ragdoll_repro.yaml`).

### Gameplay scenario tests

`tests/scenarios/*.yaml` are state-only (no rendering) gameplay regression
tests: each declares a map + starting units, then a scripted sequence of
player-equivalent actions (`move`/`shoot`/`pass`/`cancel`, driven through the
same click/choose API the interactive game uses) interleaved with
assertions on the resulting state (alive/dead, position, per-team FOV
visibility, round number, winner). They run as the
`scenario_tests` ctest target; see `tests/scenario/Scenario.h` for the full
field reference and `tests/scenarios/*.yaml` for examples.

### Visual regression tests

The same scenarios also run in an expensive *visual* mode (the
`visual_tests` ctest target, headless under `xvfb-run` like the smoke
test): for every turn of every scenario, both teams' fog-of-war panes are
rendered through the exact same `gfx::SceneRenderer` pass the app uses and
captured as one PNG per team per turn, then diffed against the checked-in
goldens under `tests/goldens/` with a pixelmatch-style tolerance
(per-channel threshold + max differing-pixel fraction, so driver/AA noise
doesn't fail the build but object-sized changes do).

When a visual change is *intentional*, regenerate and commit the goldens:

```sh
scripts/update_golden_baselines.sh
```

The runner (`build/tactics_visual_tests`) can also record a continuous
per-team video of each scenario with `--video` (requires `ffmpeg`); see
`--help` for the flags.

With `--server build/tbgwaf_server_testctl --skip-goldens` the runner instead
hosts a real backend and films each team's own `RemoteClient` mirror (one
video per connected client), so the footage shows what a browser tab shows
(server fog filtering, replayed rounds, shot tracers). The review page
uses this mode; the `visual_server_tests` ctest runs it without video.
Assertions stay with `net_scenario_tests`; click markers and move-frontier
frames are not drawn in this mode, and timeline scenarios fall back to
in-process.

### Networked scenario tests

`net_scenario_tests` (`tests/net_scenario_tests.cpp`) replays the same YAML
scenarios through a **real `tbgwaf_server` process**, so the room, protocol,
serialization and fog-of-war paths are covered too. No GL; it runs headless
(also with `-DTBGWAF_BUILD_CLIENT=OFF`).

- The runner spawns `tbgwaf_server_testctl`: the server built with the
  loopback-only **control tap** (`--control-port`) compiled in. The shipped
  `tbgwaf_server` has no such flag. The tap speaks the same WebSocket framing as
  the game port and accepts `load_scenario` (scene + units, the same spec the
  YAML uses; clients receive it in `start`), `dump_state` (unfiltered server
  state) and `shutdown`.
- Per scenario it connects one headless client per team on the normal game port
  (`GameLogic` mirror + `net::RemoteClient` over a small native WebSocket client
  in `tests/net/`). Each scripted action runs through the **acting team's own
  mirror** (the same `ClickUnit`/`ChooseMove`/... calls as the in-process
  runner); `RemoteClient` ships the resulting plans, and a no-op `reaction` acts
  as a barrier so the next step starts after the server has applied them.
- Assertions run against the server's `dump_state` using the shared assertion
  code, plus checks that the server's fog matches a recomputation and that
  each client's mirror shows exactly what its team may see. Sighting-memory
  assertions run on the owning client's mirror (age 0 only; older ones are
  in-process only).
- All three runners advance rounds in the same fixed step
  (`constants::kSimStepSeconds`, 1/30 s), so reactions resolve identically.

```mermaid
flowchart LR
  Y[("tests/scenarios/*.yaml")]

  subgraph L["scenario_tests - in-process, logic only"]
    LR1[Scenario runner] --> LG[GameLogic]
  end

  subgraph V["visual_tests - xvfb + software GL"]
    VR[Visual runner] --> VG[GameLogic] --> SR[SceneRenderer]
    SR --> PNG[PNG goldens diff]
    SR --> VID[per-team videos]
  end

  subgraph N["net_scenario_tests - real server process, no GL"]
    NR[Net runner]
    NR -->|"click API, per acting team"| CB["Blue client: GameLogic + RemoteClient"]
    NR -->|"click API, per acting team"| CR["Red client: GameLogic + RemoteClient"]
    CB <-->|"WebSocket, game port"| SV
    CR <-->|"WebSocket, game port"| SV
    NR -->|"WebSocket, control tap: load_scenario, dump_state"| SV["tbgwaf_server_testctl<br/>Lobby + GameSession"]
    NR --> AS["assert on server state<br/>+ client fog mirrors"]
  end

  Y --> LR1
  Y --> VR
  Y --> NR

  subgraph U["Unit-level ctests"]
    UT["logic_tests, map_generator_tests,<br/>camera_tests, net_tests,<br/>server_tests (incl. real-socket loopback)"]
  end

  SM["smoke_test: tactics_app under xvfb"]
  CI["GitHub Actions ci.yml:<br/>flow sync check, full build + ctest,<br/>headless server-only build + ctest"]
  CI --> L
  CI --> V
  CI --> N
  CI --> U
  CI --> SM
```

### Weapons & the asset/animation gallery

Figures carry one of three procedural blocky weapons (issue #126), assigned
deterministically by unit id (`id % 3`): 0 = assault rifle, 1 = sniper
rifle, 2 = Desert Eagle — so every squad of three fields one of each. The
weapon sets the figure's animation class: rifles (AR + sniper) are carried
two-handed across the chest (both arms IK-solved onto the weapon, no arm
swing while running) and shoulder-aimed for shots; the Desert Eagle keeps
the one-handed low-ready carry and quick-draw shot. Weapons are visual
only — hit resolution is identical across them, though each weapon has its
own fire interval and bullet-scatter cone (every bullet of a burst flies its
own line).

Every weapon model and animation can be inspected on the **asset &
animation gallery**, a standalone page built alongside the game:
[live gallery](https://thedeepestspace.github.io/tbgwaf/gallery/) on Pages,
and `pr-preview/pr-<number>/gallery/` in each PR preview. The page lists
the three weapon turntables plus idle/run/shoot/empty-mag and downhill/uphill zipline rides per weapon — the
empty-mag clip dumps the whole magazine at the weapon's own fire interval,
each bullet leaving its own scattered tracer line (drag to orbit,
scroll to zoom, play/pause and scrub the loop). The catalog, framing, and
renderer live in `src/gallery/GalleryScene.*`, shared verbatim between the
web viewer (`src/gallery/gallery_main.cpp` + `web/gallery.html`) and a
native interactive build, `build/tactics_gallery_app` (left/right switch item, space pauses).

### PR scenario review page

Every pull request gets a **scenario review page** in its Pages preview:
per-team WebM videos of every scenario present at the PR's head, grouped by
how the PR changes each one (new/modified/unchanged, classified purely from
git metadata on the YAML files, not from screenshot diffs; deleted
scenarios are listed by name), generated by
`scripts/generate_scenario_review_page.py` and deployed to
`pr-preview/pr-<number>/scenarios/` by the same PR-preview workflow. The
workflow comments a direct link on the PR.

## Online play (server)

`tbgwaf_server` is a server-authoritative WebSocket backend: it pairs two
connections per room ("two sockets, one game"), holds the match in a headless
`tactics_logic` instance, and validates every action.

```sh
# Server only -- no SDL2/EGL/GLESv2 needed.
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release -DTBGWAF_BUILD_CLIENT=OFF
cmake --build build
./build/tbgwaf_server --port 8080     # PORT env var also works; GET /health -> 200
```

`-DTBGWAF_BUILD_CLIENT=OFF` skips the SDL2/GLES app and GL-based tests;
`-DTBGWAF_BUILD_SERVER=OFF` skips the server. Both default to ON.

**Local end-to-end test:** build the web client (below), serve the build dir,
and open it with a server URL (a bare port means `ws://localhost:<port>`):

```
http://localhost:8000/index.html?server=8080
```

With a server configured the page is split screen: the left and right panes
are two separate clients of the server, paired in a room private to the page
load, and the Human vs Human / Blue vs Jev / Jev vs Jev controls work as in the
local preview (the Jev side plans through the adapter and commits its own
team). Switching mode or Restart reloads into a fresh room. Known gap: the
server always plays its seed-driven map, so Jev vs Jev's capture-the-flag
default only applies to the no-server preview.

`?solo=1` shows a single pane instead, for two browsers: open
`index.html?server=ws://localhost:8080&solo=1&room=mygame` in both; the first
waits (Blue), the second is paired (Red). `?room=` picks the room (default for
`solo`: none -- use the Room screen or name one).

`?online=1` (or the panel's **Online (2 players)** button) opens the single-pane game; its
own menu (New Game -> **Online (2 players)**, shown only when a server is configured) leads to
the in-game Room screen: **New room** has the server pick an unused 5-character code (shown
in-game while waiting; share it), and **Join** enters an existing room by code (unknown or
full rooms are refused and the Room screen shows why). **Leave room** (top-left) reloads to the
start screen. The wire `join` message carries `"intent"`, and `waiting` echoes the `room`.
`?room=NAME` skips the menu and connects straight to that room. Without `?server=` the page runs the in-page demo
(`?local=1` forces it). A default server can be baked in at build time:
`emcmake cmake -B build-web -DTBGWAF_SERVER_URL=wss://...` (the query parameter
still overrides it). An `https://` page can only reach `wss://` (or
`ws://localhost`).

**Protocol** (JSON text frames; full reference in `src/net/Protocol.h`).
Clients send discrete actions -- `move` (waypoint legs + facing), `shoot`,
`pass`, `cancel`, `focus`, `reaction`, `commit`, `new_match` --
each validated through the same click/choose flow the game and scenario
tests use. The server replies with rule state only: `ack`, the team's own
`plans`, `peer` readiness, and after each round a fog-filtered `round`
timeline (sampled positions, shot events, resulting state). Hidden enemy
figures and enemy plans are never sent, and no animation fields cross the
wire -- the client derives walk/shoot/fall animation locally. A round runs
once both teams have committed.

### Performance suite

The **Performance suite** is an observational native/software-GL benchmark
for 10 pinned, randomly selected `urban-auto` map seeds. It initializes the
normal six figures and map-fitted two-pane camera, warms up the renderer, and
records 120 frames of the debug-panel measurements: FPS/frame time, FOV time,
draw calls, triangles, and vertices. Raw JSON retains every sample plus
per-map and whole-run percentiles; no timing value is a pass/fail CI gate.

Run the **Performance suite** workflow manually and choose a branch, tag, or
SHA. It builds Release under Xvfb/llvmpipe, uploads the raw result, preserves
the run in `gh-pages`, and refreshes the [published report](https://thedeepestspace.github.io/tbgwaf/performance/).
Every PR preview also measures the PR head and publishes `pr-preview/pr-<number>/performance/`, compared against the preserved master runs.
Master runs form the baseline shown beside measurements from other refs.
Locally, after a Release configure:

```sh
cmake --build build --target tactics_performance_suite
LIBGL_ALWAYS_SOFTWARE=1 SDL_VIDEODRIVER=x11 \
  xvfb-run --auto-servernum --server-args='-screen 0 1280x1024x24' \
  build/tactics_performance_suite --output performance-results.json
```

## Web (WASM) build

Requires the [Emscripten SDK](https://emscripten.io/docs/getting_started/downloads.html)
(already installed and on `PATH` in this repo's devcontainer):

```sh
emcmake cmake -B build-web -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build-web
```

This produces `build-web/tactics_app.{html,js,wasm}`. Serve the directory
with any static file server (e.g. `python3 -m http.server -d build-web`) and
open `tactics_app.html` -- it can't be opened as a `file://` URL, browsers
block WASM/fetch from local files. Pushes to `master` build and publish this
target to GitHub Pages automatically (`.github/workflows/pages.yml`).

Pull requests also get their own preview build, published to a per-PR
subdirectory (`pr-preview/pr-<number>/`) on the `gh-pages` branch
(`.github/workflows/pr-preview.yml`) without touching the production build.
The workflow comments on the PR with a link to the preview once it's ready,
and removes the preview automatically when the PR closes.

## App flow

Screen flow (Splash -> Game Mode -> Map Select / Builder -> Gameplay, plus the in-game Room screen), declared in
`flow/app_flow.yaml`. The build regenerates `flow/app_flow.mmd` from it; paste
that file's contents below if the flow changes.

```mermaid
%%{init: {"flowchart": {"htmlLabels": false}, "themeVariables": {"fontFamily": "Arial, sans-serif"}}}%%
flowchart TD
    splash("  Splash  ")
    game_mode("  Game Mode  ")
    map_select("  Map Select  ")
    builder("  Builder  ")
    builder_edit("  Builder Editor  ")
    room("  Room  ")
    gameplay("  Gameplay  ")
    splash -->|new_game| game_mode
    game_mode -->|select_regular| map_select
    game_mode -->|select_ctf| map_select
    game_mode -->|select_builder| builder
    game_mode -->|select_online| room
    game_mode -->|back| splash
    map_select -->|select_urban| gameplay
    map_select -->|select_hills| gameplay
    map_select -->|back| game_mode
    builder -->|open_builder_scene| builder_edit
    builder -->|exit_builder| game_mode
    builder_edit -->|start_builder_game| gameplay
    builder_edit -->|back_to_builder| builder
    gameplay -->|back_to_builder| builder
    room -->|create_room| gameplay
    room -->|join_room| gameplay
    room -->|back| game_mode
```

## Controls

- **Left click** one of your own figures (in your own viewport) to select
  it, then choose **Move**, **Shoot**, or **Pass** from the
  action menu.
  - Move: click a destination on the ground; the figure paths around
    obstacles via its navmesh. A move can only reach as far as the figure
    can run within one round's fixed execution window.
  - Shoot: click an enemy figure. Once the round executes, the shot fires
    the first instant the target is within your figure's forward-facing FOV
    cone with a clear line of sight -- including mid-round, if the target
    only walks into view while the round plays out. If that never happens
    before everything stops moving, the shot expires as a miss.
- **Right-click drag** to orbit the camera; **scroll** to zoom. Either side
  can freely look around its own viewport at any time.
- **Esc** cancels the current action/selection.

Rounds are simultaneous (WEGO): both teams plan every living figure
concurrently -- each viewport only accepts clicks for its own squad -- and a
single **Commit Round** then executes both sides' plans together, moves
animating concurrently and shots resolving continuously against live
FOV/line-of-sight. Two figures shooting each other in the same instant both
go down (wiping out both squads at once is a draw). Play continues round by
round until a team is eliminated.
