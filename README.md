# tbgwaf

A turn-based tactics prototype: two 3-figure squads (Blue vs Red) fight on a
blocky obstacle field until one side is eliminated.

## Build

```sh
cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Run the game with `./build/tactics_app`. `ctest` also runs a headless
smoke test under `xvfb-run` if available.

## Controls

- **Left click** a highlighted figure (whoever's turn it is) to select it,
  then choose **Move**, **Shoot**, or **Pass** from the action menu.
  - Move: click a destination on the ground; the figure paths around
    obstacles via its navmesh and walks there.
  - Shoot: click an enemy figure. It's a hit (one-shot kill) only if the
    target is within your figure's forward-facing FOV cone and there's a
    clear line of sight; otherwise it's a miss. Either way the action is
    consumed.
- **Right-click drag** to orbit the camera; **scroll** to zoom.
- **Esc** cancels the current action/selection.

Turns strictly alternate Blue/Red (skipping eliminated figures) until every
figure on one team is dead.
