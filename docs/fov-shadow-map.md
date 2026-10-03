# Shadow-map projective FOV mask (prototype, issue #110)

Follow-up to #106. The analytic CPU FOV overlay (`DrawFovCone`) casts
corner rays and stitches ground quads around box/deck shadows; it covers
ground and deck tops only and would go combinatorial with wall faces. This
prototype replaces it with a per-unit projective mask: render scene depth
from the unit's eye, then tint every fragment of the static scene that lies
inside the cone and passes the depth test. It is **off by default**; the CPU
path stays as it was.

## How to turn it on

- App: `TBGWAF_FOV_SHADOW_MAP=1 ./build/tactics_app` (optionally
  `TBGWAF_FOV_PROBE_HEIGHT=<units>`, see "Ground semantics").
- Scenario YAML: a top-level `render: {fov_overlay: shadow_map}` block
  (plus optional `fov_probe_height`). Two scenarios use it and have goldens:
  `tests/scenarios/fov_shadow_map_elevated_highway_ramp.yaml` (identical to
  `elevated_highway_ramp` apart from the render block, so the two golden
  sets are directly comparable) and
  `tests/scenarios/fov_shadow_map_building_walls.yaml` (hand-authored
  blocks showing wall and roof highlighting).
- Visual runner: `--fov-shadow-map` forces the mask for every scenario
  (goldens then won't match); `--profile` prints per-pane render timings.

## What it does

Per pane, for each living unit of the pane's team
(`SceneRenderer::DrawFovShadowMask`):

1. Two eye-space depth passes into 1024x2048 `DEPTH_COMPONENT24` maps, one
   per half of the 150-degree cone (each 78 degrees wide, so they overlap
   by 3 degrees; 120 degrees tall). Casters: the ground box or terrain
   mesh, and a static mesh of obstacles, decks/ramps as 0.45-thick slabs
   (the `LineOfSightClear` slab thickness, same as the lit pass), sidewalk
   slabs/patches and roads. The static mesh is built once per scene
   (content-hashed) rather than re-tessellated per draw. Units are not
   casters: gameplay LOS ignores them, and so does the CPU overlay.
2. One mask pass over the same geometry into the pane: depth test `LEQUAL`
   with a small polygon offset, no depth writes, the same stencil trick as
   the CPU path so teammates' masks don't stack. The fragment shader
   discards fragments facing away from the eye, fragments outside the
   analytic 150-degree azimuth cone (so the cone's angular edges stay
   crisp, as today), and fragments whose eye sightline fails a 3x3 PCF
   depth test in linearized depth. Everything else gets the flat
   team-colour tint at the CPU path's alpha. No attenuation, no N.L.

## Findings

### A precision trap worth knowing about

The first renders had wide, evenly spaced bands on every surface more than
a few units from the viewer, which no bias short of +5 world units removed.
Instrumenting the shader (writing `stored - fragment` depth to colour and
reading pixels back) showed the stored depth is a sawtooth whose step is
~0.08 units at 4 units out and ~0.5 units at 15 units out: an 11-bit
mantissa, i.e. half precision. GLSL ES defaults fragment-shader samplers to
`lowp`, and on Mesa llvmpipe a `lowp sampler2D` depth read comes back at
fp16. Declaring the samplers `highp sampler2D` fixed it completely. The
existing directional shadow map uses a default-precision sampler too; its
orthographic projection and comparatively large NDC bias (0.0008-0.003,
which is 0.1-1.3 world units over the generated maps' light depth range)
presumably give it enough slack, which is why nothing showed there, but it
is the same hazard and worth a `highp` of its own some day (not changed
here, to keep the existing goldens untouched).

### Edge quality

Occlusion edges are shadow-map-resolution limited, as expected. The mask
covers each map's 120-degree vertical range with 2048 rows, so the texel
footprint along flat ground is `(2 tan 60deg / 2048) * t^2 / eyeHeight`:
about 0.11 units at 10 units out, 0.45 at 20 and 1.0 at 30. The azimuth
direction is 20x finer (`2 tan 39deg / 1024 * t`, 0.05 units at 30). So
radial shadow boundaries (an obstacle's ground shadow seen end-on) are
clean, while boundaries that run across the view direction stair-step at
distance: with a deliberately huge bias the staircase on the far building
shadow in `fov_shadow_map_elevated_highway_ramp` turn 0 was plainly visible
at ~15 px per step; the 3x3 PCF in the shipped shader softens this into a
one-texel-wide ramp rather than removing it. The cone's angular edges are
analytic and as crisp as today.

Acne handling after the precision fix: a normal-offset of 2 texels scaled
by sin(theta) plus a slope-scaled linear-depth bias capped at 5 texels.
With that, no acne is visible in either golden set, including the ramp top
seen at a grazing angle from its foot (the worst case). The price is that
a surface's shadow boundary moves outward by roughly the offset; at 20
units that is of the same order as the texel footprint (0.5 units), so it
is not the limiting factor.

Coverage gaps that the CPU path doesn't have: nothing steeper than 60
degrees below or above the eye is in either map, so a standing viewer has an
untinted disc of radius 0.87 units at its feet (visible as a small triangle
at the figure in a CPU-vs-mask diff) and a wall closer than
`0.58 * (wallTop - 1.5)` units is untinted above the 60-degree line.

How close is it to the CPU path on what the CPU path covers? Running the
CPU variant of each scenario against the shadow-map goldens with the
harness's default tolerance (per-channel 25, pixelmatch-style): the ramp
scenario's Blue frames differ in 1.02% of pixels on turns 0-2 and are
within the 0.2% budget on turn 3 (viewer on the ramp); the diff image is
edge slivers along the cone/deck boundaries, the tinted building wall on
the right, and the feet disc. The building scenario's Blue frames differ
by 3.7% -- the diff image is exactly the four eye-facing walls, the feet
disc and a one-pixel sliver along the cone's angular edge; the Red frames
(no walls in Red's cone) are within budget. Note the 25-per-channel
threshold hides the roof tint: alpha 0.15 over the light roof moves the
pixel by (12, 3, 14), so roof differences are not counted in these
percentages even though they are plainly visible in the goldens.

### Perf per unit / pane

Measured with `tactics_visual_tests --profile` (glFinish-bracketed
`RenderPane`, 640x720 pane) on **Mesa llvmpipe**, 4 cores of an AMD EPYC
9V74, i.e. CPU software rasterization. No GPU or WebGL2 numbers: this
environment has no GPU and no Emscripten toolchain.

| scenario (own units/pane)                | CPU overlay | shadow map | delta / unit |
|------------------------------------------|------------:|-----------:|-------------:|
| elevated_highway_ramp, Blue / Red (1)    | 30.2 / 22.7 ms | 39.9 / 35.4 ms | +10 / +13 ms |
| building_walls, Blue / Red (1)           | 8.1 / 7.2 ms   | 21.9 / 20.7 ms | +14 / +14 ms |
| default_scene_shoot_rows, Blue / Red (2.9 avg) | 17.2 / 14.7 ms | 50.5 / 47.9 ms | +11.6 ms |

So about **10-14 ms per unit per pane under llvmpipe**: two 2-megatexel
depth rasterizations plus a full-pane 9-tap mask pass. On a GPU the same
work is a few draw calls with trivial fill, and the obvious reductions are
not implemented yet: the maps are re-rendered every frame although a unit's
eye only moves while a round executes (cache per unit, invalidate on
move/turn/scene change), and the three units of a team could share one
texture array and a single mask pass. Depth-map memory is fixed at 16 MB
(allocated even in CPU mode so the mode can be flipped at runtime).

### Ground semantics

The issue frames the CPU overlay as a gameplay rule ("where could a crouched
target hide") rather than pure LOS. Looking at `GroundShadow` /
`DeckSpans`, the rule it implements is exactly "is the ground point itself
visible from the eye", with decks as 0.45 slabs. That is the same test the
mask performs when the probe height is 0, so by default the prototype
keeps the CPU semantics on ground and deck tops; the only differences are
the ones listed above plus one approximation the CPU path makes and the
mask doesn't (deck tops are not occluded by obstacles or other decks there;
they are here).

To evaluate the alternative, `fovProbeHeight` tests the sightline to a
point that high above every upward-facing fragment. Rendering the building
scene with `fov_probe_height: 1.5` (eye height): the ground strip behind
the 1.0-tall block disappears entirely (a standing target there is seen
over the block) and the tall block's ground shadow shortens to the region
where even a 1.5-high point is hidden. Walls and roofs are unaffected.

Decision: keep probe height 0 as the default. It matches the existing
overlay, it is the conservative reading a player wants ("the enemy might be
prone there"), and it stays consistent with the obstacle-visibility
sampling in `ComputeTeamVisibility`. Pure target-LOS is a one-line setting
if the gameplay rule changes.

### Recommendation on retiring the CPU path

Not yet. The mask reproduces the CPU overlay on ground and deck tops to
within ~1% of pane pixels and adds walls, roofs, deck sides and correct
deck/obstacle occlusion of deck tops, with none of the ray machinery -- so
the direction is right. Before it can replace the CPU path:

1. Measure on a real GPU and in the WebGL2 build (the primary target). The
   llvmpipe cost (10-14 ms/unit/pane) is not representative, but the CI
   visual tests also run on llvmpipe and a 3-unit team would push the
   visual suite's per-frame time to ~50 ms per pane.
2. Cache per-unit depth maps and only re-render when the unit moves or
   turns; in a WEGO game that is a small fraction of frames.
3. Decide whether 1-unit radial edge resolution at 30 units is acceptable
   or needs a warped/cascaded projection or a larger map.
4. Cover the feet disc and the >60-degree band (a third, downward map, or
   accept the gap).

Until then the mask is a flag-gated prototype with its own goldens, and the
CPU overlay remains the default.
