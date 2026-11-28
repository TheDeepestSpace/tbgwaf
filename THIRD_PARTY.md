# Third-party assets

## RobotExpressive animation curves

The procedural figure rig in `src/gfx/FigureRig.cpp` includes compact,
retargeted samples from the `Idle` and `Walking` clips in
[`RobotExpressive.glb`](https://github.com/mrdoob/three.js/tree/b924f0cad4058dc4dde71445c796980c3cd5b5ed/examples/models/gltf/RobotExpressive).
Only eight scalar pose samples per loop are included; the source mesh and GLB
are not redistributed.

- Model and animation: Tomás Laulhé (Quaternius)
- Modifications and glTF conversion: Don McCurdy
- License: [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)
- Source revision: `b924f0cad4058dc4dde71445c796980c3cd5b5ed`

The samples were obtained by evaluating the glTF joint hierarchy at eight
evenly spaced times, projecting the upper/lower arm and leg vectors into the
sagittal plane, and storing the resulting joint angles plus normalized body
height (the same procedure `scripts/extract_biped_clip.py` now automates).
Runtime interpolation and all proportion retargeting are implemented
locally in `FigureRig.cpp`.

## Universal Animation Library cover clips

The cover poses in `src/gfx/FigureRig.cpp` (issue #290) include compact,
retargeted samples from two clips of the Quaternius Universal Animation
Library packs:

- `Crouch_Idle_Loop` from `UAL1_Standard.glb`, Universal Animation Library
  (Standard), tactical-cover crouch;
- `Idle_FoldArms_Loop` from `UAL2_Standard.glb`, Universal Animation Library 2
  (Standard), flat-cover lean (legs/spine only; the arms keep carrying the
  weapon).

The tactical-peek beat (issue #291) additionally includes a compact,
retargeted sample of:

- `Crouch_Fwd_Loop` from `UAL1_Standard.glb`, Universal Animation Library
  (Standard), the crouched creep used while the figure slides out past the
  corner and back (legs/spine only; the arms keep carrying the weapon, and
  the aimed hold reuses the `Crouch_Idle_Loop` samples above).

Only eight scalar pose samples per joint curve are included, produced with
`scripts/extract_biped_clip.py`; the source meshes, rigs and GLBs are not
redistributed.

Mixamo (Adobe) was considered as a source for the peek clip and rejected:
its animations are royalty-free for use *in* a project, but Adobe's terms do
not clearly permit redistributing the animation data itself (even
retargeted) in a public open-source repository, and downloads require an
Adobe account. The Quaternius packs are CC0, which is unambiguous.

- Animations: Quaternius
- Source: [Universal Animation Library](https://quaternius.itch.io/universal-animation-library)
  and [Universal Animation Library 2](https://quaternius.itch.io/universal-animation-library-2)
  (Standard zips, as published November 2026)
- License: [CC0 1.0](https://creativecommons.org/publicdomain/zero/1.0/)
