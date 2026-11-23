# Third-party assets

## RobotExpressive animation curves

The procedural figure rig in `src/gfx/SceneRenderer.cpp` includes compact,
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
height. Runtime interpolation and all proportion retargeting are implemented
locally in `SceneRenderer.cpp`.
