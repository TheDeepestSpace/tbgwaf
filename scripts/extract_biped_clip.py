#!/usr/bin/env python3
"""Retarget a skeletal clip from a glTF binary into FigureRig BipedCurve rows.

Samples a named animation clip at N evenly spaced times, projects the limb
bones into the character's sagittal plane, and prints the per-joint angles in
the convention used by src/gfx/FigureRig.cpp: degrees, 0 = bone hanging
straight down, positive = swung forward; knee/elbow angles are relative to
their parent bone. Also prints the spine pitch (forward lean, degrees), spine
side lean (toward the character's right, degrees) and the pelvis height
normalized by its bind height (for bob/drop curves).

This is how the Idle/Walking curves were originally produced from
RobotExpressive.glb, and how the cover crouch/lean curves were produced from
the Quaternius Universal Animation Library (see THIRD_PARTY.md).

Usage:
  python3 -I scripts/extract_biped_clip.py <file.glb> [clip_name] [samples]

With no clip name, lists the clips in the file. Only needs the stdlib.

Bone names are resolved against a few known skeleton conventions (Unreal
mannequin-style as used by Quaternius UAL, and RobotExpressive).
"""

import json
import math
import struct
import sys

# Candidate node names per logical joint, first match wins.
BONES = {
    "pelvis": ["pelvis", "Hips", "Torso"],
    "spine_top": ["spine_03", "Spine2", "Neck"],
    "thigh_l": ["thigh_l", "LeftUpLeg", "UpperLegL"],
    "calf_l": ["calf_l", "LeftLeg", "LowerLegL"],
    "foot_l": ["foot_l", "LeftFoot", "FootL"],
    "ball_l": ["ball_l", "LeftToeBase"],
    "thigh_r": ["thigh_r", "RightUpLeg", "UpperLegR"],
    "calf_r": ["calf_r", "RightLeg", "LowerLegR"],
    "foot_r": ["foot_r", "RightFoot", "FootR"],
    "ball_r": ["ball_r", "RightToeBase"],
    "upperarm_l": ["upperarm_l", "LeftArm", "UpperArmL"],
    "lowerarm_l": ["lowerarm_l", "LeftForeArm", "LowerArmL"],
    "hand_l": ["hand_l", "LeftHand", "HandL"],
    "upperarm_r": ["upperarm_r", "RightArm", "UpperArmR"],
    "lowerarm_r": ["lowerarm_r", "RightForeArm", "LowerArmR"],
    "hand_r": ["hand_r", "RightHand", "HandR"],
}

COMPONENT_FMT = {5120: "b", 5121: "B", 5122: "h", 5123: "H", 5125: "I", 5126: "f"}
TYPE_COUNT = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def load_glb(path):
    with open(path, "rb") as f:
        magic, _, _ = struct.unpack("<III", f.read(12))
        assert magic == 0x46546C67, "not a GLB"
        clen, ctype = struct.unpack("<II", f.read(8))
        doc = json.loads(f.read(clen))
        blen, btype = struct.unpack("<II", f.read(8))
        blob = f.read(blen)
    return doc, blob


def read_accessor(doc, blob, index):
    acc = doc["accessors"][index]
    view = doc["bufferViews"][acc["bufferView"]]
    comps = TYPE_COUNT[acc["type"]]
    fmt = COMPONENT_FMT[acc["componentType"]]
    size = struct.calcsize(fmt)
    offset = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride", comps * size)
    out = []
    for i in range(acc["count"]):
        base = offset + i * stride
        out.append(struct.unpack_from("<" + fmt * comps, blob, base))
    return out


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_rotate(q, v):
    qv = (v[0], v[1], v[2], 0.0)
    qc = (-q[0], -q[1], -q[2], q[3])
    r = quat_mul(quat_mul(q, qv), qc)
    return (r[0], r[1], r[2])


def quat_slerp(a, b, t):
    dot = sum(x * y for x, y in zip(a, b))
    if dot < 0.0:
        b = tuple(-x for x in b)
        dot = -dot
    if dot > 0.9995:
        out = tuple(x + t * (y - x) for x, y in zip(a, b))
    else:
        theta = math.acos(max(-1.0, min(1.0, dot)))
        sa = math.sin((1.0 - t) * theta) / math.sin(theta)
        sb = math.sin(t * theta) / math.sin(theta)
        out = tuple(sa * x + sb * y for x, y in zip(a, b))
    norm = math.sqrt(sum(x * x for x in out))
    return tuple(x / norm for x in out)


def sample_channel(times, values, t, is_rotation):
    if t <= times[0]:
        return values[0]
    if t >= times[-1]:
        return values[-1]
    hi = 1
    while times[hi] < t:
        hi += 1
    lo = hi - 1
    span = times[hi] - times[lo]
    f = 0.0 if span <= 0.0 else (t - times[lo]) / span
    if is_rotation:
        return quat_slerp(values[lo], values[hi], f)
    return tuple(a + f * (b - a) for a, b in zip(values[lo], values[hi]))


class Skeleton:
    def __init__(self, doc, blob):
        self.doc = doc
        self.blob = blob
        self.nodes = doc["nodes"]
        self.parent = [-1] * len(self.nodes)
        for i, n in enumerate(self.nodes):
            for c in n.get("children", []):
                self.parent[c] = i
        self.by_name = {n.get("name", ""): i for i, n in enumerate(self.nodes)}
        self.joints = {}
        for key, names in BONES.items():
            for name in names:
                if name in self.by_name:
                    self.joints[key] = self.by_name[name]
                    break

    def clip(self, name):
        for a in self.doc.get("animations", []):
            if a.get("name") == name:
                return a
        raise SystemExit(f"clip {name!r} not found")

    def pose(self, anim, t):
        """World position of every node at time t (None = bind pose)."""
        trans = {}
        rot = {}
        scale = {}
        if anim is not None:
            for ch in anim["channels"]:
                tgt = ch["target"]
                node = tgt.get("node")
                if node is None:
                    continue
                smp = anim["samplers"][ch["sampler"]]
                times = [v[0] for v in read_accessor(self.doc, self.blob, smp["input"])]
                values = read_accessor(self.doc, self.blob, smp["output"])
                path = tgt["path"]
                v = sample_channel(times, values, t, path == "rotation")
                if path == "translation":
                    trans[node] = v
                elif path == "rotation":
                    rot[node] = v
                elif path == "scale":
                    scale[node] = v
        world = {}

        def resolve(i):
            if i in world:
                return world[i]
            n = self.nodes[i]
            tr = trans.get(i, tuple(n.get("translation", (0, 0, 0))))
            ro = rot.get(i, tuple(n.get("rotation", (0, 0, 0, 1))))
            sc = scale.get(i, tuple(n.get("scale", (1, 1, 1))))
            p = self.parent[i]
            if p < 0:
                prot, ppos, pscale = (0, 0, 0, 1), (0, 0, 0), (1, 1, 1)
            else:
                prot, ppos, pscale = resolve(p)
            local = tuple(tr[k] * pscale[k] for k in range(3))
            off = quat_rotate(prot, local)
            pos = tuple(ppos[k] + off[k] for k in range(3))
            wrot = quat_mul(prot, ro)
            wscale = tuple(pscale[k] * sc[k] for k in range(3))
            world[i] = (wrot, pos, wscale)
            return world[i]

        return {i: resolve(i)[1] for i in range(len(self.nodes))}


def main():
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    doc, blob = load_glb(sys.argv[1])
    if len(sys.argv) < 3:
        for a in doc.get("animations", []):
            print(a.get("name"))
        return
    clip_name = sys.argv[2]
    samples = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    skel = Skeleton(doc, blob)
    anim = skel.clip(clip_name)
    duration = 0.0
    for smp in anim["samplers"]:
        times = read_accessor(doc, blob, smp["input"])
        duration = max(duration, times[-1][0])

    missing = [k for k in ("pelvis", "thigh_l", "calf_l", "foot_l") if k not in skel.joints]
    if missing:
        raise SystemExit(f"skeleton missing joints: {missing}")
    bind = skel.pose(None, 0.0)
    pelvis_bind_y = bind[skel.joints["pelvis"]][1]

    def joint(pose, key):
        return pose[skel.joints[key]]

    curves = {}

    def put(name, value):
        curves.setdefault(name, []).append(value)

    for s in range(samples):
        t = duration * s / samples
        pose = skel.pose(anim, t)
        # Character forward: horizontal toe direction, averaged over both feet
        # (falls back to world -Z if the skeleton has no toe bones).
        fwd = [0.0, 0.0, -1.0]
        if "ball_l" in skel.joints and "ball_r" in skel.joints:
            bl = joint(pose, "ball_l")
            fl = joint(pose, "foot_l")
            br = joint(pose, "ball_r")
            fr = joint(pose, "foot_r")
            fwd = [bl[0] - fl[0] + br[0] - fr[0], 0.0, bl[2] - fl[2] + br[2] - fr[2]]
        norm = math.hypot(fwd[0], fwd[2])
        fwd = [fwd[0] / norm, 0.0, fwd[2] / norm]
        right = [-fwd[2], 0.0, fwd[0]]

        def sagittal(a, b):
            """Angle of bone a->b from straight-down, degrees, +forward."""
            v = [b[k] - a[k] for k in range(3)]
            fx = v[0] * fwd[0] + v[2] * fwd[2]
            fy = v[1]
            return math.degrees(math.atan2(fx, -fy))

        def lateral(a, b):
            """Angle of bone a->b from straight-up, degrees, +toward right."""
            v = [b[k] - a[k] for k in range(3)]
            rx = v[0] * right[0] + v[2] * right[2]
            ry = v[1]
            return math.degrees(math.atan2(rx, ry))

        for side in ("l", "r"):
            hip = sagittal(joint(pose, f"thigh_{side}"), joint(pose, f"calf_{side}"))
            knee = sagittal(joint(pose, f"calf_{side}"), joint(pose, f"foot_{side}")) - hip
            put(f"hip_{side}", hip)
            put(f"knee_{side}", knee)
            if f"upperarm_{side}" in skel.joints and f"hand_{side}" in skel.joints:
                sh = sagittal(joint(pose, f"upperarm_{side}"), joint(pose, f"lowerarm_{side}"))
                el = sagittal(joint(pose, f"lowerarm_{side}"), joint(pose, f"hand_{side}")) - sh
                put(f"shoulder_{side}", sh)
                put(f"elbow_{side}", el)
        if "spine_top" in skel.joints:
            pelvis = joint(pose, "pelvis")
            top = joint(pose, "spine_top")
            # Spine pitch reuses the sagittal convention but measures from
            # straight-up: +forward lean.
            v = [top[k] - pelvis[k] for k in range(3)]
            fx = v[0] * fwd[0] + v[2] * fwd[2]
            put("spine_pitch", math.degrees(math.atan2(fx, v[1])))
            put("spine_side", lateral(pelvis, top))
        put("pelvis_height", joint(pose, "pelvis")[1] / pelvis_bind_y)

    print(f"// {clip_name}: {samples} samples over {duration:.3f}s")
    for name, vals in curves.items():
        digits = 3 if name == "pelvis_height" else 1
        row = ", ".join(f"{v:.{digits}f}" for v in vals)
        print(f"{name:>14}: {{{row}}}")


if __name__ == "__main__":
    main()
