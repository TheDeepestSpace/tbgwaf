#!/usr/bin/env python3
"""Builds the PR gameplay-scenario review page (issue #14 stage 3, #105).

Classifies the gameplay scenarios touched by a PR as new/modified/deleted
purely from git metadata on the YAML files under tests/scenarios/ (never
from screenshot diffs), records a continuous per-team WebM video for every
scenario present at the PR's head — new, modified, and unchanged alike —
via the tactics_visual_tests runner, and emits a static HTML page embedding
the videos, grouped by classification. Deleted scenarios have nothing to
render and are listed by name only. The page is deployed into the existing
per-PR gh-pages preview by .github/workflows/pr-preview.yml.

Each scenario additionally gets a third column: a human-readable breakdown
of its script, grouped into rounds (the steps between `action: commit`
boundaries). The runner's --video mode emits a timing.json sidecar mapping
each executed action to its frame/timestamp in the (frame-identical) blue
and red videos; the page's JS drives both panes from one shared transport,
highlights the breakdown step currently playing, and seeks both videos to a
step's start when it is clicked. The scenario YAML is parsed with a small
built-in subset parser (block/flow mappings and sequences, plain scalars) so
the script keeps its no-dependency footprint; a scenario whose YAML falls
outside that subset just loses its breakdown column, never the page.

Must run with a working X display (CI wraps it in xvfb-run) and ffmpeg on
PATH. Always writes an index.html, even when the PR touches no scenarios.
"""

import argparse
import html
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

SCENARIO_DIR = "tests/scenarios"

PAGE_TEMPLATE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>Gameplay scenario review</title>
<style>
  body {{ font-family: system-ui, sans-serif; margin: 2rem auto; max-width: 1400px;
         padding: 0 1rem; background: #14161a; color: #e8e8e8; }}
  h1 {{ font-size: 1.5rem; }}
  h2 {{ border-bottom: 1px solid #3a3f47; padding-bottom: 0.3rem; margin-top: 2.5rem; }}
  h2 .count {{ color: #8a919c; font-weight: normal; font-size: 1rem; }}
  .scenario {{ margin: 1.5rem 0; }}
  .scenario h3 {{ margin-bottom: 0.2rem; }}
  .scenario .path {{ color: #8a919c; font-size: 0.85rem; font-family: monospace; }}
  .panes {{ display: flex; gap: 1rem; flex-wrap: wrap; margin-top: 0.8rem; }}
  .pane {{ flex: 1; min-width: 320px; max-width: 640px; }}
  .pane figcaption {{ margin-bottom: 0.3rem; font-weight: bold; }}
  .pane.blue figcaption {{ color: #6d9eeb; }}
  .pane.red figcaption {{ color: #e06666; }}
  video {{ width: 100%; background: #000; border: 1px solid #3a3f47; }}
  .note {{ color: #d0a24a; }}
  .empty {{ color: #8a919c; font-style: italic; }}
  .pane.breakdown {{ flex: 0 1 360px; min-width: 280px; }}
  .pane.breakdown figcaption {{ color: #b8c0cc; }}
  .rounds {{ position: relative; border: 1px solid #3a3f47; background: #1a1d22;
            padding: 0.4rem 0.6rem; max-height: 420px; overflow-y: auto;
            font-size: 0.9rem; }}
  .round-title {{ color: #8a919c; font-size: 0.78rem; text-transform: uppercase;
                 letter-spacing: 0.06em; margin: 0.5rem 0 0.15rem; }}
  .step {{ padding: 0.12rem 0.4rem; border-radius: 4px; }}
  .has-video .step {{ cursor: pointer; }}
  .has-video .step:hover {{ background: #262b33; }}
  .step.active {{ background: #2c3a55; box-shadow: inset 0 0 0 1px #4a6da8; }}
  .step .u {{ font-weight: bold; }}
  .step .u.blue {{ color: #6d9eeb; }}
  .step .u.red {{ color: #e06666; }}
  .transport {{ display: flex; align-items: center; gap: 0.7rem; margin-top: 0.6rem; }}
  .transport button {{ background: #262b33; color: #e8e8e8; border: 1px solid #3a3f47;
                      border-radius: 4px; width: 2.6rem; height: 1.8rem;
                      cursor: pointer; font-size: 0.8rem; }}
  .transport input[type="range"] {{ flex: 1; }}
  .transport .clock {{ font-family: monospace; font-size: 0.85rem; color: #8a919c; }}
</style>
</head>
<body>
<h1>Gameplay scenario review</h1>
<p>Scenarios under <code>{scenario_dir}</code>, classified against this PR from
git metadata on the YAML files. Every scenario present at the PR's head —
new, modified, and unchanged — gets a video: the full scripted scenario as
rendered for that team's fog-of-war pane, the PR's version (no base-branch
comparison). Both panes play in lockstep from the shared transport below
them; click a script step to jump both videos to it. Deleted scenarios are
listed by name only.</p>
{sections}
{page_js}
</body>
</html>
"""

# Plain string (not .format-ed), so the braces need no escaping. The blue
# video is the master clock: the red pane is re-pinned to it on every
# timeupdate (independent <video> elements drift if left to free-run, which
# is also why the native controls are hidden in favor of the one transport).
PAGE_JS = """<script>
(() => {
"use strict";
function fmtTime(t) {
  if (!isFinite(t) || t < 0) t = 0;
  return Math.floor(t / 60) + ":" + String(Math.floor(t % 60)).padStart(2, "0");
}
function init(sc) {
  const vids = Array.from(sc.querySelectorAll("video"));
  const steps = Array.from(sc.querySelectorAll(".step"));
  const btn = sc.querySelector(".playpause");
  const seek = sc.querySelector(".seek");
  const clock = sc.querySelector(".clock");
  const master = vids[0];
  if (!master || !btn || !seek || !clock) return;
  const followers = vids.slice(1);
  // timing.json: {fps, actions: [{index, frame, time}]}, `time` being the
  // exclusive end of that action's video segment (index 0 = initial state).
  let timing = null;
  fetch("media/" + sc.dataset.stem + "/timing.json")
    .then(r => (r.ok ? r.json() : null))
    .then(t => { timing = t; update(); })
    .catch(() => {});
  const playAll = () => { vids.forEach(v => v.play()); btn.textContent = "❚❚"; };
  const pauseAll = () => { vids.forEach(v => v.pause()); btn.textContent = "▶"; };
  const seekAll = t => { vids.forEach(v => { v.currentTime = t; }); update(); };
  btn.addEventListener("click", () => (master.paused ? playAll() : pauseAll()));
  seek.addEventListener("input", () => {
    if (master.duration) seekAll((seek.value / 1000) * master.duration);
  });
  master.addEventListener("ended", () => { seekAll(0); playAll(); });
  function update() {
    const t = master.currentTime || 0;
    if (master.duration) seek.value = Math.round((t / master.duration) * 1000);
    clock.textContent = fmtTime(t) + " / " + fmtTime(master.duration || 0);
    followers.forEach(v => {
      if (!v.seeking && Math.abs(v.currentTime - t) > 0.08) v.currentTime = t;
    });
    if (!timing || !timing.actions || !timing.actions.length) return;
    const entry = timing.actions.find(e => t < e.time) ||
                  timing.actions[timing.actions.length - 1];
    steps.forEach(s => {
      const active = Number(s.dataset.action) === entry.index;
      if (active && !s.classList.contains("active")) {
        // Keep the playing step in view, scrolling only the rounds box
        // (scrollIntoView could also yank the page itself).
        const box = s.closest(".rounds");
        if (box && (s.offsetTop < box.scrollTop ||
                    s.offsetTop + s.offsetHeight > box.scrollTop + box.clientHeight)) {
          box.scrollTop = s.offsetTop - box.clientHeight / 2;
        }
      }
      s.classList.toggle("active", active);
    });
  }
  master.addEventListener("timeupdate", update);
  master.addEventListener("seeked", update);
  master.addEventListener("loadedmetadata", update);
  steps.forEach(s => s.addEventListener("click", () => {
    if (!timing || !timing.actions) return;
    const n = Number(s.dataset.action);
    // A step's segment starts where the previous action's ended. A step with
    // no timing entry never played (the script failed earlier): don't seek.
    const prev = timing.actions.find(e => e.index === n - 1);
    if (n > 0 && !prev) return;
    seekAll(prev ? prev.time : 0);
  }));
}
document.querySelectorAll(".scenario.has-video").forEach(init);
})();
</script>"""


# --- Minimal YAML-subset parser (just enough for tests/scenarios/*.yaml:
# block mappings/sequences, flow mappings/sequences, plain scalars, comments).
# Raises ValueError on anything outside the subset; callers degrade to "no
# breakdown" rather than failing the page build.

def _strip_comment(line: str) -> str:
    in_single = in_double = False
    for i, c in enumerate(line):
        if c == "'" and not in_double:
            in_single = not in_single
        elif c == '"' and not in_single:
            in_double = not in_double
        elif c == "#" and not in_single and not in_double and (i == 0 or line[i - 1] in " \t"):
            return line[:i]
    return line


def _scalar(token: str):
    token = token.strip()
    if len(token) >= 2 and token[0] == token[-1] and token[0] in "'\"":
        return token[1:-1]
    if token in ("true", "True"):
        return True
    if token in ("false", "False"):
        return False
    if token in ("null", "~", ""):
        return None
    try:
        return int(token)
    except ValueError:
        pass
    try:
        return float(token)
    except ValueError:
        pass
    return token


def _skip_ws(s: str, i: int) -> int:
    while i < len(s) and s[i] in " \t":
        i += 1
    return i


def _parse_flow_value(s: str, i: int):
    i = _skip_ws(s, i)
    if i < len(s) and s[i] in "{[":
        return _parse_flow(s, i)
    j = i
    while j < len(s) and s[j] not in ",}]":
        j += 1
    return _scalar(s[i:j]), j


def _parse_flow(s: str, i: int):
    i = _skip_ws(s, i)
    if i >= len(s) or s[i] not in "{[":
        raise ValueError(f"expected flow collection at: {s[i:]!r}")
    closer = "}" if s[i] == "{" else "]"
    out = {} if closer == "}" else []
    i = _skip_ws(s, i + 1)
    if i < len(s) and s[i] == closer:
        return out, i + 1
    while True:
        if closer == "}":
            i = _skip_ws(s, i)
            j = s.find(":", i)
            if j < 0:
                raise ValueError(f"missing ':' in flow mapping: {s[i:]!r}")
            key = s[i:j].strip().strip("'\"")
            value, i = _parse_flow_value(s, j + 1)
            out[key] = value
        else:
            value, i = _parse_flow_value(s, i)
            out.append(value)
        i = _skip_ws(s, i)
        if i >= len(s):
            raise ValueError(f"unterminated flow collection: {s!r}")
        if s[i] == ",":
            i += 1
            continue
        if s[i] == closer:
            return out, i + 1
        raise ValueError(f"unexpected {s[i]!r} in flow collection: {s!r}")


def _parse_block(lines: list[tuple[int, str]], i: int, indent: int):
    if lines[i][1] == "-" or lines[i][1].startswith("- "):
        return _parse_block_seq(lines, i, indent)
    return _parse_block_map(lines, i, indent)


def _parse_block_map(lines: list[tuple[int, str]], i: int, indent: int):
    out: dict = {}
    while i < len(lines) and lines[i][0] == indent and not lines[i][1].startswith("- "):
        text = lines[i][1]
        key, sep, rest = text.partition(":")
        if not sep or " " in key.strip():
            raise ValueError(f"expected 'key: value', got: {text!r}")
        key = key.strip().strip("'\"")
        rest = rest.strip()
        if rest:
            out[key] = _parse_flow(rest, 0)[0] if rest[0] in "{[" else _scalar(rest)
            i += 1
        else:
            i += 1
            if i < len(lines) and lines[i][0] > indent:
                out[key], i = _parse_block(lines, i, lines[i][0])
            else:
                out[key] = None
    return out, i


def _parse_block_seq(lines: list[tuple[int, str]], i: int, indent: int):
    out: list = []
    while i < len(lines) and lines[i][0] == indent and (
            lines[i][1] == "-" or lines[i][1].startswith("- ")):
        rest = lines[i][1][2:].strip()
        if not rest:
            i += 1
            if i < len(lines) and lines[i][0] > indent:
                value, i = _parse_block(lines, i, lines[i][0])
                out.append(value)
            else:
                out.append(None)
        elif rest[0] in "{[":
            out.append(_parse_flow(rest, 0)[0])
            i += 1
        elif ":" in rest:
            # "- key: value" starts a mapping whose further keys sit two
            # columns in (right where the text after "- " starts): rewrite
            # this line without the dash and parse the mapping from there.
            lines[i] = (indent + 2, rest)
            value, i = _parse_block_map(lines, i, indent + 2)
            out.append(value)
        else:
            out.append(_scalar(rest))
            i += 1
    return out, i


def load_yaml_subset(text: str):
    lines: list[tuple[int, str]] = []
    for raw in text.splitlines():
        if "\t" in raw:
            raise ValueError("tabs are outside the supported YAML subset")
        stripped = _strip_comment(raw).rstrip()
        if not stripped.strip():
            continue
        lines.append((len(stripped) - len(stripped.lstrip(" ")), stripped.strip()))
    if not lines:
        return {}
    value, i = _parse_block(lines, 0, lines[0][0])
    if i != len(lines):
        raise ValueError(f"trailing unparsed content at: {lines[i][1]!r}")
    return value


# --- Script breakdown: YAML -> per-round action blocks. ---

def _fmt_num(v) -> str:
    if isinstance(v, float) and v.is_integer():
        return str(int(v))
    return str(v)


def _fmt_point(p) -> str:
    """[x, y, z] world point as ground coords "(x, z)", keeping y when off
    the ground (e.g. a destination on top of a crate)."""
    if not (isinstance(p, list) and len(p) == 3 and
            all(isinstance(c, (int, float)) for c in p)):
        return html.escape(str(p))
    x, y, z = p
    if y:
        return f"({_fmt_num(x)}, {_fmt_num(y)}, {_fmt_num(z)})"
    return f"({_fmt_num(x)}, {_fmt_num(z)})"


def _unit_label(units: dict, uid) -> str:
    team = units.get(uid)
    if team in ("blue", "red"):
        return f'<span class="u {team}">{team}{uid}</span>'
    return html.escape(f"unit {uid}")


def _action_text(step: dict, units: dict) -> str:
    kind = step.get("action")
    actor = _unit_label(units, step.get("actor"))
    if kind == "commit":
        return "commit round"
    if kind == "move":
        text = f"{actor} moves to {_fmt_point(step.get('destination'))}"
        waypoints = step.get("waypoints") or []
        if waypoints:
            text += " via " + ", ".join(_fmt_point(w) for w in waypoints)
        if step.get("final_facing_degrees") is not None:
            text += f", then faces {_fmt_num(step['final_facing_degrees'])}°"
        return text
    if kind == "shoot":
        text = f"{actor} shoots {_unit_label(units, step.get('target'))}"
        if step.get("expect_noop"):
            text += " (expected no-op: target not visible)"
        return text
    if kind == "pass":
        return f"{actor} passes"
    if kind == "cancel":
        return f"{actor} cancels"
    if kind == "focus":
        return f"double-click focus on {actor}"
    return html.escape(f"{kind} (actor {step.get('actor')})")


def build_breakdown_html(yaml_text: str) -> str:
    """Parses a scenario YAML into the rounds/steps HTML for the breakdown
    pane. Each executed action (assert steps don't count) gets a .step with
    data-action set to its 1-based index — the same index the visual
    runner's timing.json keys its video segments by; index 0 is the initial
    state. Raises ValueError on YAML outside the supported subset."""
    doc = load_yaml_subset(yaml_text)
    if not isinstance(doc, dict):
        raise ValueError("scenario YAML is not a mapping")
    units: dict = {}
    for unit in doc.get("units") or []:
        if isinstance(unit, dict) and "id" in unit:
            units[unit["id"]] = unit.get("team")
    rounds: list[list[str]] = []
    current: list[str] = []
    committed_rounds = 0
    action_index = 0
    for step in doc.get("script") or []:
        if not isinstance(step, dict) or "action" not in step:
            continue  # Assert-only steps don't execute (and have no timing).
        action_index += 1
        current.append(f'<div class="step" data-action="{action_index}">'
                       f"{_action_text(step, units)}</div>")
        if step["action"] == "commit":
            rounds.append(current)
            current = []
            committed_rounds += 1
    if not action_index:
        raise ValueError("scenario script has no actions")
    out = ['<div class="rounds">',
           '<div class="step" data-action="0">Initial state</div>']
    if current:  # Trailing actions never committed still form a block.
        rounds.append(current)
    for number, steps in enumerate(rounds, start=1):
        title = f"Round {number}"
        if number > committed_rounds:
            title += " (uncommitted)"
        out.append(f'<div class="round"><div class="round-title">{title}</div>')
        out.extend(steps)
        out.append("</div>")
    out.append("</div>")
    return "\n".join(out)


def build_breakdowns(paths: list[str], repo_root: Path) -> dict[str, str | None]:
    breakdowns: dict[str, str | None] = {}
    for path in paths:
        try:
            breakdowns[path] = build_breakdown_html((repo_root / path).read_text())
        except (OSError, ValueError) as e:
            print(f"[scenario-review] {path}: no script breakdown ({e})", file=sys.stderr)
            breakdowns[path] = None
    return breakdowns


def classify_changes(base: str, head: str, repo_root: Path) -> dict[str, list[str]]:
    """Returns {'A'|'M'|'D': [paths]} for scenario YAMLs changed base...head."""
    out = subprocess.run(
        ["git", "diff", "--name-status", "--no-renames", f"{base}...{head}", "--",
         f"{SCENARIO_DIR}/*.yaml", f"{SCENARIO_DIR}/*.yml"],
        cwd=repo_root, check=True, capture_output=True, text=True,
    ).stdout
    changes: dict[str, list[str]] = {"A": [], "M": [], "D": []}
    for line in out.splitlines():
        status, _, path = line.partition("\t")
        if status and status[0] in changes:
            changes[status[0]].append(path)
    return changes


def list_scenario_paths(ref: str, repo_root: Path) -> list[str]:
    """Returns paths of all scenario YAMLs present at ref."""
    out = subprocess.run(
        ["git", "ls-tree", "-r", "--name-only", ref, "--", SCENARIO_DIR],
        cwd=repo_root, check=True, capture_output=True, text=True,
    ).stdout
    return [p for p in out.splitlines() if p.endswith((".yaml", ".yml"))]


def record_videos(paths: list[str], runner: Path, repo_root: Path, media_dir: Path) -> dict:
    """Runs the visual runner with --video for each scenario; returns
    {path: {'stem', 'ok', 'detail'}}. A scenario whose script fails during
    playback still yields whatever video was captured before the failure."""
    results = {}
    for path in paths:
        stem = Path(path).stem
        with tempfile.TemporaryDirectory() as tmp:
            proc = subprocess.run(
                [str(runner), "--video", "--skip-goldens", "--out-dir", tmp, path],
                cwd=repo_root, capture_output=True, text=True,
            )
            ok = proc.returncode == 0
            detail = "" if ok else (proc.stderr.strip() or proc.stdout.strip())
            src = Path(tmp) / stem
            have_video = False
            if src.is_dir():
                dest = media_dir / stem
                dest.mkdir(parents=True, exist_ok=True)
                for team in ("blue", "red"):
                    video = src / f"{team}.webm"
                    if video.is_file():
                        shutil.copy2(video, dest / f"{team}.webm")
                        have_video = True
                timing = src / "timing.json"
                if have_video and timing.is_file():
                    shutil.copy2(timing, dest / "timing.json")
            results[path] = {"stem": stem, "ok": ok, "have_video": have_video,
                             "detail": detail}
            status = "ok" if ok else "FAILED"
            print(f"[scenario-review] {path}: {status}", file=sys.stderr)
            if detail:
                print(detail, file=sys.stderr)
    return results


def scenario_entry(path: str, info: dict | None, breakdown: str | None = None) -> str:
    name = html.escape(Path(path).stem)
    if info is None:  # Deleted: nothing to render.
        return (f'<div class="scenario"><h3>{name}</h3>'
                f'<div class="path">{html.escape(path)}</div></div>')
    stem = html.escape(info["stem"])
    classes = "scenario has-video" if info["have_video"] else "scenario"
    lines = [f'<div class="{classes}" data-stem="{stem}"><h3>{name}</h3>'
             f'<div class="path">{html.escape(path)}</div>']
    if not info["ok"]:
        note = "scenario script failed during visual playback"
        if info["have_video"]:
            note += "; video below stops at the failing step"
        lines.append(f'<p class="note">⚠ {html.escape(note)}.</p>')
    if info["have_video"] or breakdown:
        lines.append('<div class="panes">')
        if info["have_video"]:
            for team in ("blue", "red"):
                lines.append(
                    f'<figure class="pane {team}"><figcaption>{team.capitalize()} view'
                    f'</figcaption><video muted playsinline preload="metadata" '
                    f'src="media/{stem}/{team}.webm"></video></figure>')
        if breakdown:
            lines.append('<figure class="pane breakdown"><figcaption>Script'
                         f'</figcaption>{breakdown}</figure>')
        lines.append("</div>")
    if info["have_video"]:
        lines.append(
            '<div class="transport">'
            '<button class="playpause" aria-label="Play/pause both views">▶</button>'
            '<input class="seek" type="range" min="0" max="1000" value="0" step="1" '
            'aria-label="Seek both views">'
            '<span class="clock">0:00 / 0:00</span></div>')
    else:
        lines.append('<p class="note">⚠ No video could be recorded.</p>')
    lines.append("</div>")
    return "\n".join(lines)


def build_section(title: str, paths: list[str], results: dict | None,
                  breakdowns: dict[str, str | None] | None = None) -> str:
    body = [f'<h2>{html.escape(title)} <span class="count">({len(paths)})</span></h2>']
    if not paths:
        body.append('<p class="empty">None.</p>')
    for path in sorted(paths):
        if results is None:
            body.append(scenario_entry(path, None))
        else:
            body.append(scenario_entry(path, results.get(path),
                                       (breakdowns or {}).get(path)))
    return "\n".join(body)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", required=True, help="Base ref/sha of the PR")
    parser.add_argument("--head", required=True, help="Head ref/sha of the PR")
    parser.add_argument("--runner", required=True, type=Path,
                        help="Path to the built tactics_visual_tests binary")
    parser.add_argument("--out", required=True, type=Path,
                        help="Output directory for index.html + media/")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    args = parser.parse_args()

    changes = classify_changes(args.base, args.head, args.repo_root)
    all_paths = list_scenario_paths(args.head, args.repo_root)
    changed_paths = set(changes["A"]) | set(changes["M"])
    changes["U"] = [p for p in all_paths if p not in changed_paths]

    args.out.mkdir(parents=True, exist_ok=True)
    media_dir = args.out / "media"

    renderable = changes["A"] + changes["M"] + changes["U"]
    results = record_videos(renderable, args.runner.resolve(), args.repo_root, media_dir)
    breakdowns = build_breakdowns(renderable, args.repo_root)

    if not any(changes.values()):
        sections = '<p class="empty">This PR does not touch any gameplay scenarios.</p>'
    else:
        sections = "\n".join([
            build_section("New scenarios", changes["A"], results, breakdowns),
            build_section("Modified scenarios", changes["M"], results, breakdowns),
            build_section("Unchanged scenarios", changes["U"], results, breakdowns),
            build_section("Deleted scenarios", changes["D"], None),
        ])

    (args.out / "index.html").write_text(
        PAGE_TEMPLATE.format(scenario_dir=SCENARIO_DIR, sections=sections,
                             page_js=PAGE_JS))
    print(f"[scenario-review] wrote {args.out / 'index.html'} "
          f"({len(changes['A'])} new, {len(changes['M'])} modified, "
          f"{len(changes['U'])} unchanged, {len(changes['D'])} deleted)", file=sys.stderr)
    # Video failures are surfaced on the page itself; the page build only
    # fails if git classification failed (raised above).
    return 0


if __name__ == "__main__":
    sys.exit(main())
