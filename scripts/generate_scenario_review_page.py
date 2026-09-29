#!/usr/bin/env python3
"""Builds the PR gameplay-scenario review page (issue #14 stage 3).

Classifies the gameplay scenarios touched by a PR as new/modified/deleted
purely from git metadata on the YAML files under tests/scenarios/ (never
from screenshot diffs), records a continuous per-team WebM video for every
scenario present at the PR's head — new, modified, and unchanged alike —
via the tactics_visual_tests runner, and emits a static HTML page embedding
the videos, grouped by classification. Deleted scenarios have nothing to
render and are listed by name only. The page is deployed into the existing
per-PR gh-pages preview by .github/workflows/pr-preview.yml.

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
</style>
</head>
<body>
<h1>Gameplay scenario review</h1>
<p>Scenarios under <code>{scenario_dir}</code>, classified against this PR from
git metadata on the YAML files. Every scenario present at the PR's head —
new, modified, and unchanged — gets a video: the full scripted scenario as
rendered for that team's fog-of-war pane, the PR's version (no base-branch
comparison). Deleted scenarios are listed by name only.</p>
{sections}
</body>
</html>
"""


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
            results[path] = {"stem": stem, "ok": ok, "have_video": have_video,
                             "detail": detail}
            status = "ok" if ok else "FAILED"
            print(f"[scenario-review] {path}: {status}", file=sys.stderr)
            if detail:
                print(detail, file=sys.stderr)
    return results


def scenario_entry(path: str, info: dict | None) -> str:
    name = html.escape(Path(path).stem)
    lines = [f'<div class="scenario"><h3>{name}</h3>'
             f'<div class="path">{html.escape(path)}</div>']
    if info is None:  # Deleted: nothing to render.
        lines.append("</div>")
        return "\n".join(lines)
    if not info["ok"]:
        note = "scenario script failed during visual playback"
        if info["have_video"]:
            note += "; video below stops at the failing step"
        lines.append(f'<p class="note">⚠ {html.escape(note)}.</p>')
    if info["have_video"]:
        lines.append('<div class="panes">')
        for team in ("blue", "red"):
            lines.append(
                f'<figure class="pane {team}"><figcaption>{team.capitalize()} view'
                f'</figcaption><video controls muted loop '
                f'src="media/{html.escape(info["stem"])}/{team}.webm"></video></figure>')
        lines.append("</div>")
    else:
        lines.append('<p class="note">⚠ No video could be recorded.</p>')
    lines.append("</div>")
    return "\n".join(lines)


def build_section(title: str, paths: list[str], results: dict | None) -> str:
    body = [f'<h2>{html.escape(title)} <span class="count">({len(paths)})</span></h2>']
    if not paths:
        body.append('<p class="empty">None.</p>')
    for path in sorted(paths):
        body.append(scenario_entry(path, results.get(path) if results is not None else None))
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

    if not any(changes.values()):
        sections = '<p class="empty">This PR does not touch any gameplay scenarios.</p>'
    else:
        sections = "\n".join([
            build_section("New scenarios", changes["A"], results),
            build_section("Modified scenarios", changes["M"], results),
            build_section("Unchanged scenarios", changes["U"], results),
            build_section("Deleted scenarios", changes["D"], None),
        ])

    (args.out / "index.html").write_text(
        PAGE_TEMPLATE.format(scenario_dir=SCENARIO_DIR, sections=sections))
    print(f"[scenario-review] wrote {args.out / 'index.html'} "
          f"({len(changes['A'])} new, {len(changes['M'])} modified, "
          f"{len(changes['U'])} unchanged, {len(changes['D'])} deleted)", file=sys.stderr)
    # Video failures are surfaced on the page itself; the page build only
    # fails if git classification failed (raised above).
    return 0


if __name__ == "__main__":
    sys.exit(main())
