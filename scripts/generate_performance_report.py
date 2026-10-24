#!/usr/bin/env python3
"""Generate the browsable Performance suite report from preserved JSON runs."""

import argparse
import html
import json
import math
from pathlib import Path


METRICS = (
    ("fps", "FPS", True, 1),
    ("frame_ms", "Frame ms", False, 2),
    ("fov_ms", "FOV ms", False, 2),
    ("draw_calls", "Draw calls", False, 0),
    ("triangles", "Triangles", False, 0),
    ("vertices", "Vertices", False, 0),
)


def load_result(path: Path) -> dict | None:
    try:
        result = json.loads(path.read_text())
        if result.get("schema_version") != 1 or not result.get("maps"):
            return None
        result["_filename"] = path.name
        return result
    except (OSError, ValueError, TypeError):
        return None


def is_master(result: dict) -> bool:
    ref = result.get("metadata", {}).get("target_ref", "")
    return ref in {"master", "refs/heads/master", "origin/master"}


def compatible_with(result: dict, current: dict) -> bool:
    """Only compare runs made with the same scene/sample contract and renderer."""
    keys = (
        "map_type", "feature_modes", "warmup_frames", "sample_frames", "pane_count",
        "window_width", "window_height", "fov_overlay", "fog_active", "seeds",
    )
    result_config = result.get("configuration", {})
    current_config = current.get("configuration", {})
    same_config = all(result_config.get(key) == current_config.get(key) for key in keys)
    return same_config and result.get("environment", {}).get("gl_renderer") == \
        current.get("environment", {}).get("gl_renderer")


def median(values: list[float]) -> float | None:
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    middle = len(values) // 2
    if len(values) % 2:
        return values[middle]
    return (values[middle - 1] + values[middle]) / 2


def metric_p50(result: dict, metric: str, seed: int | None = None) -> float | None:
    aggregate = result.get("aggregate", {})
    if seed is not None:
        match = next((entry for entry in result.get("maps", []) if entry.get("seed") == seed), None)
        if not match:
            return None
        aggregate = match.get("aggregate", {})
    value = aggregate.get(metric, {}).get("p50")
    return float(value) if isinstance(value, (int, float)) else None


def baseline_value(runs: list[dict], metric: str, seed: int | None = None) -> tuple[float, float, float] | None:
    values = [value for run in runs if (value := metric_p50(run, metric, seed)) is not None]
    center = median(values)
    if center is None:
        return None
    return center, min(values), max(values)


def fmt(value: float | None, decimals: int) -> str:
    if value is None:
        return "—"
    if decimals == 0:
        return f"{value:,.0f}"
    return f"{value:,.{decimals}f}"


def baseline_cell(value: tuple[float, float, float] | None, decimals: int) -> str:
    if value is None:
        return "—"
    center, low, high = value
    if math.isclose(low, high, rel_tol=1e-9, abs_tol=1e-9):
        return fmt(center, decimals)
    return f'{fmt(center, decimals)} <span class="range">[{fmt(low, decimals)}–{fmt(high, decimals)}]</span>'


def delta_cell(current: float | None, baseline: tuple[float, float, float] | None,
               higher_is_better: bool, decimals: int) -> str:
    if current is None or baseline is None or baseline[0] == 0:
        return "—"
    change = 100.0 * (current - baseline[0]) / baseline[0]
    good = change >= 0 if higher_is_better else change <= 0
    css = "good" if good else "bad"
    sign = "+" if change >= 0 else ""
    return f'<span class="{css}">{sign}{change:.1f}%</span>'


def map_rows(current: dict, baselines: list[dict]) -> str:
    rows = []
    for entry in current.get("maps", []):
        seed = entry.get("seed")
        cells = [f"<th>{seed}</th>"]
        for metric, _, higher_is_better, decimals in METRICS:
            current_value = metric_p50(current, metric, seed)
            baseline = baseline_value(baselines, metric, seed)
            cells.append(f"<td>{fmt(current_value, decimals)}</td>")
            cells.append(f"<td>{baseline_cell(baseline, decimals)}</td>")
            if metric in {"fps", "frame_ms"}:
                cells.append(f"<td>{delta_cell(current_value, baseline, higher_is_better, decimals)}</td>")
        rows.append("<tr>" + "".join(cells) + "</tr>")
    return "\n".join(rows)


def cards(current: dict, baselines: list[dict]) -> str:
    output = []
    for metric, label, higher_is_better, decimals in METRICS:
        value = metric_p50(current, metric)
        baseline = baseline_value(baselines, metric)
        output.append(
            f'<div class="card"><span>{html.escape(label)}</span><strong>{fmt(value, decimals)}</strong>'
            f'<small>master {baseline_cell(baseline, decimals)} · '
            f'{delta_cell(value, baseline, higher_is_better, decimals)}</small></div>'
        )
    return "\n".join(output)


def history_rows(runs: list[dict]) -> str:
    rows = []
    for result in sorted(runs, key=lambda item: item.get("metadata", {}).get("timestamp", ""), reverse=True):
        metadata = result.get("metadata", {})
        environment = result.get("environment", {})
        commit = metadata.get("commit", "")
        filename = html.escape(result.get("_filename", ""), quote=True)
        rows.append(
            "<tr>"
            f'<td>{html.escape(metadata.get("timestamp", ""))}</td>'
            f'<td>{html.escape(metadata.get("target_ref", ""))}</td>'
            f'<td><code>{html.escape(commit[:10])}</code></td>'
            f'<td>{fmt(metric_p50(result, "fps"), 1)}</td>'
            f'<td>{fmt(metric_p50(result, "frame_ms"), 2)}</td>'
            f'<td>{html.escape(environment.get("gl_renderer", ""))}</td>'
            f'<td><a href="history/{filename}">JSON</a></td>'
            "</tr>"
        )
    return "\n".join(rows)


def build_page(current: dict, runs: list[dict]) -> str:
    metadata = current.get("metadata", {})
    config = current.get("configuration", {})
    environment = current.get("environment", {})
    baselines = [run for run in runs if is_master(run) and compatible_with(run, current)]
    header_cells = ["<th rowspan=\"2\">Seed</th>"]
    sub_cells = []
    for metric, label, _, _ in METRICS:
        columns = 3 if metric in {"fps", "frame_ms"} else 2
        header_cells.append(f'<th colspan="{columns}">{html.escape(label)}</th>')
        sub_cells.extend(["<th>current</th>", "<th>master</th>"])
        if columns == 3:
            sub_cells.append("<th>Δ</th>")
    commit = metadata.get("commit", "")
    renderer = environment.get("gl_renderer", "unknown")
    return f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>tbgwaf · Performance suite</title>
<style>
:root {{ color-scheme: dark; }}
body {{ font-family: system-ui,sans-serif; margin: 2rem auto; max-width: 1500px; padding: 0 1rem;
        background: #14161a; color: #e8e8e8; }}
a {{ color: #8ab4ff; }} h1 {{ margin-bottom: .25rem; }}
.nav {{ display: flex; gap: 1rem; margin: .2rem 0 1.5rem; }}
.muted,.range {{ color: #9299a5; }} .range {{ font-size: .8em; white-space: nowrap; }}
.cards {{ display: grid; grid-template-columns: repeat(auto-fit,minmax(180px,1fr)); gap: .75rem; }}
.card {{ border: 1px solid #343941; background: #1b1e23; border-radius: 8px; padding: .8rem; }}
.card span,.card small {{ display: block; color: #aab0ba; }} .card strong {{ font-size: 1.5rem; }}
.good {{ color: #62cf83; }} .bad {{ color: #ef7373; }}
.scroll {{ overflow-x: auto; }}
table {{ border-collapse: collapse; width: 100%; font-size: .86rem; }}
th,td {{ border: 1px solid #343941; padding: .42rem .55rem; text-align: right; white-space: nowrap; }}
thead th {{ background: #22262c; }} tbody tr:nth-child(even) {{ background: #191c20; }}
tbody th,td:nth-child(1) {{ text-align: left; }} code {{ font-size: .9em; }}
.meta {{ display: grid; grid-template-columns: max-content 1fr; gap: .25rem 1rem; }}
.meta dt {{ color: #9299a5; }} .meta dd {{ margin: 0; overflow-wrap: anywhere; }}
</style>
</head>
<body>
<h1>Performance suite</h1>
<nav class="nav"><a href="../">Game</a><a href="../gallery/">Asset gallery</a><a href="latest/results.json">Latest raw JSON</a></nav>
<p>Observational software-rendering measurements for 10 pinned <code>urban-auto</code> maps. Values are medians;
master ranges show the median from each preserved master run. Timing changes are reported, not gated.</p>
<div class="cards">{cards(current, baselines)}</div>
<h2>Per-map measurements</h2>
<div class="scroll"><table><thead><tr>{''.join(header_cells)}</tr><tr>{''.join(sub_cells)}</tr></thead>
<tbody>{map_rows(current, baselines)}</tbody></table></div>
<p class="muted">Each map warms up for {config.get('warmup_frames', '—')} frames, then records
{config.get('sample_frames', '—')} frames at {config.get('window_width', '—')}×{config.get('window_height', '—')},
two fog-of-war panes with six figures and shadow-map FOV.</p>
<h2>Current run</h2>
<dl class="meta">
<dt>Ref</dt><dd>{html.escape(metadata.get('target_ref', ''))}</dd>
<dt>Commit</dt><dd><code>{html.escape(commit)}</code></dd>
<dt>Recorded</dt><dd>{html.escape(metadata.get('timestamp', ''))}</dd>
<dt>Run</dt><dd>{html.escape(metadata.get('run_id', ''))}</dd>
<dt>Renderer</dt><dd>{html.escape(renderer)}</dd>
<dt>GL</dt><dd>{html.escape(environment.get('gl_vendor', ''))} · {html.escape(environment.get('gl_version', ''))}</dd>
<dt>Platform</dt><dd>{html.escape(environment.get('platform', ''))}</dd>
</dl>
<h2>Run history</h2>
<div class="scroll"><table><thead><tr><th>Recorded</th><th>Ref</th><th>Commit</th><th>FPS</th><th>Frame ms</th><th>Renderer</th><th>Raw</th></tr></thead>
<tbody>{history_rows(runs)}</tbody></table></div>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current", required=True, type=Path)
    parser.add_argument("--history-dir", required=True, type=Path)
    parser.add_argument("--out", required=True, type=Path)
    args = parser.parse_args()

    current = load_result(args.current)
    if current is None:
        parser.error(f"invalid performance result: {args.current}")
    runs = []
    if args.history_dir.exists():
        for path in args.history_dir.glob("*.json"):
            if result := load_result(path):
                runs.append(result)
    current_run_id = current.get("metadata", {}).get("run_id")
    if not any(run.get("metadata", {}).get("run_id") == current_run_id for run in runs):
        runs.append(current)

    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / "index.html").write_text(build_page(current, runs))
    print(f"wrote {args.out / 'index.html'} ({len(runs)} preserved runs, "
          f"{sum(is_master(run) for run in runs)} master baselines)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
