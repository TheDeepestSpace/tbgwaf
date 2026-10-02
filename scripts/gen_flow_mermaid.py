#!/usr/bin/env python3
"""Render the flow YAML as a Mermaid stateDiagram-v2 (.mmd).

usage: gen_flow_mermaid.py <flow.yaml> <out.mmd>
The CMake step then runs mermaid-cli (mmdc) on the result to produce an SVG.
"""
import sys

import yaml


def main():
    src, out = sys.argv[1:3]
    with open(src) as f:
        flow = yaml.safe_load(f)
    lines = ["stateDiagram-v2", f"    [*] --> {flow['initial']}"]
    for s in flow["states"]:
        if s.get("description"):
            lines.append(f"    {s['name']} : {s['description']}")
    for t in flow["transitions"]:
        lines.append(f"    {t['from']} --> {t['to']} : {t['event']}")
    with open(out, "w") as f:
        f.write("\n".join(lines) + "\n")


main()
