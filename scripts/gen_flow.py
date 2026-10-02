#!/usr/bin/env python3
"""Generate the app flow C++ header and Mermaid diagram from flow/app_flow.yaml.

usage: gen_flow.py <flow.yaml> --header <out.h> --mermaid <out.mmd> --templates <dir>
Outputs are rendered with Jinja2 from flow/templates/*.j2.
Uses PyYAML when available, else a tiny parser for the restricted shape the
flow file uses (documented in the YAML header).
"""
import argparse
import re

import jinja2


def parse_minimal(text):
    doc = {"states": [], "transitions": []}
    section = None
    for raw in text.splitlines():
        line = raw.split("#", 1)[0].rstrip()
        if not line.strip():
            continue
        if not line.startswith(" "):
            key, _, val = line.partition(":")
            val = val.strip()
            if val:
                doc[key.strip()] = val
                section = None
            else:
                section = key.strip()
                doc.setdefault(section, [])
            continue
        m = re.match(r"\s*-\s*\{(.*)\}\s*$", line)
        if not m or section is None:
            raise SystemExit(f"unsupported YAML line: {raw!r}")
        item = {}
        for pair in m.group(1).split(","):
            k, _, v = pair.partition(":")
            item[k.strip()] = v.strip()
        doc[section].append(item)
    return doc


def load(path):
    text = open(path, encoding="utf-8").read()
    try:
        import yaml
        return yaml.safe_load(text)
    except ImportError:
        return parse_minimal(text)


def camel(s):
    return "".join(p.capitalize() for p in s.split("_"))


def validate(doc):
    ids = [s["id"] for s in doc["states"]]
    if doc["initial"] not in ids:
        raise SystemExit("initial state not in states")
    events = set()
    for t in doc["transitions"]:
        if t["from"] not in ids or t["to"] not in ids:
            raise SystemExit(f"transition references unknown state: {t}")
        if (t["from"], t["event"]) in events:
            raise SystemExit(f"duplicate transition: {t}")
        events.add((t["from"], t["event"]))


def render(template_dir, name, doc):
    env = jinja2.Environment(
        loader=jinja2.FileSystemLoader(template_dir),
        undefined=jinja2.StrictUndefined,
        trim_blocks=True, lstrip_blocks=True, keep_trailing_newline=True)
    env.filters["camel"] = camel
    events = sorted({t["event"] for t in doc["transitions"]})
    return env.get_template(name).render(events=events, **doc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("yaml")
    ap.add_argument("--header", required=True)
    ap.add_argument("--mermaid", required=True)
    ap.add_argument("--templates", required=True, help="dir with *.j2 templates")
    a = ap.parse_args()
    doc = load(a.yaml)
    validate(doc)
    open(a.header, "w", encoding="utf-8").write(render(a.templates, "AppFlow.h.j2", doc))
    open(a.mermaid, "w", encoding="utf-8").write(render(a.templates, "app_flow.mmd.j2", doc))


if __name__ == "__main__":
    main()
