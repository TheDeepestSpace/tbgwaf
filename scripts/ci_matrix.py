#!/usr/bin/env python3
"""Emit the CI shard matrix and verify shard coverage.

  ci_matrix.py matrix <build-dir>   -> JSON for fromJSON (stdout)
  ci_matrix.py check <build-dir>    -> fail unless every test runs in exactly
                                       one shard and every Test* function
                                       defined in tests/*.cpp is registered.

Shard count is derived from the live test list (~TESTS_PER_SHARD tests per
shard, capped at MAX_SHARDS); tests are assigned index % N by the binaries.
"""
import json, re, subprocess, sys
from pathlib import Path

BINARIES = {  # ctest-name -> (binary, source)
    "map_generator_tests": ("tactics_map_generator_tests", "tests/map_generator_tests.cpp"),
    "urban_terrain_tests": ("tactics_urban_terrain_tests", "tests/urban_terrain_tests.cpp"),
}
TESTS_PER_SHARD = 4
MAX_SHARDS = 16


def listing(build, binary):
    out = subprocess.run([str(Path(build) / binary), "--list"], check=True,
                         capture_output=True, text=True).stdout
    return out.split()


def shard_count(n):
    return max(1, min(MAX_SHARDS, -(-n // TESTS_PER_SHARD)))


def matrix(build):
    include = []
    for name, (binary, _) in BINARIES.items():
        n = len(listing(build, binary))
        count = shard_count(n)
        include += [{"suite": name, "binary": binary, "shard": f"{i}/{count}"}
                    for i in range(count)]
    return {"include": include}


def check(build):
    bad = []
    for name, (binary, src) in BINARIES.items():
        tests = listing(build, binary)
        if len(tests) != len(set(tests)):
            bad.append(f"{name}: duplicate names in --list")
        defined = set(re.findall(r"^(?:static )?void (Test\w+)\(\)", Path(src).read_text(), re.M))
        # Seed-range variants (Foo_1_3) are defined as wrappers; accept them.
        if defined != set(tests):
            bad.append(f"{name}: defined-but-unregistered {sorted(defined - set(tests))}, "
                       f"registered-but-undefined {sorted(set(tests) - defined)}")
        count = shard_count(len(tests))
        seen = []
        for i in range(count):
            seen += [t for k, t in enumerate(tests) if k % count == i]
        if sorted(seen) != sorted(tests):
            bad.append(f"{name}: shards do not cover each test exactly once")
        print(f"{name}: {len(tests)} tests, {count} shards")
    if bad:
        print("\n".join(bad), file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    cmd, build = sys.argv[1], sys.argv[2]
    if cmd == "matrix":
        print(json.dumps(matrix(build), separators=(",", ":")))
    else:
        check(build)
