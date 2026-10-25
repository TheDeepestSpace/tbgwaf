#!/usr/bin/env bash
# Add files to an existing gh-pages preview directory without touching the rest
# of it (the pr-preview action replaces the whole directory, so late-finishing
# jobs publish themselves with a plain commit instead).
# Usage: publish_to_preview.sh <dest-dir-on-gh-pages> <source-dir>
set -euo pipefail
dest="$1"; src="$2"
git config --global --add safe.directory "$GITHUB_WORKSPACE"
git config user.name "github-actions[bot]"
git config user.email "41898282+github-actions[bot]@users.noreply.github.com"
for attempt in 1 2 3 4 5 6; do
  git fetch --depth=1 origin gh-pages
  rm -rf /tmp/ghp && git worktree prune
  git worktree add -f --detach /tmp/ghp FETCH_HEAD
  mkdir -p "/tmp/ghp/$dest"
  cp -r "$src"/. "/tmp/ghp/$dest"/
  (cd /tmp/ghp && git add -A "$dest" && { git diff --cached --quiet || git commit -qm "Publish $dest ($GITHUB_JOB)"; })
  if (cd /tmp/ghp && git push origin HEAD:gh-pages); then exit 0; fi
  sleep $((attempt * 3))
done
echo "failed to publish after retries" >&2; exit 1
