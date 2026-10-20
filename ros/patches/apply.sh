#!/usr/bin/env bash
# Apply the patches listed in `series` to the upstream checkouts in ros/src/upstream.
# Safe to re-run: a patch that is already applied is skipped. A patch that applies neither
# forwards nor backwards stops the script, which usually means the pin in upstream.repos moved.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
upstream="$here/../src/upstream"

while read -r repo patch; do
  [[ -z "$repo" || "$repo" == \#* ]] && continue
  dir="$upstream/$repo"
  file="$here/$patch"
  if [[ ! -d "$dir/.git" ]]; then
    echo "error: $dir is not a checkout; run vcs import first" >&2
    exit 1
  fi
  if git -C "$dir" apply --reverse --check "$file" 2>/dev/null; then
    echo "already applied: $patch -> $repo"
  elif git -C "$dir" apply --check "$file" 2>/dev/null; then
    git -C "$dir" apply "$file"
    echo "applied:         $patch -> $repo"
  else
    echo "error: $patch doesn't apply to $repo at $(git -C "$dir" rev-parse --short HEAD)." >&2
    echo "       Check its base SHA in ros/patches/README.md against ros/upstream.repos." >&2
    exit 1
  fi
done < "$here/series"
