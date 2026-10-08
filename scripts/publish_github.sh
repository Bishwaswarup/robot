#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git rev-parse --show-toplevel)"
repo_name="${1:-$(basename "$repo_root")}"

if ! command -v gh >/dev/null 2>&1; then
    echo "GitHub CLI (gh) is required. Install it and run 'gh auth login' first." >&2
    exit 1
fi

gh auth status >/dev/null
git -C "$repo_root" rev-parse --verify HEAD >/dev/null

if git -C "$repo_root" remote get-url origin >/dev/null 2>&1; then
    echo "Using existing origin: $(git -C "$repo_root" remote get-url origin)"
    git -C "$repo_root" push --set-upstream origin HEAD
else
    gh repo create "$repo_name" \
        --private \
        --source="$repo_root" \
        --remote=origin \
        --push
fi
