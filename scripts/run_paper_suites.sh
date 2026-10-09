#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
python="${PYTHON:-python}"

for suite in main lookahead scaling; do
    "$python" "$repo_root/scripts/paper_runs.py" --suites "$suite" "$@"
done
