#!/usr/bin/env bash
set -euo pipefail
cell_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 -B "$cell_dir/launch.py" "$@"
