#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo "Compatibility entry point: using the reproducible full bootstrap"
exec "${SCRIPT_DIR}/bootstrap" --profile all "$@"
