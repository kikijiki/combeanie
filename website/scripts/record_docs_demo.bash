#!/usr/bin/env bash
# SPDX-License-Identifier: MIT
set -euo pipefail
repo="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$repo"
exec nix develop .#baseline --command bash "$repo/website/scripts/record_docs_demo_inner_restock.bash" "$@"
