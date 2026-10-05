#!/usr/bin/env bash
# Copyright 2026 Matteo Bernacchia <dev@kikijiki.com>
# SPDX-License-Identifier: MIT
#
# Print the extra `--cmake-args ...` group that routes C/C++ compiles through kache
# (https://github.com/kunobi-ninja/kache) via CMake's compiler-launcher mechanism, or print
# nothing when the integration must stay off.
#
# Usage:
#   kache_cmake_args.bash    print the `--cmake-args ...` group
#
# Why no seed handling lives here: the Nix dev-shell environment exports NIX_CFLAGS_COMPILE
# containing `-frandom-seed=<10-char id>`, injected into every cc1 by the gcc wrapper, which
# kache resolves into its cache key. That id used to be the dev-shell profile's store-path
# prefix, so it rotated whenever the profile was rebuilt and every fresh shell went cold
# (Card 042 finding 2). The id is now pinned at the source instead: `flake.nix`'s shared
# mkShell sets `NIX_OUTPATH_USED_AS_RANDOM_SEED = "combeanie"`, which stdenv's
# reproducible-builds.sh setup hook reads in preference to `$out`, so every dev shell exports
# `-frandom-seed=combeanie` before anything runs (Card 045). The Card 042 `--env` rewrite was
# therefore removed. Note the pin still has to ride the environment, never argv: an explicit
# `-frandom-seed` on the command line is classified by kache as an unsupported flag
# (`unmodeled [TooHard]`) and the compile becomes a passthrough with no caching at all.
#
# Design rules (Card 042):
#   - Optional and fail-open: no kache on PATH, no yq, or an unreadable defaults file all
#     print nothing and exit 0, so `just build` runs the exact command it ran before this
#     script existed. The Nix sandbox has none of these tools, so `nix flake check` builds
#     are untouched by construction.
#   - A CLI `--cmake-args` REPLACES colcon's defaults from config/colcon-defaults.yaml
#     (argparse nargs='*' + set_defaults, no merge), so the base list is re-emitted from
#     that same file first — one source of truth, and fresh build dirs keep `-G Ninja` and
#     `-DCMAKE_BUILD_TYPE=RelWithDebInfo`.
#   - The launcher is the absolute path of the found kache: the Nix dev shell gives CMake
#     absolute compiler paths, so PATH shims may never be hit; the launcher prefix always
#     runs regardless of where the compiler binary lives.

set -euo pipefail

if [[ $# -gt 0 ]]; then
  # Unknown mode: fail open, print nothing.
  exit 0
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
defaults_file="${repo_root}/config/colcon-defaults.yaml"

# Fail open at every step: absent tools mean "build exactly as today".
if ! command -v kache >/dev/null 2>&1; then
  exit 0
fi
if ! command -v yq >/dev/null 2>&1 || [[ ! -r "${defaults_file}" ]]; then
  exit 0
fi

base_args="$(
  yq -r '.build["cmake-args"] // [] | .[]' "${defaults_file}" 2>/dev/null |
    tr '\n' ' ' | sed 's/[[:space:]]*$//'
)" || exit 0
if [[ -z "${base_args// /}" ]]; then
  exit 0
fi

kache_bin="$(command -v kache)"
printf -- '--cmake-args %s -DCMAKE_C_COMPILER_LAUNCHER=%s -DCMAKE_CXX_COMPILER_LAUNCHER=%s' \
  "${base_args}" "${kache_bin}" "${kache_bin}"
