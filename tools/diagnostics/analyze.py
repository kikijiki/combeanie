#!/usr/bin/env python3
"""Summarise one instrumented acceptance run: limit margins, peak errors, abort context."""

import csv
from pathlib import Path
import sys

# The URDF bounds that controller_manager enforces, not the margined ones MoveIt plans to.
# A run is interesting precisely when a joint reaches one of these, so the tighter planning
# limits would hide the thing being looked for.
LIM = {
    "shoulder_pan_joint": (-6.283185, 6.283185),
    "shoulder_lift_joint": (-6.283185, 6.283185),
    "elbow_joint": (-3.141593, 3.141593),
    "wrist_1_joint": (-6.283185, 6.283185),
    "wrist_2_joint": (-6.283185, 6.283185),
    "wrist_3_joint": (-6.283185, 6.283185),
}


def main(d):
    d = Path(d)
    csvp = d / "arm_state.csv"
    exit_file = d / "exit.txt"
    exit_code = exit_file.read_text().strip() if exit_file.exists() else "?"
    print(f"### {d.name}  exit={exit_code}")
    log = (d / "console.log").read_text(errors="replace") if (d / "console.log").exists() else ""
    for pat in (
        "out of bounds",
        "PATH_TOLERANCE",
        "Aborted due to",
        "Position Error:",
        "Holding position due to",
        "restricted to zero",
        "STATUS_FAILED",
        "detail=",
    ):
        hits = [ln for ln in log.splitlines() if pat in ln]
        if hits:
            print(f"  [{pat}] x{len(hits)}")
            for h in hits[:6]:
                print("    ", h[:220])
    if not csvp.exists():
        return
    with csvp.open() as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return
    print(f"  samples={len(rows)} t=[{rows[0]['stamp']},{rows[-1]['stamp']}]")
    for j, (lo, hi) in LIM.items():
        fb = [float(r[j + "_fb"]) for r in rows if r[j + "_fb"]]
        er = [(abs(float(r[j + "_err"])), float(r["stamp"])) for r in rows if r[j + "_err"]]
        me, mt = max(er)
        print(
            f"  {j}: fb[{min(fb):+.4f},{max(fb):+.4f}] "
            f"margin(lo={min(fb) - lo:+.4f},hi={hi - max(fb):+.4f}) max|err|={me:.4f}@{mt:.2f}"
        )
    # any sample where output velocity is large but feedback velocity is ~0 => command clamped
    clamped = []
    for r in rows:
        for j in LIM:
            try:
                o = float(r[j + "_out"])
                f = float(r[j + "_fbv"])
            except (ValueError, KeyError):
                continue
            if abs(o) > 0.05 and abs(f) < 0.2 * abs(o):
                clamped.append((float(r["stamp"]), j, o, f))
    print(f"  clamp-signature samples: {len(clamped)}")
    for c in clamped[:10]:
        print(f"    t={c[0]:.2f} {c[1]} out={c[2]:+.4f} fbv={c[3]:+.4f}")


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        main(arg)
