#!/usr/bin/env python3
"""Print the last camera-rate record in one readable block, for hand-checking a row."""

from __future__ import annotations

import json
import sys

with open(sys.argv[1], encoding="utf-8") as handle:
    record = json.loads(handle.read().splitlines()[-1])
print(
    f"{record['label']} [{record.get('composition')}] domain={record.get('domain')} "
    f"load1={record.get('load1_at_start')} gpu%={record.get('gpu_busy_window_mean')} "
    f"vram={record.get('vram_used_mib_window_max')} "
    f"first_msgs={record.get('all_first_messages_arrived')} "
    f"t_first={record.get('seconds_to_first_messages', 0):.0f}s"
)
for key, value in sorted((record.get("gpu_engine_seconds_in_window") or {}).items()):
    print(f"  gpu {key.split(':')[-1][:28]:28s} {value}")
print(f"  geometry {record.get('delivered_geometry')}")
for topic, entry in sorted(record.get("topics", {}).items()):
    if not entry.get("count"):
        print(f"  {topic:32s} count=0")
        continue
    print(
        f"  {topic:32s} n={entry['count']:3d} "
        f"sim={entry.get('rate_sim_hz', 0):.3f}Hz wall={entry.get('rate_wall_hz', 0):.3f}Hz "
        f"rtf={entry.get('rtf_estimate', 0):.3f} "
        f"dt min/med/max={entry.get('sim_delta_min', 0):.3f}/"
        f"{entry.get('sim_delta_median', 0):.3f}/{entry.get('sim_delta_max', 0):.3f}"
    )
