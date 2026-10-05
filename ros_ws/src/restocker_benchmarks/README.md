# restocker_benchmarks

Owns seeded scenarios, metrics, result metadata, serialization, and plots. It consumes public
runtime interfaces rather than implementation internals.

## What is here

| Module | Purpose |
| --- | --- |
| `scenarios.py` | The scenario table: a stock file and a goal sequence, and nothing else |
| `runner.py` | One headless run: launch, drive the goals, record every outcome, tear down |
| `taxonomy.py` | Grouping a terminal outcome by cause and by how far it got |
| `report.py` | A campaign directory to `summary.json`, `report.md` and plots |
| `plots.py` | SVG bar charts, standard library only |
| `tray_perception_campaign.py` | Predeclared wrist-tray matrix, covariance-derived attachment budget, offline summary |

Executables: `ros2 run restocker_benchmarks benchmark_run <scenario>` and
`ros2 run restocker_benchmarks benchmark_report [campaign]`, wrapped by `just benchmark`,
`just benchmark-all` and `just benchmark-report`.

## Behavior

This is a measurement tool, not a gate. `benchmark_run` exits 0 when it records a failure. The
measured system fails intermittently, and a pass count alone would hide the causes.

A failed goal does not stop its run. Failures are grouped by distinct cause with counts rather than
totalled, and the report lists the brief's metrics it cannot measure instead of filling them with
defaults.

## Dependency shape

The runner launches `restocker_bringup`'s public composition as a subprocess, by name, exactly as
an operator would. It does not declare a manifest dependency on it, because runtime packages must
not depend on the composition root. It does depend on `restocker_gazebo` and
`restocker_moveit_config` for their share directories, so a benchmark reads the same stock and
planner configuration the launch loads rather than a copy.

`taxonomy.py`, `scenarios.py`, `report.py`, `plots.py` and `tray_perception_campaign.py` import
nothing from ROS. A stored campaign can be re-aggregated and re-plotted from a plain Python
interpreter after the workspace that produced it is gone; only `runner.py` needs `rclpy`.
