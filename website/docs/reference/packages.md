---
id: packages
title: Package map
sidebar_position: 3
---

# Package map

All application packages live under `ros_ws/src` and are named `restocker_*`. Runtime packages
must never depend on `restocker_bringup`. Benchmarks launch bringup by name as a subprocess so
they measure the same public surface an operator drives.

| Package | Responsibility | Must not own |
| --- | --- | --- |
| `restocker_interfaces` | Backend-neutral msgs / srvs / actions | Application behavior |
| `restocker_description` | Robot + workcell Xacro, frames, geometry YAML | Controllers or tasks |
| `restocker_gazebo` | Worlds, products, ground-truth adapter, attachment plugin | Business semantics |
| `restocker_control` | Controllers, health, joint → telemetry adapter | Planning or task semantics |
| `restocker_moveit_config` | SRDF, OMPL, kinematics, `move_group` | World-state authority |
| `restocker_world_state` | Persistent semantic truth and snapshots | Learned inference |
| `restocker_perception` | Observations and backend adapters | Motion commands |
| `restocker_task_executor` | Task machine, candidates, coordinator, ports, projector | Joint-level control |
| `restocker_reasoner` | Optional constrained recovery recommendations | Safety authority |
| `restocker_recovery` | Bounded deterministic recovery primitives | Arbitrary model actions |
| `restocker_bringup` | Launch composition (`baseline`, `simulation`) | Reusable runtime logic |
| `restocker_benchmarks` | Seeded scenarios and metrics | Private runtime coupling |

```text
restocker_interfaces (leaf)
        ▲
 description · gazebo · control · moveit_config · world_state · perception
        ▲
 task_executor · reasoner · recovery
        ▲
 bringup (composition only) · benchmarks (subprocess client)
```

## Outside `ros_ws/src`

| Path | Role |
| --- | --- |
| `flake.nix` / `flake.lock` | Pinned Nix environment |
| `justfile` | Human task interface |
| `website/` | This public Docusaurus site |
| `scripts/` | Doctor, demo, workspace helpers |

## References

- **Literature:** [Coleman et al., MoveIt case study](https://arxiv.org/abs/1404.3785)
- **Reference implementation:** [moveit2](https://github.com/ros-planning/moveit2)
- **This project:** package READMEs under `ros_ws/src/*/README.md`
