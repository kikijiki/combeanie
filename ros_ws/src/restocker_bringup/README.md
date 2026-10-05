# restocker_bringup

Owns complete launch compositions and runtime profile validation. It is a composition root and may depend on runtime packages; runtime packages must never depend on it. The simulation launch delegates simulator details to `restocker_gazebo`.

`baseline.launch.py` enables persistent planning-scene projection and the fail-closed manipulation
coordinator by default. The projector publishes `/planning_scene_projection/status`;
planning consumers require `STATE_APPLIED` and an `applied_revision` covering their world-state
snapshot. The coordinator publishes `/restock_action_coordinator/status` and rejects goals until a
strict snapshot proves clean startup authority, required services are live, and `world -> shelf`
and `tool0 <- grasp_center` are stable and match description-owned geometry. An accepted action can
plan and execute the complete grasp, carry, insert, release, verification, and retreat sequence
through MoveIt, `ros2_control`, Gazebo attachment, and authoritative world state. Set
`planning_scene_projection:=false` or `task_coordinator:=false` only for isolated subsystem
diagnostics.
