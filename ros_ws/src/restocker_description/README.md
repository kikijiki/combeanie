# restocker_description

Owns the reusable robot assembly: a project rail and carriage carrying the official Universal
Robots `ur_description` UR10e, plus the project gripper and wrist camera. The UR10e contributes its
DAE visual meshes, STL collision meshes, kinematics, inertias and limits; this package owns the
`carriage -> base_link` mount and the `tool0 -> gripper -> grasp_center` attachment.

The `restocker_robot` macro is rooted at `rail_base`; `restocker_robot_with_parent` adds an
optional fixed external attachment. `restocker.urdf.xacro` uses that wrapper for the single-robot
RViz assembly attached to `world`.

`gripper` is the physical mount frame. `grasp_center` is a massless fixed semantic frame 0.14 m
along gripper +Z at the center of the finger contact region. Manipulation candidates use it while
MoveIt solves the chain ending at `tool0`.

`beanie` (`urdf/beanie.xacro`) is the project's namesake: a decorative knit cap fixed-jointed to
the UR10e `shoulder_link` cap. It is **visual only** — five primitives under `<visual>` (ribbed
cuff, crown, pom) and no `<collision>`, `<inertial>`, `ros2_control`, or MoveIt entry, so it
adds no body, mass, or planning surface to the robot. The cap-sized seat covers the shoulder
cap without overhanging its silhouette: `upper_arm_link` and `wrist_1_link` can never reach it
at any configuration, while grazing by `forearm_link` or `wrist_2_link` in extreme folds is
accepted as visual-only (both must still clear the shoulder's own collision mesh to get
there). The wrist camera frustum stays clear at the default and simulator-initial poses, the
overhead image never overlaps the work regions across the full rail and shoulder-pan sweep,
and the hat's depth returns fall inside the shoulder self-filter capsule that obstacle
extraction already applies. The description tests assert each of those invariants; see
`test/test_robot_description.py`.

`config/gripper_geometry.yaml` is the single source for the gripper body, finger collision boxes,
joint limits and dynamics, grasp-center offset, and positive-clearance attachment policy. The robot
Xacro loads it directly so simulator attachment validation matches the emitted geometry.

`config/workcell_geometry.yaml` is the versioned source for surveyed shelf, divider, front-rail,
roller-bed, stock-region, and lane-volume geometry. The workcell Xacro loads it directly and
generates the divider bodies, the inclined roller bed and the per-lane semantic frames from it, so
the lane count lives in one file. The lanes are gravity-fed: the bed rises toward the rear, so a
product set down at the lane mouth runs forward and comes to rest against the rail. Static tests
check that the emitted collision model and semantic volumes are consistent.
