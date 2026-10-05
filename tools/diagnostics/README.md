# Motion forensics

Hand-run tools for diagnosing a failed or aborted run of the manipulation acceptance test. They
are not part of the build or CI; nothing imports them, and no test covers them. They exist because
failures such as a controller abort or a trajectory that grazes a divider leave their evidence in
per-cycle telemetry that is gone when the run ends.

Typical use:

```
# In one terminal, alongside `just launch-baseline`:
python3 record_state.py /tmp/run01/arm_state.csv

# After the run:
python3 analyze.py /tmp/run01          # limit margins, peak errors, abort context
python3 collide.py /tmp/run01          # links that penetrate the workcell geometry
```

`record_state.py` subscribes to `/arm_controller/controller_state`,
`/rail_controller/controller_state`, `/joint_states`, `/clock`,
`/perception/object_observations` and `/planning_scene_projection/status`, polls
`/get_planning_scene` at 4 Hz, and writes one CSV row per cycle plus sidecar files (`.rail`,
`.js`, `.clock`, `.obj`, `.scene`, `.status`). The clock trace is what rules out simulation
starvation as a cause of a tracking abort. Every sidecar is stamped on the simulation clock, so
rows from different files describe the same instant.

`.obj` and `.scene` are a matched pair: `.obj` is where a product really is, from the Gazebo
ground-truth stream, and `.scene` is where MoveIt believes it is, read from the same service the
projector verifies against. A trajectory that runs into a product was planned against a scene that
is missing the product, against one that holds it at a pose `.obj` disagrees with, or against a
correct scene. These are three different faults, and telling them apart after the run needs both
traces.

`.scene` is polled rather than taken from `/monitored_planning_scene` because that topic carries
diffs, and a diff stream has to be replayed to answer "what was in the scene at time t". `.status`
carries the projector's own state, error code, applied revision and content generation, which is
what distinguishes a scene that is wrong from a projection that stopped converging.

Attached products in `.scene` carry a pose in the parent link's frame, not the planning frame, so
only the `world` rows are directly comparable with `.obj`.

`analyze.py` expects that directory to also hold the run's `console.log` and, optionally,
`exit.txt`.

## Caveat: fk.py duplicates the URDF

`fk.py` carries its own copy of the kinematic chain and the workcell collision shapes, transcribed
from `restocker_description`. That duplication is deliberate (an independent forward kinematic
solution must be independent of the one under investigation), but it means **fk.py does not track
the URDF**.

Against Gazebo's `/world/restocking/pose/info` link poses at a matched sample, `upper_arm_link`,
`forearm_link`, the three wrist links and `gripper` agree to within 1.1 mm. Two cautions: compare at
a matched instant (the two streams are sampled independently, and during a fast segment a 7 ms
offset is 20 mm at the wrist), and pass the recorded finger joint positions to `link_frames`
(the fingers travel 32 mm, so a wrong aperture moves each one by that much). Re-check it against
`restocker_description` after any geometry change. The joint bounds in `analyze.py` are likewise the URDF's enforced
limits, not the margined ones MoveIt plans against (see
`restocker_moveit_config/config/joint_limits.yaml`).

## The offline planning probe

`planning_probe/` answers questions no live run can afford: a placement sweep is thousands of
Cartesian plans and a live run takes minutes each. It plans against the real `RobotModel`, SRDF and
planning scene with **no simulator running**, so it needs no display, no Gazebo, and no simulator
slot.

It is in the tree because three families of shipped numbers (the seeded generator's approach
corridor, the depth at which a pre-grasp stops being reachable, and the insert's rail band) had one
source, a program that was never committed and cannot be recovered from `git`.

### Running it

```
./tools/diagnostics/planning_probe/run.bash --mode geometry
./tools/diagnostics/planning_probe/run.bash --mode fixture \
  --start "rail_joint=0.1 shoulder_pan_joint=-1.57 ..." \
  --target 0.6,0.379,0.9307,-0.5,-0.5,-0.5,0.5
```

`run.bash` builds the probe if it is not built, expands the descriptions, reads the workcell pose
out of the baseline scenario and the padding out of `move_group.launch.py`, and passes everything
after the script name to the probe unchanged. Build output goes to `artifacts/planning_probe/`,
which `.gitignore` already excludes and `just clean-all` already removes; `RESTOCKER_PROBE_WORKDIR`
moves it. Nothing is written into `ros_ws/`, so a sweep never disturbs a workspace another agent is
testing against.

The probe is **not** a package under `ros_ws/src`. It is not part of `just build`,
`just test`, or CI, exactly like the rest of this directory, and `scripts/check_repository.py`
refuses unexpected package directories there.

Output is CSV on stdout with a leading `#` provenance block naming the padding, the group, the seed
count and the descriptions, so a redirected sweep still says what produced it.

### The modes, and which shipped constant each one backs

| Mode | What it sweeps | Backs |
| --- | --- | --- |
| `geometry` | No planner at all. Recomputes the corridor half-width, the pre-grasp stand-off, the jaw apertures and the grasp axial offsets from the files that own them. | `scenario_config.load_approach_corridor`, `PLANNER_ROBOT_PADDING_M`, `PREGRASP_DISTANCE_M` |
| `reach` | A lone product on an empty tray, sweeping the pre-grasp depth (the distance from the rail axis to the pre-grasp `tool0` frame). | `MINIMUM_PREGRASP_REACH_Y_M` |
| `corridor` | A target held at a clear placement, one neighbour swept across and along the approach. The first across-distance at which every start completes is the implied half-width plus the neighbour's radius. | `WRIST_LINK_RADIUS_M`, `ApproachCorridor.half_width_m` |
| `approach` | A product swept along the drawable y interval with neighbours at the fixed scenario's tray spacing. | the handover's "a truncating approach is a different fault" |
| `placements` | Whole generated scenarios, read from the CSV `generator_sweep.py --mode placements` writes: for every product of every seed, with every other product of that seed in the scene. | generator version 5's before-and-after table |
| `layout` | One literal arrangement given as `--products geometry_key:x:y,...`, measured at each of `--yaws` separately, reporting the pre-grasp IK solutions and what refused them alongside both straight lines, approach *and* retract. | nothing yet; every other mode measures the −y approach alone |
| `envelope` | No product, no approach: the tray-station IK envelope. Sweeps camera world y (`--from/--to/--step`) and world z (`--along-from/--along-to/--along-step`) at both surveyed tray slice centres, poses the camera optical frame under `--orientation lookat` (boresight at the slice centre, image-right +X — the shipped convention) or `--orientation straightdown` (the specification's boresight), maps it to a tool0 goal through the fixed wrist-camera mount, and counts collision-aware IK solutions on `manipulator` at tip `tool0`. | the tray-station envelope recorded behind `survey_stations.cpp`'s tray stations |
| `fixture` | One recorded refusal, not a sweep: `--start "name=value name=value ..."` and `--target x,y,z,qx,qy,qz,qw` take the `fixture:` segment a linear refusal receipt now carries, and the mode re-runs that exact line both collision-checked and collision-free at the shipped 0.005 m and at `--cartesian-step` (0.0005 m by default), reporting `complete`/`kinematic`/`collision` and, for a kinematic stop, the joint closest to one of its own bounds. | Distinguishing interpolation from a kinematic limit: a stop the finer step clears was interpolation, one that holds at both is a kinematic limit, and only the second justifies changing the egress bar |

`fixture` replays one line, and two things decide what a replay can claim.
`CartesianInterpolator` makes a **single IK attempt per waypoint** (it passes a zero timeout on
purpose, so random restarts cannot create joint-space jumps), so a stop can be one attempt that
did not converge rather than a line no configuration can follow — run the same fixture repeatedly
and treat only a stop that reproduces at every repetition *and* at both steps as a kinematic
limit; raising the solver's configured timeout does not change the replay, the interpolator does
not use it. And the console line carries no scene: the collision-free control is the faithful
half, while the checked run is replayed against this probe's rebuilt scene.

`layout` is the only mode that varies the approach yaw. Every other *product* mode fixes it at
−π/2 (`envelope` has no approach at all), the
−y approach, because that is the one `scenario_config.load_approach_corridor` and the seeded
generator's `required_lateral_distance` are derived for. `grasp.approach_yaws_rad` holds three,
`grasp_candidates.cpp` scores all three, and `restock_coordinator_driver.cpp` executes
`candidates.front()` with no fallback to the next one, so an arrangement has to survive whichever
yaw wins the score, not merely the one the corridor rule models. `--yaws` is what asks that
question.

`generator_sweep.py` beside it is the draw half of the same exercise: `--mode budget` counts the
seeds whose placement search runs out of attempts, `--mode intervals` prints the drawable y
interval per class before and after the reachability clip, and `--mode placements` emits the draws
the probe reads. It replays superseded generator version 4 as well as the shipped version 5,
because a version comparison needs both and the shipped module only knows how to draw version 5.

### Enforced invariants

Both earlier versions of this program were wrong in a way that made a broken system look healthy.
Both mistakes are now enforced in code.

**The padding is applied, and the constructor asserts that it took.**
`scene->getCollisionEnvNonConst()->setPadding(robot_description_planning.default_robot_padding)`.
Without it the probe measures a corridor no collision check ever uses: MoveIt inflates every robot
link before checking, so an unpadded probe reports a 1.5 mm interpenetration as clearance. This was
the version 3 to version 4 correction of the generator's corridor rule. Because `setPadding` could
return without effect and reproduce the defect silently, the padding is read back off every
collision-geometry link and a mismatch is fatal.

**A sweep is never reduced to its best branch.** The coordinator does not choose the arm
configuration it approaches from: the pre-grasp traverse is a pose goal on the redundant
`manipulator` group, so MoveIt's sampler picks the rail and the elbow. Every mode reports `starts`
and `complete` as separate columns and a placement counts as clear only when they are equal.
"Some branch works" and "every branch works" are different answers, and the failures live in the
gap: reduced to best-branch, all four of the campaign's failing seeds look clean.

A third rule: **the answer is sample-size dependent, and the sample size belongs in the report.** `--seeds` is how many random
IK seeds are drawn per pose, and the `starts` column is how many distinct collision-free
configurations came back (typically 30 to 55 of 60). A branch that fails one time in fifty is
invisible at `--seeds 8` and plain at `--seeds 60`. Any "every start completes" result is a
statement about the branches that were sampled, so quote `starts` beside it.

### Build gotchas

`RobotModelLoader` reads its kinematics plugin from
`<robot_description>_kinematics.<group>.kinematics_solver` on the node, and when the descriptions
are supplied as strings (as here) the prefix comes out empty. The probe sets the
parameter overrides under both `robot_description_kinematics.` and `_kinematics.` so the solver
loads either way. The symptom otherwise is `No kinematics plugins defined` followed by every
`setFromIK` failing, which is indistinguishable from a genuinely unreachable pose.

`rclcpp::init` is given an empty argv rather than this program's, because the probe's own options
are not ROS arguments and `rcl` rejects them.

### What it cannot answer

It is a geometry and kinematics instrument. It says nothing about physics, contact, controller
tracking, or timing:

- **Anything two products do to each other on contact.** ODE has no cylinder/cylinder narrowphase
  collider, so products passed through one another until `develop` gained a collider for that
  shape pair. The probe places products as static collision geometry and never simulates contact,
  so it was unaffected by that defect and can neither confirm nor deny anything measured under it.
- **Whether a plan executes.** A path the probe finds collision-free can still abort on a path
  tolerance, a velocity limit, or a deadline. Those live in `analyze.py` and in the acceptance test.
- **Where a product ends up.** The gravity-fed lane carries a released product forward until it
  stops; that is a DART result and the probe has no dynamics.
- **What the planning scene contained at time `t` during a run.** That is `record_state.py`'s
  `.obj`/`.scene` pair.
