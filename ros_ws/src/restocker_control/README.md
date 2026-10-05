# restocker_control

Owns controller-manager configuration, arm/rail/gripper trajectory controllers, startup ordering,
and controller diagnostics.

## Controller boundary

MoveIt and the smoke client submit `FollowJointTrajectory` goals to the arm, rail, and gripper
joint-trajectory controllers. Those controllers claim position command interfaces exposed by
`gz_ros2_control`; the simulator remains responsible for the joint dynamics. The joint-state
broadcaster publishes measured state for TF and post-command validation.

`control_smoke_test` validates that the expected controllers are active and have claimed the
expected interfaces before sending any trajectory. Controller discovery is a read-only operation:
a response lost during Gazebo startup is retired and retried, but trajectory goals are never
replayed automatically.

After controller readiness, the smoke client discards joint states observed during discovery and
requires a new sample showing both simulated fingers at `0.005 +/- 0.003` m (strictly above the
zero hard stop). The same pure package contract supplies these startup
targets and the later smoke trajectories. No action client is created until the fresh startup state
passes validation.

The discovery retry is split into two layers:

- `ControllerContractProbe` owns deterministic deadlines, retry policy, contract validation, and
  typed outcomes. It has no ROS dependency beyond the existing contract data types.
- `RclcppControllerProbeTransport` owns service discovery, request identity, bounded spinning,
  pending-request cleanup, and interruptible waits.

Both layers use steady time. ROS or simulation clock changes cannot extend the startup deadline.

## Smoke-test configuration

The installed `control_smoke_test` node accepts these parameters:

| Parameter | Default | Meaning |
| --- | ---: | --- |
| `startup_timeout_sec` | `20.0` | One overall budget shared by controller discovery and contract probes. |
| `controller_response_attempt_timeout_sec` | `1.0` | Maximum wait for one `ListControllers` response. |
| `action_timeout_sec` | `5.0` | Budget for action discovery, responses, and final-state observation. |
| `execute_trajectory` | `true` | When false, stop after validating the controller contract. |

Timeout values must be finite, positive, representable as nanoseconds, and the response-attempt
timeout must not exceed the overall startup timeout. Invalid values fail before service discovery.

## Diagnostics

The process returns distinct exit codes so launch tests and operators can separate failure classes:

| Code | Meaning |
| ---: | --- |
| `0` | Contract, trajectory execution, and final-state checks succeeded. |
| `10` | Controller service discovery exhausted the startup deadline. |
| `11` | No controller-list response arrived before the deadline. |
| `12` | Responses arrived, but the controller contract remained unhealthy. |
| `13` | ROS context shutdown interrupted discovery or a bounded wait. |
| `14` | The controller service transport failed. |
| `15` | Timeout configuration was invalid. |
| `20`-`24` | Trajectory action discovery, goal, result, or execution failed. |
| `30`-`31` | Final joint-state observation timed out or did not reach commanded targets. |
| `32` | No fresh joint state arrived after controller readiness. |
| `33` | Fresh joint states arrived, but the finger initialization remained invalid. |

Timed-out requests are removed by rclcpp request ID before retry. A late response cannot
be mistaken for a later attempt. Cleanup contention is reported because it means rclcpp had already retired the entry, usually
because the response raced the timeout.
