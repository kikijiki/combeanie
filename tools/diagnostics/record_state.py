#!/usr/bin/env python3
"""Log arm controller reference/feedback/error per cycle to CSV for abort forensics.

`.obj` says where a product really is; `.scene` says where MoveIt believes it is at the same
instant. A path that executes into a product is either absent from `.scene`, present at a pose
`.obj` disagrees with, or present and correct. These are different faults.
"""

import contextlib
import sys

from control_msgs.msg import JointTrajectoryControllerState
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from restocker_interfaces.msg import ObjectObservation, PlanningSceneProjectionStatus
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState


class Recorder(Node):
    def __init__(self, path):
        super().__init__("arm_state_recorder")
        self.f = open(path, "w", buffering=1)
        self.names = None
        self.prev_clock = None
        self.clock_f = open(path + ".clock", "w", buffering=1)
        qos = QoSProfile(
            depth=200, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(
            JointTrajectoryControllerState, "/arm_controller/controller_state", self.on_state, qos
        )
        self.rail_f = open(path + ".rail", "w", buffering=1)
        self.create_subscription(
            JointTrajectoryControllerState, "/rail_controller/controller_state", self.on_rail, qos
        )
        self.js_f = open(path + ".js", "w", buffering=1)
        self.js_names = None
        self.create_subscription(JointState, "/joint_states", self.on_js, qos)
        self.obj_f = open(path + ".obj", "w", buffering=1)
        self.obj_f.write("stamp,frame,source_object_id,x,y,z,qx,qy,qz,qw\n")
        self.create_subscription(
            ObjectObservation, "/perception/object_observations", self.on_obj, qos
        )
        best = QoSProfile(
            depth=500, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST
        )
        self.create_subscription(Clock, "/clock", self.on_clock, best)

        # Scene contents, read from the service the projector verifies against. Polled instead
        # of taken from /monitored_planning_scene, which carries diffs that would need replaying.
        self.scene_f = open(path + ".scene", "w", buffering=1)
        self.scene_f.write("stamp,kind,object_id,x,y,z,dims\n")
        self.scene_group = MutuallyExclusiveCallbackGroup()
        self.scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene", callback_group=self.scene_group
        )
        self.scene_pending = None
        self.create_timer(0.25, self.on_scene_timer, callback_group=self.scene_group)

        self.status_f = open(path + ".status", "w", buffering=1)
        self.status_f.write("stamp,state,error_code,applied_revision,content_generation,detail\n")
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            PlanningSceneProjectionStatus,
            "/planning_scene_projection/status",
            self.on_status,
            latched,
        )

    def on_status(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        detail = msg.detail.replace(",", ";").replace("\n", " ")
        self.status_f.write(
            f"{t:.6f},{msg.state},{msg.error_code},{msg.applied_revision},"
            f"{msg.scene_content_generation},{detail}\n"
        )

    def on_scene_timer(self):
        if self.scene_pending is not None and not self.scene_pending.done():
            return
        if not self.scene_client.service_is_ready():
            return
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        self.scene_pending = self.scene_client.call_async(request)
        self.scene_pending.add_done_callback(self.on_scene)

    def on_scene(self, future):
        try:
            scene = future.result().scene
        except Exception:
            return
        # Stamped from the simulation clock; the node itself runs on the system clock.
        t = self.prev_clock if self.prev_clock is not None else 0.0
        for obj in scene.world.collision_objects:
            self.write_scene_object(t, "world", obj)
        for attached in scene.robot_state.attached_collision_objects:
            self.write_scene_object(t, "attached", attached.object)

    def write_scene_object(self, t, kind, obj):
        # A primitive pose is relative to obj.pose. The projector emits obj.pose at the product
        # with the primitive pose at identity, so the world position is their sum. Rotation is
        # not recorded: product collision cylinders are axis-aligned here.
        offset = (0.0, 0.0, 0.0)
        if obj.primitive_poses:
            local = obj.primitive_poses[0].position
            offset = (local.x, local.y, local.z)
        x = obj.pose.position.x + offset[0]
        y = obj.pose.position.y + offset[1]
        z = obj.pose.position.z + offset[2]
        dims = "|".join(";".join(f"{d:.4f}" for d in prim.dimensions) for prim in obj.primitives)
        self.scene_f.write(f"{t:.6f},{kind},{obj.id},{x:.5f},{y:.5f},{z:.5f},{dims}\n")

    def on_clock(self, msg):
        import time as _time

        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.prev_clock is not None:
            dt = t - self.prev_clock
            if dt <= 0.0009 or dt > 0.0011:
                self.clock_f.write(f"JUMP,{self.prev_clock:.6f},{t:.6f},{dt:.6f}\n")
        self.prev_clock = t
        self.clock_n = getattr(self, "clock_n", 0) + 1
        if self.clock_n % 100 == 0:
            self.clock_f.write(f"RTF,{t:.6f},{_time.monotonic():.6f}\n")

    def on_obj(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self.obj_f.write(
            f"{t:.6f},{msg.header.frame_id},{msg.source_object_id},{p.x:.5f},{p.y:.5f},"
            f"{p.z:.5f},{o.x:.5f},{o.y:.5f},{o.z:.5f},{o.w:.5f}\n"
        )

    def on_js(self, msg):
        if self.js_names is None:
            self.js_names = list(msg.name)
            self.js_f.write("stamp," + ",".join(self.js_names) + "\n")
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        by_name = dict(zip(msg.name, msg.position, strict=False))
        self.js_f.write(
            f"{t:.6f},"
            + ",".join(f"{by_name.get(n, float('nan')):.6f}" for n in self.js_names)
            + "\n"
        )

    def on_rail(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        def g(seq, i):
            try:
                return f"{seq[i]:.6f}"
            except Exception:
                return ""

        self.rail_f.write(
            f"{t:.6f},{g(msg.reference.positions, 0)},{g(msg.feedback.positions, 0)},"
            f"{g(msg.error.positions, 0)},{g(msg.feedback.velocities, 0)},"
            f"{g(msg.output.velocities, 0)}\n"
        )

    def on_state(self, msg):
        if self.names is None:
            self.names = list(msg.joint_names)
            cols = ["stamp"]
            for n in self.names:
                cols += [f"{n}_ref", f"{n}_fb", f"{n}_err", f"{n}_refv", f"{n}_fbv", f"{n}_out"]
            self.f.write(",".join(cols) + "\n")
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        row = [f"{t:.6f}"]

        def g(seq, i):
            try:
                return f"{seq[i]:.6f}"
            except Exception:
                return ""

        for i in range(len(self.names)):
            row += [
                g(msg.reference.positions, i),
                g(msg.feedback.positions, i),
                g(msg.error.positions, i),
                g(msg.reference.velocities, i),
                g(msg.feedback.velocities, i),
                g(msg.output.velocities, i),
            ]
        self.f.write(",".join(row) + "\n")


def main():
    rclpy.init()
    node = Recorder(sys.argv[1])
    with contextlib.suppress(KeyboardInterrupt):
        rclpy.spin(node)


if __name__ == "__main__":
    main()
