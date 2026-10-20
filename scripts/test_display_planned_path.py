#!/usr/bin/env python3
"""Smoke-test / demo node for parallel_multigoal_planner.

What it does, every iteration:

  1. Reads the robot's *actual* joint state off a `sensor_msgs/JointState`
     topic (default `/joint_states`).
  2. Asks `ik_solver` (via `~get_bounds` + `~get_frames` + `~get_fk`) for the
     canonical joint order, its base/tip frame, and the current end-effector
     pose (forward kinematics of the state read in step 1).
  3. Builds `num_goals` alternative Cartesian goals arranged on a horizontal
     circle of radius `radius` around that current pose (same orientation,
     same frame) -- i.e. "targets around the current state".
  4. Sends a `ParallelMultigoalPlan` goal (start = actual state, goals = the
     circle) to `/parallel_multigoal_plan` and waits for the result.
  5. On success, wraps the returned trajectory in a
     `moveit_msgs/DisplayTrajectory` and publishes it on
     `/display_planned_path` so RViz's `MotionPlanning` display (Displays ->
     Add -> moveit_ros_visualization/MotionPlanning, "Trajectory Topic" =
     `display_planned_path`) shows/animates it.

This only *plans*, it never sends anything to a controller -- the robot
never actually moves, so re-running it repeatedly is harmless.

Requires, already running: MoveIt-ish `robot_description`, `ik_solver`, and
`collision_checker_server` -- see docs/01-quickstart.md in this package.

Usage
-----
    ros2 run parallel_multigoal_planner test_display_planned_path.py \\
        --ros-args -p ik_solver_ns:=solver1 -p radius:=0.15 -p num_goals:=6

All parameters (with defaults) are listed in `_declare_parameters()` below.
Pass `-p iterations:=1` for a single plan-and-display shot (the node keeps
spinning afterwards so RViz can still receive the latched last trajectory
message; Ctrl-C to exit).
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point, Pose, Quaternion
from sensor_msgs.msg import JointState
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    DisplayTrajectory,
    JointConstraint,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
    RobotTrajectory,
)

from ik_solver_msgs.srv import GetBound, GetFk, GetFrames
from parallel_multigoal_planner_msgs.action import ParallelMultigoalPlan


class TestDisplayPlannedPath(Node):
    def __init__(self):
        super().__init__("test_display_planned_path")
        self._declare_parameters()

        p = self.get_parameter
        self.ik_solver_ns = p("ik_solver_ns").value
        self.action_name = p("action_name").value
        self.joint_states_topic = p("joint_states_topic").value
        self.display_topic = p("display_planned_path_topic").value
        self.group_name = p("group_name").value
        self.num_goals = int(p("num_goals").value)
        self.radius = float(p("radius").value)
        self.z_offset = float(p("z_offset").value)
        self.allowed_planning_time = float(p("allowed_planning_time").value)
        self.service_timeout_s = float(p("service_timeout_s").value)
        self.verbose = bool(p("verbose").value)
        self.iterations = int(p("iterations").value)  # 0 == forever
        self.period_s = float(p("period_s").value)

        self._latest_joint_state = None
        self.create_subscription(JointState, self.joint_states_topic, self._joint_state_cb, 10)
        
        self.get_logger().info(f"Add service clients to {self.ik_solver_ns}.")
        self.bounds_cli = self.create_client(GetBound, f"/{self.ik_solver_ns}/get_bounds")
        self.frames_cli = self.create_client(GetFrames, f"/{self.ik_solver_ns}/get_frames")
        self.fk_cli = self.create_client(GetFk, f"/{self.ik_solver_ns}/get_fk")

        self.action_client = ActionClient(self, ParallelMultigoalPlan, self.action_name)

        display_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.display_pub = self.create_publisher(DisplayTrajectory, self.display_topic, display_qos)

        self.joint_names = None  # resolved lazily, cached

    def _declare_parameters(self):
        self.declare_parameter("ik_solver_ns", "ik_solver")
        self.declare_parameter("action_name", "parallel_multigoal_plan")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("display_planned_path_topic", "/display_planned_path")
        self.declare_parameter("group_name", "manipulator")  # cosmetic only, see docs/02
        self.declare_parameter("num_goals", 16)
        self.declare_parameter("radius", 0.10)  # meters
        self.declare_parameter("z_offset", 0.0)  # meters, applied to the whole ring
        self.declare_parameter("allowed_planning_time", 8.0)
        self.declare_parameter("service_timeout_s", 5.0)
        self.declare_parameter("verbose", True)
        self.declare_parameter("iterations", 1)  # 0 == loop forever
        self.declare_parameter("period_s", 5.0)

    def _joint_state_cb(self, msg: JointState):
        self._latest_joint_state = msg

    # -- helpers -----------------------------------------------------------

    def wait_for_joint_state(self, timeout_s: float = 10.0) -> JointState:
        """Block (spinning this node) until a JointState has been received."""
        deadline = self.get_clock().now().nanoseconds + int(timeout_s * 1e9)
        while rclpy.ok() and self._latest_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.get_clock().now().nanoseconds > deadline:
                raise RuntimeError(
                    f"No message received on '{self.joint_states_topic}' within {timeout_s}s"
                )
        return self._latest_joint_state

    def resolve_joint_names(self):
        if self.joint_names is not None:
            return self.joint_names
        if not self.bounds_cli.wait_for_service(timeout_sec=self.service_timeout_s):
            raise RuntimeError(f"'{self.bounds_cli.service_name}' service not available")
        future = self.bounds_cli.call_async(GetBound.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_s)
        if future.result() is None:
            raise RuntimeError("get_bounds call failed/timed out")
        self.joint_names = [b.joint_name for b in future.result().boundaries]
        if not self.joint_names:
            raise RuntimeError("get_bounds returned an empty joint set")
        self.get_logger().info(f"Canonical joints: {self.joint_names}")
        return self.joint_names

    def get_frames(self):
        if not self.frames_cli.wait_for_service(timeout_sec=self.service_timeout_s):
            raise RuntimeError(f"'{self.frames_cli.service_name}' service not available")
        future = self.frames_cli.call_async(GetFrames.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_s)
        if future.result() is None:
            raise RuntimeError("get_frames call failed/timed out")
        return future.result()

    def get_current_pose(self, joint_names, ordered_positions, base_frame, tip_frame) -> Pose:
        if not self.fk_cli.wait_for_service(timeout_sec=self.service_timeout_s):
            raise RuntimeError(f"'{self.fk_cli.service_name}' service not available")
        req = GetFk.Request()
        req.configuration.configuration = list(ordered_positions)
        req.joint_names = joint_names
        req.reference_frame = base_frame
        req.tip_frame = tip_frame
        future = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.service_timeout_s)
        if future.result() is None:
            raise RuntimeError("get_fk call failed/timed out")
        return future.result().pose.pose

    @staticmethod
    def reorder_to_canonical(joint_names, js: JointState):
        by_name = dict(zip(js.name, js.position))
        missing = [n for n in joint_names if n not in by_name]
        if missing:
            raise RuntimeError(f"'{js.header.frame_id or 'joint_states'}' is missing joints: {missing}")
        return [by_name[n] for n in joint_names]

    @staticmethod
    def circle_targets(center: Point, radius: float, n: int, z_offset: float, phase: float = 0.0):
        """n points on a horizontal circle of `radius` around `center`."""
        targets = []
        for i in range(n):
            theta = phase + 2.0 * math.pi * i / n
            targets.append(
                Point(
                    x=center.x + radius * math.cos(theta),
                    y=center.y + radius * math.sin(theta),
                    z=center.z + z_offset,
                )
            )
        return targets

    @staticmethod
    def cartesian_alternative(frame_id: str, position: Point, orientation: Quaternion) -> Constraints:
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.constraint_region = BoundingVolume(primitive_poses=[Pose(position=position, orientation=Quaternion(w=1.0))])
        c.position_constraints = [pc]
        oc = OrientationConstraint(header=pc.header, orientation=orientation)
        c.orientation_constraints = [oc]
        return c

    # -- one plan-and-display cycle -----------------------------------------

    def run_once(self, phase: float = 0.0) -> bool:
        joint_names = self.resolve_joint_names()

        js = self.wait_for_joint_state()
        start_positions = self.reorder_to_canonical(joint_names, js)

        frames = self.get_frames()
        base_frame = frames.base_frame
        tip_frame = frames.tool_frame or frames.flange_frame
        self.get_logger().info(f"base_frame='{base_frame}' tip_frame='{tip_frame}'")

        current_pose = self.get_current_pose(joint_names, start_positions, base_frame, tip_frame)
        self.get_logger().info(
            f"Current EE pose in '{base_frame}': "
            f"[{current_pose.position.x:.3f}, {current_pose.position.y:.3f}, {current_pose.position.z:.3f}]"
        )

        targets = self.circle_targets(current_pose.position, self.radius, self.num_goals, self.z_offset, phase)

        goal = ParallelMultigoalPlan.Goal()
        goal.motion_plan_request.start_state.joint_state = JointState(name=joint_names, position=start_positions)
        goal.motion_plan_request.group_name = self.group_name
        goal.motion_plan_request.allowed_planning_time = self.allowed_planning_time
        goal.motion_plan_request.goal_constraints = [
            self.cartesian_alternative(base_frame, pt, current_pose.orientation) for pt in targets
        ]
        goal.verbose = self.verbose

        if not self.action_client.wait_for_server(timeout_sec=self.service_timeout_s):
            raise RuntimeError(f"Action server '{self.action_name}' not available")

        def feedback_cb(feedback_msg):
            fb = feedback_msg.feedback
            self.get_logger().info(f"[{fb.progress:.2f}] {fb.status}")

        send_future = self.action_client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        response = result_future.result().result.motion_plan_response

        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(f"Planning failed, error_code={response.error_code.val}")
            return False

        traj = response.trajectory.joint_trajectory
        self.get_logger().info(f"Got a trajectory with {len(traj.points)} points, publishing on '{self.display_topic}'.")

        display = DisplayTrajectory()
        display.trajectory_start = RobotState(joint_state=goal.motion_plan_request.start_state.joint_state)
        display.trajectory = [RobotTrajectory(joint_trajectory=traj)]
        self.display_pub.publish(display)
        return True


def main():
    rclpy.init()
    node = TestDisplayPlannedPath()
    try:
        i = 0
        while rclpy.ok() and (node.iterations == 0 or i < node.iterations):
            phase = i * (math.pi / 6.0)  # rotate the ring a bit each iteration
            try:
                node.run_once(phase)
            except RuntimeError as exc:
                node.get_logger().error(str(exc))
            i += 1
            if node.iterations == 0 or i < node.iterations:
                node.get_logger().info(f"Sleeping {node.period_s:.1f}s before next iteration (Ctrl-C to stop)...")
                end = node.get_clock().now().nanoseconds + int(node.period_s * 1e9)
                while rclpy.ok() and node.get_clock().now().nanoseconds < end:
                    rclpy.spin_once(node, timeout_sec=0.2)
        node.get_logger().info("Done planning. Keeping the node alive so RViz keeps the last trajectory latched "
                                "(Ctrl-C to exit).")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
