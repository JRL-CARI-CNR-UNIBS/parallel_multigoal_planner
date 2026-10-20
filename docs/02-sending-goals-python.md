# 2. Sending goals from Python (rclpy)

All examples below assume the chain from
[tutorial 1](01-quickstart.md) is running, and use the same
`ik_solver_ns` (`"solver1"`) to fetch the canonical joint list via
`~get_bounds` — exactly what `parallel_multigoal_planner` itself does
internally, so the joint order always matches what the action expects.

## Common setup

```python
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from ik_solver_msgs.srv import GetBound

from parallel_multigoal_planner_msgs.action import ParallelMultigoalPlan

from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
)
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Quaternion


class PlannerClient(Node):
    def __init__(self, ik_solver_ns: str):
        super().__init__("parallel_multigoal_planner_client")
        self.ik_solver_ns = ik_solver_ns

        bounds_cli = self.create_client(GetBound, f"/{ik_solver_ns}/get_bounds")
        bounds_cli.wait_for_service()
        future = bounds_cli.call_async(GetBound.Request())
        rclpy.spin_until_future_complete(self, future)
        boundaries = future.result().boundaries

        self.joint_names = [b.joint_name for b in boundaries]
        self.get_logger().info(f"Canonical joints: {self.joint_names}")

        self.action_client = ActionClient(self, ParallelMultigoalPlan, "parallel_multigoal_plan")
        self.action_client.wait_for_server()

    def start_state(self, positions):
        js = JointState()
        js.name = self.joint_names
        js.position = positions
        return js
```

`positions` must give a value for every joint in `self.joint_names` — that's
the canonical order the whole planner uses (see the package README's
[Known limitations](../README.md#known-limitations)).

## A single joint-space goal

The simplest possible request: one alternative, a fixed joint configuration.
Good for the "does the chain actually work" smoke test mentioned at the end
of [tutorial 1](01-quickstart.md#5-confirm-the-action-is-up).

```python
def send_joint_goal(client: PlannerClient, start_positions, goal_positions):
    goal = ParallelMultigoalPlan.Goal()
    goal.motion_plan_request.start_state.joint_state = client.start_state(start_positions)
    goal.motion_plan_request.group_name = "manipulator"  # cosmetic: copied into the response
    goal.motion_plan_request.allowed_planning_time = 5.0
    goal.verbose = True

    c = Constraints()
    c.joint_constraints = [
        JointConstraint(joint_name=name, position=pos, weight=1.0)
        for name, pos in zip(client.joint_names, goal_positions)
    ]
    goal.motion_plan_request.goal_constraints = [c]

    return goal
```

## A single Cartesian goal

`position_constraints` and `orientation_constraints` must **both** be set
(and `joint_constraints` left empty) for an element to be treated as a
Cartesian goal. The pose is passed straight through to `ik_solver`'s
`~get_task_redundant_ik_array` — it must already be expressed in
`ik_solver`'s own base frame (see
[Known limitations](../README.md#known-limitations); there is no TF
transform step).

```python
def send_cartesian_goal(client: PlannerClient, start_positions, target_frame, position, orientation):
    goal = ParallelMultigoalPlan.Goal()
    goal.motion_plan_request.start_state.joint_state = client.start_state(start_positions)
    goal.motion_plan_request.group_name = "manipulator"
    goal.motion_plan_request.allowed_planning_time = 5.0
    goal.verbose = True

    c = Constraints()

    pc = PositionConstraint()
    pc.header.frame_id = target_frame  # must be ik_solver's base frame
    pc.constraint_region = BoundingVolume(primitive_poses=[])
    # only .position is read; a primitive isn't required, but at least one
    # primitive_pose must be present to carry it
    from geometry_msgs.msg import Pose
    pc.constraint_region.primitive_poses = [Pose(position=position, orientation=Quaternion(w=1.0))]
    c.position_constraints = [pc]

    oc = OrientationConstraint()
    oc.header.frame_id = target_frame
    oc.orientation = orientation
    c.orientation_constraints = [oc]

    goal.motion_plan_request.goal_constraints = [c]
    return goal
```

Example values:

```python
position = Point(x=0.4, y=0.1, z=0.5)
orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
```

## Mixing several alternative goals in one request

This is the actual point of the action: `goal_constraints` is a list of
independent *alternatives*, not requirements that must all hold together.
Here we offer three alternatives — two Cartesian, one joint-space — and let
the planner pick whichever is cheapest to reach:

```python
def send_mixed_goal(client: PlannerClient, start_positions, target_frame):
    goal = ParallelMultigoalPlan.Goal()
    goal.motion_plan_request.start_state.joint_state = client.start_state(start_positions)
    goal.motion_plan_request.allowed_planning_time = 8.0
    goal.verbose = True

    from geometry_msgs.msg import Pose

    def cartesian_alt(position, orientation):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = target_frame
        pc.constraint_region = BoundingVolume(primitive_poses=[Pose(position=position, orientation=Quaternion(w=1.0))])
        c.position_constraints = [pc]
        oc = OrientationConstraint(header=pc.header, orientation=orientation)
        c.orientation_constraints = [oc]
        return c

    def joint_alt(positions):
        c = Constraints()
        c.joint_constraints = [
            JointConstraint(joint_name=n, position=p, weight=1.0)
            for n, p in zip(client.joint_names, positions)
        ]
        return c

    goal.motion_plan_request.goal_constraints = [
        cartesian_alt(Point(x=0.4, y=0.1, z=0.5), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        cartesian_alt(Point(x=0.3, y=-0.2, z=0.6), Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)),
        joint_alt([0.0] * len(client.joint_names)),
    ]
    return goal
```

## Sending it, with feedback and a result

```python
def run(client: PlannerClient, goal):
    def feedback_cb(feedback_msg):
        fb = feedback_msg.feedback
        client.get_logger().info(f"[{fb.progress:.2f}] {fb.status}")

    send_future = client.action_client.send_goal_async(goal, feedback_callback=feedback_cb)
    rclpy.spin_until_future_complete(client, send_future)
    goal_handle = send_future.result()

    if not goal_handle.accepted:
        client.get_logger().error("Goal rejected")
        return None

    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(client, result_future)
    result = result_future.result().result

    err = result.motion_plan_response.error_code.val
    if err != 1:  # moveit_msgs.msg.MoveItErrorCodes.SUCCESS
        client.get_logger().error(f"Planning failed, error_code={err}")
        return None

    traj = result.motion_plan_response.trajectory.joint_trajectory
    client.get_logger().info(f"Got a trajectory with {len(traj.points)} points.")
    return result.motion_plan_response


def main():
    rclpy.init()
    client = PlannerClient(ik_solver_ns="solver1")

    start = [0.0] * len(client.joint_names)
    goal = send_joint_goal(client, start, [0.2] * len(client.joint_names))
    run(client, goal)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
```

## Cancelling a goal

```python
send_future = client.action_client.send_goal_async(goal)
rclpy.spin_until_future_complete(client, send_future)
goal_handle = send_future.result()

# ... later ...
cancel_future = goal_handle.cancel_goal_async()
rclpy.spin_until_future_complete(client, cancel_future)
```

The server checks for cancellation between solver iterations (see
[Architecture](04-architecture.md)) and responds with
`error_code.val == moveit_msgs.msg.MoveItErrorCodes.PREEMPTED` (`-7`).

Next: [3. Sending goals from C++](03-sending-goals-cpp.md), or skip to
[4. Architecture](04-architecture.md) for what happens on the server side.
