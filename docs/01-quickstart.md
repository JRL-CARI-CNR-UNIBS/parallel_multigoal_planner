# 1. Quickstart

This walks through bringing up the full chain —
MoveIt → `ik_solver` → `parallel_multigoal_planner` — and
confirming the action is reachable, before sending any real goals (that's
tutorials [2](02-sending-goals-python.md) and
[3](03-sending-goals-cpp.md)).

## Prerequisites

You need, for your robot:

- a MoveIt stack (something publishing `robot_description` and serving
  `/get_planning_scene`, normally `move_group`);
- an `ik_solver` node configured for your arm (see the
  [`ik_solver` README](../../ik_solver/README.md)).

Collision checking runs inside the planner, on the planning scene it fetches
from `/get_planning_scene` at each request: no collision checking node is
needed.

## 1. Bring up MoveIt

```bash
ros2 launch <your_robot_moveit_config> demo.launch.py
```

```bash
ros2 service list | grep get_planning_scene
# -> /get_planning_scene
```

## 2. Bring up ik_solver

```bash
ros2 launch ik_solver ik_solver.launch.py config:=/path/to/config.yaml
```

Note the *namespace* your solver is configured under (e.g. `solver1` in the
example config shipped with `ik_solver`) — you'll need it below as
`ik_solver_ns`. Confirm:

```bash
ros2 service list | grep get_task_redundant_ik_array
# -> /solver1/get_task_redundant_ik_array
```

## 3. Bring up parallel_multigoal_planner

```bash
ros2 launch parallel_multigoal_planner parallel_multigoal_planner.launch.py \
  ik_solver_ns:=solver1
```

You should see, on startup:

```
[INFO] [parallel_multigoal_planner]: parallel_multigoal_plan action server ready (ik_solver_ns='solver1', planning_scene_service='/get_planning_scene').
[INFO] [parallel_multigoal_planner]: parallel_multigoal_planner node spinning.
```

## 4. Confirm the action is up

```bash
ros2 action list -t
# -> /parallel_multigoal_plan  [parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan]

ros2 action info /parallel_multigoal_plan
```

At this point the joint model hasn't been resolved yet — that happens lazily
on the *first* goal (a call to `ik_solver`'s `~get_bounds`), not at startup.
If that call is going to fail (wrong `ik_solver_ns`, solver not up yet)
you'll only find out when the first goal aborts with `Cannot resolve joint
model` in the log — worth doing once before relying on it, e.g. by sending
the minimal joint-space goal from
[tutorial 2](02-sending-goals-python.md#a-single-joint-space-goal).

Next: [2. Sending goals from Python](02-sending-goals-python.md).
