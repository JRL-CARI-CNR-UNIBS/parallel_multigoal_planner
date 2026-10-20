# parallel_multigoal_planner

A ROS 2 action server that plans a motion reaching **any one** of several
alternative goals — Cartesian or joint-space, mixed freely in the same
request — rather than a single fixed goal. It resolves every Cartesian
alternative to a set of collision-free joint configurations via
[`ik_solver`](https://github.com/JRL-CARI-CNR-UNIBS/ik_solver) and
[`collision_checker_server`](../collision_check_server), then hands the whole
combined candidate set to
[`mirrt_star`](../mirrt_star)'s `MultigoalSolver` to find a path to whichever
one is cheapest to reach.

- **Package:** `parallel_multigoal_planner`
- **Node executable:** `parallel_multigoal_planner_node`
- **Action:** `parallel_multigoal_plan` → globally `/parallel_multigoal_plan`
- **Action type:** `parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan`
- **License:** BSD-3-Clause
- **Maintainer:** Manuel Beschi (manuel.beschi@unibs.it)

See [`parallel_multigoal_planner_msgs`](../parallel_multigoal_planner_msgs)
for the action definition, and [`docs/`](docs) for step-by-step tutorials
(sending goals from Python/C++, the internal pipeline, tuning parameters).

---

## Table of contents

- [How it works](#how-it-works)
- [Package layout](#package-layout)
- [Dependencies](#dependencies)
- [Building](#building)
- [Running](#running)
- [Action API](#action-api)
- [Parameters](#parameters)
- [Concurrency model](#concurrency-model)
- [Known limitations](#known-limitations)
- [Troubleshooting](#troubleshooting)
- [Tutorials](docs/)

## How it works

On each accepted goal, `ParallelMultigoalPlannerActionServer::execute()` runs
on its own detached thread and does, in order:

1. **Resolve the joint model** (once, cached): calls `ik_solver`'s
   `~get_bounds` service to fetch the canonical joint name list and position
   bounds. This ordering is used for every `Eigen::VectorXd` in the planner —
   the start state, every candidate goal, the sampler bounds, and the
   returned trajectory.

2. **Build the candidate goal set** — for every element `c` of
   `motion_plan_request.goal_constraints`:
   - if `c.joint_constraints` is set, it becomes one candidate directly (no
     IK or collision check — mirrt_star still validates it when the goal is
     registered with the solver);
   - otherwise (`c.position_constraints` + `c.orientation_constraints`, a
     Cartesian goal), it calls `ik_solver`'s `~get_task_redundant_ik_array`
     service to get an array of IK solutions, then batches all of them into
     one call to `collision_checker_server`'s `/collision_check` service and
     keeps only the collision-free, in-bounds ones.

   Both service calls run on their own `MutuallyExclusiveCallbackGroup`
   using async requests (`async_send_request()` + `future.wait_for()`), so
   the action server's own goal/cancel callbacks are never blocked behind
   them.

3. **Plan** — builds a fresh `graph::mirrt_star::MultigoalSolver` per goal
   (metrics: `EuclideanMetrics`; sampler: `UniformSampler` over the joint
   bounds from step 1; collision checker: see below), registers the start
   state and every surviving candidate goal, then drives the solver's
   `update()` loop directly (instead of the blocking `TreeSolver::solve()`)
   so it can check `is_canceling()` and publish feedback every
   `feedback_period_iterations`.

4. **Convert** the resulting `graph::core::Path` into a
   `moveit_msgs/MotionPlanResponse` (a `JointTrajectory` with the start state
   prepended, `waypoint_dt_s` seconds apart) and return it as the action
   result.

### Where collision checking comes from

`mirrt_star`'s `MultigoalSolver` needs a `graph::core::CollisionCheckerPtr`
for its own internal tree-extension checks — this is a *different* need from
the goal-set filtering in step 2 above, but this package reuses the same
service for both: `ServiceCollisionChecker` (in
`include/parallel_multigoal_planner/service_collision_checker.hpp`) is a
`graph::core::CollisionCheckerBase` implementation that forwards every check
to `collision_checker_server`'s `/collision_check` service, batching all the
sampled points of a tree edge into a single request. This is why the package
needs no MoveIt/planning-scene dependency of its own (unlike, e.g.,
`moveit_collision_checker`) — every collision query, whether filtering IK
solutions or extending the search tree, goes through the one already-running
`collision_checker_server` node.

## Package layout

```
parallel_multigoal_planner/
├── include/parallel_multigoal_planner/
│   ├── joint_order_utils.hpp                     # (names,values) -> canonical-order Eigen::VectorXd
│   ├── service_collision_checker.hpp             # graph_core CollisionCheckerBase over /collision_check
│   ├── goal_set_builder.hpp                      # goal_constraints[] -> candidate joint-space goals
│   └── parallel_multigoal_planner_action_server.hpp
├── src/
│   ├── parallel_multigoal_planner/
│   │   ├── service_collision_checker.cpp
│   │   ├── goal_set_builder.cpp
│   │   └── parallel_multigoal_planner_action_server.cpp
│   └── parallel_multigoal_planner_node.cpp       # main(): MultiThreadedExecutor + action server
├── config/logger_param.yaml                      # cnr_logger config
├── launch/parallel_multigoal_planner.launch.py
├── docs/                                          # tutorials, see docs/README.md
├── CMakeLists.txt
└── package.xml
```

## Dependencies

ROS 2 / MoveIt:
- `rclcpp`, `rclcpp_action`, `moveit_msgs`, `trajectory_msgs`, `ament_index_cpp`

Interfaces:
- `parallel_multigoal_planner_msgs` (this action's definition)
- `ik_solver_msgs` (`GetBound`, `GetIkArray`, `CollisionChecking` service/message types)

Planning:
- `graph_core`, `mirrt_star` (`graph::mirrt_star::MultigoalSolver`)
- `cnr_logger`

At runtime, the node also expects, on the ROS graph:
- an `ik_solver` node exposing `~get_bounds` and
  `~get_task_redundant_ik_array` (declared as an `exec_depend`, not linked
  against — this package only talks to it over ROS services);
- a `collision_checker_server` node exposing `/collision_check` (same:
  `exec_depend` only).

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select parallel_multigoal_planner_msgs parallel_multigoal_planner
source install/setup.bash
```

## Running

Bring up `ik_solver` and `collision_checker_server` for your robot first
(see their own READMEs), then:

```bash
ros2 launch parallel_multigoal_planner parallel_multigoal_planner.launch.py \
  ik_solver_ns:=solver1 collision_check_service:=/collision_check
```

or run the node directly with parameters on the command line / a params
file — see [Parameters](#parameters).

Confirm the action is up:

```bash
ros2 action list -t
# -> /parallel_multigoal_plan  [parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan]
```

## Action API

`parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan`:

```
# Request
moveit_msgs/MotionPlanRequest motion_plan_request
bool verbose
---
# Result
moveit_msgs/MotionPlanResponse motion_plan_response
---
# Feedback
string status
float32 progress
```

`motion_plan_request.goal_constraints` is a list of **independent,
alternative** goals (OR'd, not combined): if it has length N, there are N
candidate goals, and each element must specify *either* `joint_constraints`
*or* both `position_constraints` and `orientation_constraints` — never both,
never neither. See [`docs/02-sending-goals-python.md`](docs/02-sending-goals-python.md)
and [`docs/03-sending-goals-cpp.md`](docs/03-sending-goals-cpp.md) for full
worked examples of building each kind of goal.

`motion_plan_request.start_state.joint_state` must name every joint returned
by `ik_solver`'s `~get_bounds` (extra joints are ignored). If
`allowed_planning_time <= 0`, `default_allowed_planning_time` is used
instead.

On failure, `motion_plan_response.error_code.val` is one of the standard
`moveit_msgs/MoveItErrorCodes` values (`INVALID_ROBOT_STATE`,
`INVALID_GOAL_CONSTRAINTS`, `NO_IK_SOLUTION`, `START_STATE_IN_COLLISION`,
`PLANNING_FAILED`, `PREEMPTED` on cancellation, ...).

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `ik_solver_ns` | string | `"ik_solver"` | Namespace of the `ik_solver` node; services are resolved as `/<ik_solver_ns>/get_bounds` and `/<ik_solver_ns>/get_task_redundant_ik_array`. |
| `collision_check_service` | string | `"/collision_check"` | Full name of `collision_checker_server`'s service. |
| `service_timeout_s` | double | `5.0` | Timeout for every individual service call (bounds/IK/collision). |
| `default_allowed_planning_time` | double | `5.0` | Used when `motion_plan_request.allowed_planning_time <= 0`. |
| `direct_connect_time` | double | `0.2` | Per-goal timeout given to `MultigoalSolver::addGoal()`'s straight-line direct-connection attempt from the start state. |
| `max_iterations` | int | `1000000` | Hard cap on solver `update()` iterations (the time budget above is the effective limit in practice). |
| `feedback_period_iterations` | int | `200` | Publish an action feedback message every N solver iterations. |
| `waypoint_dt_s` | double | `0.5` | Fixed spacing (seconds) between consecutive `JointTrajectoryPoint`s in the result — a placeholder timing scheme, not a velocity-limited one. |
| `checker_min_distance` | double | `0.05` | Sampling step (in configuration-space units) used when batching a tree edge into points for `/collision_check`. |
| `mirrt_star_param_ns` | string | `"/parallel_multigoal_planner/mirrt_star"` | `cnr_param` namespace passed to `MultigoalSolver::config()`. Left unpopulated, mirrt_star's own defaults apply (`max_distance`, `rewire_radius`, `bidirectional`, `tube_radius`, ...) — populate params under this namespace to tune the solver, see [`docs/05-tuning-and-parameters.md`](docs/05-tuning-and-parameters.md). |
| `logger_config_file` | string | `<share>/config/logger_param.yaml` | `cnr_logger` config file. |

## Concurrency model

- `main()` runs an `rclcpp::executors::MultiThreadedExecutor` spinning the
  node.
- Each accepted goal is executed on its own **detached `std::thread`**
  (`handle_accepted()` → `execute()`), never on an executor thread — so
  blocking service calls inside `execute()` (via `future.wait_for()`) never
  starve the executor, and multiple goals can genuinely run concurrently.
- The three service clients (`get_bounds`, `get_task_redundant_ik_array`,
  `/collision_check`) each sit on their own `MutuallyExclusiveCallbackGroup`,
  so their responses are never queued behind the action server's own
  goal/cancel callbacks or behind each other.
- Per-goal planning state (`MultigoalSolver`, `Metrics`, `Sampler`, `Node`s)
  is constructed fresh inside `execute()` — nothing is shared/mutated across
  concurrent goals except the joint-model cache (guarded by a mutex, resolved
  once) and the collision-checker/service clients (stateless wrappers around
  thread-safe `rclcpp::Client`s).

## Known limitations

- Cartesian goal poses (`position_constraints`/`orientation_constraints`)
  are passed to `get_task_redundant_ik_array` exactly as given — in the
  frame named by the constraint's own `header.frame_id` — with **no TF
  transform**. They must already be expressed in `ik_solver`'s base frame.
- The output trajectory's timing (`waypoint_dt_s` spacing) is a fixed
  placeholder, not velocity/acceleration-limited time parameterization.
- `MultigoalSolver::update()`'s loop is stopped as soon as *any* improving
  update occurs (matching `graph_core::TreeSolver`'s own default `solve()`
  semantics) — this is a first-solution planner, not one that keeps
  refining for the full time budget once a solution is found.
- `start_state.joint_state` must contain every joint of the canonical joint
  set (from `ik_solver`'s `~get_bounds`); partial start states are rejected.

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Goal rejected immediately, `goal_constraints is empty` | The request's `motion_plan_request.goal_constraints` array has no elements. |
| Aborts with `INVALID_ROBOT_STATE` | `start_state.joint_state` is missing one or more of the joints reported by `ik_solver`'s `~get_bounds`. |
| Aborts with `NO_IK_SOLUTION` | Every Cartesian goal element failed to yield a collision-free IK solution, and no joint-space element was provided either. |
| `Cannot resolve joint model` | `ik_solver`'s `~get_bounds` service isn't reachable — check `ik_solver_ns` and that the solver node is running. |
| `/collision_check service is not available` (repeated) | `collision_checker_server` isn't running, or `collision_check_service` doesn't match its actual service name. |
| Planning always times out | Check `default_allowed_planning_time`/`allowed_planning_time`, and whether any goal actually registered (`Registered N candidate goal configuration(s)` info log with `verbose:=true`). |

---

## API documentation

The API reference of the classes, functions and scripts is in [docs/API.md](docs/API.md) (generated from the
Doxygen comments).

The code is documented with Doxygen. From the package folder run `doxygen Doxyfile` and open
`doxygen/html/index.html`. The license text is in [LICENSE](LICENSE).
