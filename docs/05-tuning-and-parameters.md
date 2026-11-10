# 5. Tuning and parameters

There are two layers of parameters at play here: the ones the
`parallel_multigoal_planner` node itself declares (fully wired, set them the
normal ROS 2 way), and `mirrt_star`'s own solver parameters, which are also
ROS 2 parameters of the node, under the `mirrt_star.` prefix (see
[below](#tuning-the-solver-itself)).

## Node parameters

These are declared by `ParallelMultigoalPlannerActionServer` and behave like
any other ROS 2 node parameter — set them via a params file, `-p` on the
command line, or launch-file arguments. Full reference table:
[README → Parameters](../README.md#parameters).

The three most likely to need changing per-robot:

```yaml
/parallel_multigoal_planner:
  ros__parameters:
    ik_solver_ns: "solver1"
    planning_scene_service: "/get_planning_scene"
    default_allowed_planning_time: 5.0
```

```bash
ros2 run parallel_multigoal_planner parallel_multigoal_planner_node \
  --ros-args --params-file params.yaml
```

or via the launch file's arguments (`ik_solver_ns`,
`planning_scene_service` — see
[`launch/parallel_multigoal_planner.launch.py`](../launch/parallel_multigoal_planner.launch.py)).

A few worth knowing about beyond the "must change per robot" set:

- **`service_timeout_s`** — raise this if `get_bounds` /
  `get_task_redundant_ik_array` / `/get_planning_scene` calls are timing
  out under load (a large octomap makes the planning scene slow to fetch).
- **`checker_resolution`** — the distance between the points checked along a
  tree edge by `ParallelMoveitCollisionChecker::checkConnection()`. Smaller
  means more points per edge (safer, slower); larger means fewer, coarser
  checks (faster, more chance of tunnelling through a thin obstacle). This is
  a configuration-space distance, so its right order of magnitude depends on
  your joint bounds/units.
- **`checker_threads`** — threads used to check the points of one edge.
- **`direct_connect_time`** — how long `addGoal()` is allowed to try a
  straight-line connection from the start state to each candidate goal
  before giving up and falling back to tree search for that goal. With many
  candidate goals this runs once per goal, so keep it small.
- **`waypoint_dt_s`** — purely cosmetic time-spacing between trajectory
  points in the result; not velocity-limited (see
  [README → Known limitations](../README.md#known-limitations)). If you
  need real time parameterization, post-process
  `motion_plan_response.trajectory` (e.g. with MoveIt's
  `TimeOptimalTrajectoryGeneration`) before executing it.

## Tuning the solver itself

The solver parameters are node parameters under the `mirrt_star.` prefix
(e.g. `mirrt_star.max_distance`). At every planning request the node copies
them into a `graph::core::ParamMap` and passes it to
`MultigoalSolver::config()`; parameters given to the node under that prefix
are declared automatically. Every key has a default, so the planner also
works without any solver parameter.

[`config/mirrt_star.yaml`](../config/mirrt_star.yaml) lists all the keys
with their defaults; the launch file loads it (argument `mirrt_star_params`):

```bash
ros2 launch parallel_multigoal_planner parallel_multigoal_planner.launch.py \
  mirrt_star_params:=/path/to/my_mirrt_star.yaml
```

Since the parameters are read at every request, they can be changed at
runtime:

```bash
ros2 param set /parallel_multigoal_planner mirrt_star.max_distance 0.5
```

The tunable keys, with their defaults (from
[`graph_core::TreeSolver::config()`](../../graph_core/graph_core/src/graph_core/solvers/tree_solver.cpp)
and
[`MultigoalSolver::config()`](../../mirrt_star/src/mirrt_star/solvers/mirrt_star.cpp)):

| Key | Default | Meaning |
|---|---|---|
| `max_distance` | `1.0` | Max tree-extension step size (configuration-space units). |
| `use_kdtree` | `true` | Use a KD-tree for nearest-neighbour search. |
| `extend` | `false` | Use "extend" instead of "connect" for tree expansion. |
| `utopia_tolerance` | `0.01` | A solution is accepted as optimal once its cost is within `(1 + utopia_tolerance)` of the utopia bound. |
| `rewire_radius` | `2 * max_distance` | RRT*-style rewiring radius. |
| `mixed_strategy` | `true` | Blend tube-informed and plain sampling per goal. |
| `bidirectional` | `true` | Grow trees from both start and each goal. |
| `k_nearest` | `false` | k-nearest vs. radius-based neighbour search. |
| `local_bias` | `0.3` | Bias toward locally-informed sampling around the best-known tube. |
| `apple_weight` | `10.0` | Extra cost weight used by the multi-goal selection policy. |
| `tube_radius` | `0.3` | Radius of the informed "tube" sampler around a candidate solution. |
| `forgetting_factor` | `0.01` | Decay rate for `local_bias` adaptation. |

The goal selection keys (`policy_type`, `policy_name`, `reward_fcn`,
`warm_start_reward`, `epsilon_coef`, `reward_gain`, ...) are listed in
[`config/mirrt_star.yaml`](../config/mirrt_star.yaml).

Messages of graph_core and mirrt\* go to the child logger
`<node name>.graph_core`, so their level can be set separately:

```bash
ros2 run parallel_multigoal_planner parallel_multigoal_planner_node \
  --ros-args --log-level parallel_multigoal_planner.graph_core:=debug
```
