# 5. Tuning and parameters

There are two layers of parameters at play here: the ones the
`parallel_multigoal_planner` node itself declares (fully wired, set them the
normal ROS 2 way), and `mirrt_star`'s own internal solver parameters (read
through `cnr_param`, currently running with their compiled-in defaults —
see [below](#tuning-the-solver-itself) before assuming you can override
them from a YAML file).

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
    collision_check_service: "/collision_check"
    default_allowed_planning_time: 5.0
```

```bash
ros2 run parallel_multigoal_planner parallel_multigoal_planner_node \
  --ros-args --params-file params.yaml
```

or via the launch file's arguments (`ik_solver_ns`,
`collision_check_service` — see
[`launch/parallel_multigoal_planner.launch.py`](../launch/parallel_multigoal_planner.launch.py)).

A few worth knowing about beyond the "must change per robot" set:

- **`service_timeout_s`** — raise this if `get_bounds` /
  `get_task_redundant_ik_array` / `/collision_check` calls are timing out
  under load (a busy `collision_checker_server` batching many points per
  edge check can be slow on a large scene).
- **`checker_min_distance`** — the edge-sampling step used by
  `ServiceCollisionChecker::checkConnection()`. Smaller means more points
  per tree edge sent to `/collision_check` (safer, slower); larger means
  fewer, coarser checks (faster, more chance of tunnelling through a thin
  obstacle). This is a configuration-space distance, so its right order of
  magnitude depends on your joint bounds/units.
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

`MultigoalSolver::config(mirrt_star_param_ns)` reads its parameters through
`cnr_param` under the namespace given by the `mirrt_star_param_ns` node
parameter (default `/parallel_multigoal_planner/mirrt_star`). Every one of
them has a hardcoded fallback, so **as shipped, this package always plans
with mirrt_star's compiled-in defaults** — no separate solver config file is
required to get a working planner.

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

If you need to change any of these, `cnr_param` supports two mechanisms
(see the [`cnr_param` README](../../cnr_param/README.md)) — **neither is
currently wired up in this package**, so pick one and wire it in
`parallel_multigoal_planner_action_server.cpp` before relying on it:

- **A mapped YAML file** — `cnr_param`'s `MAPPED_FILE` backend, independent
  of ROS. This is how `collision_checker_server` reads its `padding`/
  `scaling` parameters and how `ik_solver` reads its per-solver config.
- **Live ROS 2 parameters** — requires calling
  `cnr::param::ros2::CNR_PARAM_INIT_ROS2_MODULE(node)` once (with a
  `std::shared_ptr<rclcpp::Node>&`, from `<cnr_param/ros2/param.h>`) before
  the first `solver->config(...)` call; only then does `cnr::param`'s ROS2
  backend actually query the parameter server. Without that call,
  `graph::core::get_param()`'s `cnr::param::is_ros2_avaliable()` check is
  false and every lookup silently falls back to the compiled-in default —
  which is exactly the (harmless) behavior you'll see today.

Until one of those is wired in, the only way to change solver behavior is
to edit the defaults in `mirrt_star`'s source directly, or to construct the
`MultigoalSolver` with your own already-configured `Metrics`/`Sampler`
instances (`ParallelMultigoalPlannerActionServer::execute()` is the place to
do that — see [Architecture](04-architecture.md#class-responsibilities)).
