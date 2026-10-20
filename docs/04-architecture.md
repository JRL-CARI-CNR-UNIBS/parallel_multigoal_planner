# 4. Architecture

## Sequence, end to end

```mermaid
sequenceDiagram
    participant Client
    participant Server as ParallelMultigoalPlannerActionServer
    participant Bounds as ik_solver:~get_bounds
    participant IK as ik_solver:~get_task_redundant_ik_array
    participant CC as collision_checker_server:/collision_check
    participant Solver as MultigoalSolver (mirrt_star)

    Client->>Server: send_goal(ParallelMultigoalPlan)
    Server->>Server: handle_goal() — reject if goal_constraints empty
    Server-->>Client: ACCEPT_AND_EXECUTE
    Server->>Server: handle_accepted() spawns detached thread -> execute()

    Note over Server,Bounds: once per node lifetime (cached)
    Server->>Bounds: GetBound.Request()
    Bounds-->>Server: joint_names, lower/upper bounds

    Server->>Server: start_q = reorder(start_state, joint_names)

    loop for each element c of goal_constraints
        alt c.joint_constraints set
            Server->>Server: candidate = reorder(c.joint_constraints, joint_names)
        else c.position/orientation_constraints set
            Server->>IK: GetIkArray.Request(target pose)
            IK-->>Server: IkSolution[] (many joint configs)
            Server->>CC: CollisionChecking.Request(all configs, batched)
            CC-->>Server: feasibility per config
            Server->>Server: keep collision-free configs as candidates
        end
    end

    Server->>Solver: new MultigoalSolver(metrics, checker, sampler, ...)
    Server->>Solver: config(), addStart(start), addGoal(...) per candidate
    Server->>Solver: finalizeProblem()

    loop until solved, cancelled, or timeout
        Server->>Server: is_canceling()?
        Server->>Solver: update(solution)
        Solver->>CC: (internally, via ServiceCollisionChecker) batched edge checks
        CC-->>Solver: feasibility
        Server-->>Client: feedback(status, progress)  [every feedback_period_iterations]
    end

    Server->>Server: path -> MotionPlanResponse (JointTrajectory)
    Server-->>Client: result(motion_plan_response)
```

## Class responsibilities

| Class | File | Responsibility |
|---|---|---|
| `ParallelMultigoalPlannerActionServer` | `parallel_multigoal_planner_action_server.{hpp,cpp}` | Owns the action server, the three service clients, orchestrates one `execute()` per goal, converts `Path` → `MotionPlanResponse`. |
| `GoalSetBuilder` | `goal_set_builder.{hpp,cpp}` | Turns `goal_constraints[]` into a flat `std::vector<Eigen::VectorXd>` of candidate joint-space goals. Owns the `~get_bounds` / `~get_task_redundant_ik_array` / `/collision_check` request/response plumbing for that purpose. |
| `ServiceCollisionChecker` | `service_collision_checker.{hpp,cpp}` | A `graph::core::CollisionCheckerBase` that forwards `check()`/`checkConnection()` to `/collision_check`, batching a whole tree edge into one request. Used **by the solver itself**, not by `GoalSetBuilder` (which calls `/collision_check` directly for a different reason — filtering a batch of IK solutions once, not repeatedly during tree search). |
| `reorderByJointName()` | `joint_order_utils.hpp` | The one place that maps a `(names, values)` pair (from a `JointState`, a `JointConstraint[]`, or an IK response) onto the canonical joint order. Every `Eigen::VectorXd` in the planner goes through this. |

## Why two different collision-check call sites?

It's the same ROS service (`/collision_check`) reached two different ways,
for two different access patterns:

- **`GoalSetBuilder::filterCollisionFree()`** — called once per Cartesian
  `goal_constraints` element, with *all* of that element's IK solutions
  batched into a single request. This happens before any planning starts.
- **`ServiceCollisionChecker`** — called by `MultigoalSolver` itself,
  potentially thousands of times per goal, as it extends its search tree.
  `checkConnection()` batches the sampled points of one tree edge into one
  request (governed by `checker_min_distance`), which is the hot path, but
  it's still one service round-trip per edge — see
  [Known limitations](../README.md#known-limitations) and
  [tuning](05-tuning-and-parameters.md) if this turns out to be a
  bottleneck for your robot/scene.

## Why drive `update()` manually instead of calling `solve()`?

`graph_core::TreeSolver::solve()` is a tight loop with no hooks:

```cpp
for (unsigned int iter = 0; iter < max_iter; iter++) {
  if (update(solution)) { solved_ = true; return true; }
  if (elapsed >= 0.98 * max_time) break;
}
return false;
```

Calling it directly would give no way to check `gh->is_canceling()` or call
`gh->publish_feedback()` mid-solve. `execute()` reimplements the same
stopping criterion (stop at the first improving `update()`, or at time/
iteration limits) itself, adding the cancellation check and periodic
feedback publish — see `parallel_multigoal_planner_action_server.cpp`.

One subtlety this handles explicitly: `MultigoalSolver::addGoal()` can
already find a direct start→goal connection (and set `solved_ = true`)
*before* the `update()` loop ever runs — in that case `solver->getSolution()`
is used to retrieve the path, since the loop body (which is what would
otherwise assign the local `solution` variable) never executes.

## Threading

See the README's [Concurrency model](../README.md#concurrency-model) section
for the callback-group/executor layout. In short: one detached
`std::thread` per accepted goal, each with its own `MultigoalSolver`
instance; three shared, stateless service-client wrappers; a
`MultiThreadedExecutor` spinning the node so blocking `future.wait_for()`
calls inside a goal's thread never stall other goals or the action server's
own goal/cancel callbacks.

Next: [5. Tuning and parameters](05-tuning-and-parameters.md).
