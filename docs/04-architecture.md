# 4. Architecture

## Sequence, end to end

```mermaid
sequenceDiagram
    participant Client
    participant Server as ParallelMultigoalPlannerActionServer
    participant Bounds as ik_solver:~get_bounds
    participant IK as ik_solver:~get_task_redundant_ik_array
    participant Scene as move_group:/get_planning_scene
    participant Solver as MultigoalSolver (mirrt_star)

    Client->>Server: send_goal(ParallelMultigoalPlan)
    Server->>Server: handle_goal() — reject if goal_constraints or group_name empty
    Server-->>Client: ACCEPT_AND_EXECUTE
    Server->>Server: handle_accepted() spawns detached thread -> execute()

    Server->>Server: joint_names = active joints of group_name (robot model)
    Note over Server,Bounds: once per node lifetime (cached)
    Server->>Bounds: GetBound.Request()
    Bounds-->>Server: joint names, lower/upper bounds (reordered to joint_names)

    Server->>Scene: GetPlanningScene.Request()
    Scene-->>Server: current scene
    Server->>Server: checker = ParallelMoveitCollisionChecker(scene, group_name)

    Server->>Server: start_q = reorder(start_state, joint_names)

    loop for each element c of goal_constraints
        alt c.joint_constraints set
            Server->>Server: candidate = reorder(c.joint_constraints, joint_names)
        else c.position/orientation_constraints set
            Server->>IK: GetIkArray.Request(target pose)
            IK-->>Server: IkSolution[] (many joint configs)
            Server->>Server: keep configs with checker->check(q) == true
        end
    end

    Server->>Solver: new MultigoalSolver(metrics, checker, sampler, ...)
    Server->>Solver: config(), addStart(start), addGoal(...) per candidate
    Server->>Solver: finalizeProblem()

    loop until solved, cancelled, or timeout
        Server->>Server: is_canceling()?
        Server->>Solver: update(solution)
        Solver->>Solver: edge checks with the local checker (in-process)
        Server-->>Client: feedback(status, progress)  [every feedback_period_iterations]
    end

    Server->>Server: path -> MotionPlanResponse (JointTrajectory)
    Server-->>Client: result(motion_plan_response)
```

## Class responsibilities

| Class | File | Responsibility |
|---|---|---|
| `ParallelMultigoalPlannerActionServer` | `parallel_multigoal_planner_action_server.{hpp,cpp}` | Owns the action server, the robot model and the three service clients; for each goal fetches the planning scene, creates the collision checker, runs `execute()` and converts `Path` → `MotionPlanResponse`. |
| `GoalSetBuilder` | `goal_set_builder.{hpp,cpp}` | Turns `goal_constraints[]` into a flat `std::vector<Eigen::VectorXd>` of candidate joint-space goals. Owns the `~get_bounds` / `~get_task_redundant_ik_array` request/response plumbing, and filters IK solutions with the collision checker it is given. |
| `graph::collision_check::ParallelMoveitCollisionChecker` | `moveit_collision_checker` package | In-process collision checker on a local copy of the planning scene. One instance per goal, shared by `GoalSetBuilder` (IK filtering) and `MultigoalSolver` (tree extension). |
| `reorderByJointName()` | `joint_order_utils.hpp` | The one place that maps a `(names, values)` pair (from a `JointState`, a `JointConstraint[]`, or an IK response) onto the canonical joint order. Every `Eigen::VectorXd` in the planner goes through this. |

## Collision checking

All collision checks run in-process, on a `planning_scene::PlanningScene`
built from `/get_planning_scene` when the request starts. The same checker is
used to filter IK solutions (once per Cartesian goal element) and by
`MultigoalSolver` while it extends its trees (the hot path: thousands of
calls per goal). `checkConnection()` checks points `checker_resolution`
apart along each edge, split across `checker_threads` threads.

Each request gets its own scene and checker, so concurrent requests do not
share collision state, and a request does not see scene changes made while it
is planning.

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
