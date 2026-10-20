# parallel_multigoal_planner API reference

This page is generated from the Doxygen comments in the source code. To browse the same
documentation as HTML, run `doxygen Doxyfile` in the repository folder and open
`doxygen/html/index.html`.

It covers the C++ classes and functions and the scripts. The ROS interface of the nodes
(actions, services, topics and parameters) is described in the [README](../README.md).

## Contents

- [`include/parallel_multigoal_planner/goal_set_builder.hpp`](#includeparallel_multigoal_plannergoal_set_builderhpp)
  - [`parallel_multigoal_planner::GoalSetBuilder`](#class-parallel_multigoal_plannergoalsetbuilder)
- [`include/parallel_multigoal_planner/joint_order_utils.hpp`](#includeparallel_multigoal_plannerjoint_order_utilshpp)
- [`include/parallel_multigoal_planner/parallel_multigoal_planner_action_server.hpp`](#includeparallel_multigoal_plannerparallel_multigoal_planner_action_serverhpp)
  - [`parallel_multigoal_planner::ParallelMultigoalPlannerActionServer`](#class-parallel_multigoal_plannerparallelmultigoalplanneractionserver)
- [`include/parallel_multigoal_planner/service_collision_checker.hpp`](#includeparallel_multigoal_plannerservice_collision_checkerhpp)
  - [`parallel_multigoal_planner::ServiceCollisionChecker`](#class-parallel_multigoal_plannerservicecollisionchecker)
- [Scripts](#scripts)

## include/parallel_multigoal_planner/goal_set_builder.hpp

### class `parallel_multigoal_planner::GoalSetBuilder`

Turns the (Cartesian XOR joint-space) elements of a moveit_msgs/Constraints[] goal_constraints array into a flat set of candidate joint-space goal configurations for MultigoalSolver.

For a Cartesian element (position_constraints + orientation_constraints):

1. calls ik_solver's get_task_redundant_ik_array to get IK solutions,
2. calls collision_checker_server's /collision_check to keep only the collision-free ones,
3. adds the survivors to the candidate set. For a joint-space element (joint_constraints): the configuration is added directly, no IK/collision check performed (mirrt_star validates it itself when the goal is registered with the solver).

All service calls are synchronous (blocking) from the caller's point of view, but are implemented with async_send_request()+future.wait_for() so they never spin an executor themselves: the caller must ensure the node is being spun by an executor on a different thread while this class is used.

#### Public functions

##### `GoalSetBuilder`

```cpp
GoalSetBuilder(rclcpp::Client< ik_solver_msgs::srv::GetBound >::SharedPtr bounds_client, rclcpp::Client< ik_solver_msgs::srv::GetIkArray >::SharedPtr ik_client, rclcpp::Client< ik_solver_msgs::srv::CollisionChecking >::SharedPtr collision_client, const cnr_logger::TraceLoggerPtr &logger, double service_timeout_s=5.0)
```

Constructor.

**Parameters:**

- `bounds_client`: Client of the ik_solver `get_bounds` service.
- `ik_client`: Client of the ik_solver `get_ik_array` service.
- `collision_client`: Client of the collision checking service.
- `logger`: Logger.
- `service_timeout_s`: Timeout of each service call [s].

##### `fetchJointModel`

```cpp
bool fetchJointModel(std::vector< std::string > &joint_names, Eigen::VectorXd &lower_bound, Eigen::VectorXd &upper_bound, std::string &error)
```

Query ik_solver's get_bounds service for the canonical joint set (names + position bounds) that defines the search space ordering used throughout the planner.

##### `buildGoalConfigurations`

```cpp
bool buildGoalConfigurations(const std::vector< moveit_msgs::msg::Constraints > &goal_constraints, const std::vector< std::string > &canonical_joint_names, std::vector< Eigen::VectorXd > &goal_configurations, std::string &error)
```

Build the candidate joint-space goal set for one goal_constraints array, in `canonical_joint_names` order.

**Returns:** false only on a hard error (e.g. malformed constraints, service unavailable); an element that legitimately yields zero candidates (e.g. unreachable Cartesian target) is not an error, it simply contributes nothing to `goal_configurations`.

## include/parallel_multigoal_planner/joint_order_utils.hpp

### Free functions and types

#### `reorderByJointName`

```cpp
bool reorderByJointName(const std::vector< std::string > &names, const std::vector< double > &values, const std::vector< std::string > &canonical_order, Eigen::VectorXd &out, std::string &error)
```

Reorder a (names, values) pair into the given canonical joint order.

**Returns:** false (with `error` populated) if any canonical joint is missing from `names`.

## include/parallel_multigoal_planner/parallel_multigoal_planner_action_server.hpp

### class `parallel_multigoal_planner::ParallelMultigoalPlannerActionServer`

Action server for parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan.

On each accepted goal (executed on its own detached thread, see handle_accepted()):

1. Resolves the canonical joint set from ik_solver's get_bounds (cached after the first call).
2. Turns every element of motion_plan_request.goal_constraints into zero or more candidate joint-space goal configurations, via `GoalSetBuilder` (IK + collision-check ROS services for Cartesian elements, straight pass-through for joint-space elements).
3. Feeds the candidate set to a fresh graph::mirrt_star::MultigoalSolver and drives its update() loop directly (rather than the blocking TreeSolver::solve()) so that goal cancellation and feedback publishing both work.
4. Converts the resulting graph::core::Path into a MotionPlanResponse.

#### Public types

##### `Action`

```cpp
using parallel_multigoal_planner::ParallelMultigoalPlannerActionServer::Action = parallel_multigoal_planner_msgs::action::ParallelMultigoalPlan
```

##### `GoalHandle`

```cpp
using parallel_multigoal_planner::ParallelMultigoalPlannerActionServer::GoalHandle = rclcpp_action::ServerGoalHandle<Action>
```

#### Public functions

##### `ParallelMultigoalPlannerActionServer`

```cpp
ParallelMultigoalPlannerActionServer(const rclcpp::Node::SharedPtr &node)
```

Declare the parameters, create the service clients and start the action server.

Parameters: `ik_solver_ns` ("ik_solver"), `collision_check_service` ("/collision_check"), `service_timeout_s` (5.0), `default_allowed_planning_time` (5.0), `direct_connect_time` (0.2), `max_iterations` (1000000), `feedback_period_iterations` (200), `waypoint_dt_s` (0.5), `checker_min_distance` (0.05), `mirrt_star_param_ns` ("/parallel_multigoal_planner/mirrt_star"), `logger_config_file`. See `docs/05-tuning-and-parameters.md`.

**Parameters:**

- `node`: Node used for parameters, clients and the action server; it must be spun by a multi-threaded executor.

## include/parallel_multigoal_planner/service_collision_checker.hpp

### class `parallel_multigoal_planner::ServiceCollisionChecker`

Inherits from `graph::core::CollisionCheckerBase`.

graph_core CollisionCheckerBase implementation that delegates every collision query to the /collision_check ROS 2 service exposed by collision_checker_server (ik_solver_msgs/srv/CollisionChecking), instead of maintaining a local MoveIt planning scene.

`check()` sends a single-configuration request; `checkConnection()` batches all the sampled points of a tree edge into a single service call, since this is the hot path called by mirrt_star while extending the tree.

#### Public types

##### `ClientT`

```cpp
using parallel_multigoal_planner::ServiceCollisionChecker::ClientT = rclcpp::Client<ik_solver_msgs::srv::CollisionChecking>
```

Client type of the collision checking service.

#### Public functions

##### `ServiceCollisionChecker`

```cpp
ServiceCollisionChecker(const ClientT::SharedPtr &client, const std::vector< std::string > &joint_names, const cnr_logger::TraceLoggerPtr &logger, const double &min_distance=0.05, const double &service_timeout_s=5.0)
```

Constructor.

**Parameters:**

- `client`: Client of the collision checking service.
- `joint_names`: Joint names, in the order of the configurations passed to `check()`.
- `logger`: Logger.
- `min_distance`: Distance between the points checked along a connection.
- `service_timeout_s`: Timeout of each service call [s].

##### `check`

```cpp
bool check(const Eigen::VectorXd &configuration) override
```

Check one configuration with one service call.

**Parameters:**

- `configuration`: Configuration.

**Returns:** True if collision-free; false if in collision or if the service fails.

##### `checkConnection`

```cpp
bool checkConnection(const Eigen::VectorXd &configuration1, const Eigen::VectorXd &configuration2) override
```

Check the segment between two configurations, sampled every `min_distance`, with one batched service call.

**Parameters:**

- `configuration1`: First configuration.
- `configuration2`: Second configuration.

**Returns:** True if all the sampled points are collision-free.

##### `clone`

```cpp
graph::core::CollisionCheckerPtr clone() override
```

Copy that shares the same service client.

## Scripts

### `scripts/test_display_planned_path.py`

```
Smoke-test / demo node for parallel_multigoal_planner.

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
```
