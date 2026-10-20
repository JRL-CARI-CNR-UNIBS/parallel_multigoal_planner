# 3. Sending goals from C++ (rclcpp_action)

A minimal C++ client, mirroring the Python one from
[tutorial 2](02-sending-goals-python.md): resolve the canonical joint names
via `~get_bounds`, build one Cartesian and one joint-space alternative, send
the goal, print feedback, print the result.

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <parallel_multigoal_planner_msgs/action/parallel_multigoal_plan.hpp>
#include <ik_solver_msgs/srv/get_bound.hpp>

#include <moveit_msgs/msg/move_it_error_codes.hpp>

using ParallelMultigoalPlan = parallel_multigoal_planner_msgs::action::ParallelMultigoalPlan;
using GoalHandle = rclcpp_action::ClientGoalHandle<ParallelMultigoalPlan>;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("parallel_multigoal_planner_cpp_client");

  // 1. Resolve the canonical joint set, exactly like the server does.
  auto bounds_client = node->create_client<ik_solver_msgs::srv::GetBound>("/solver1/get_bounds");
  bounds_client->wait_for_service();
  auto bounds_future = bounds_client->async_send_request(std::make_shared<ik_solver_msgs::srv::GetBound::Request>());
  rclcpp::spin_until_future_complete(node, bounds_future);

  std::vector<std::string> joint_names;
  for (const auto& b : bounds_future.get()->boundaries)
    joint_names.push_back(b.joint_name);

  // 2. Build the goal: one Cartesian alternative + one joint-space alternative.
  auto action_client = rclcpp_action::create_client<ParallelMultigoalPlan>(node, "parallel_multigoal_plan");
  action_client->wait_for_action_server();

  ParallelMultigoalPlan::Goal goal;
  goal.motion_plan_request.start_state.joint_state.name = joint_names;
  goal.motion_plan_request.start_state.joint_state.position = std::vector<double>(joint_names.size(), 0.0);
  goal.motion_plan_request.allowed_planning_time = 8.0;
  goal.verbose = true;

  moveit_msgs::msg::Constraints cartesian_alt;
  cartesian_alt.position_constraints.emplace_back();
  auto& pc = cartesian_alt.position_constraints.front();
  pc.header.frame_id = "base_link";  // must be ik_solver's base frame
  geometry_msgs::msg::Pose target_pose;
  target_pose.position.x = 0.4;
  target_pose.position.y = 0.1;
  target_pose.position.z = 0.5;
  target_pose.orientation.w = 1.0;
  pc.constraint_region.primitive_poses.push_back(target_pose);
  cartesian_alt.orientation_constraints.emplace_back();
  cartesian_alt.orientation_constraints.front().header = pc.header;
  cartesian_alt.orientation_constraints.front().orientation = target_pose.orientation;

  moveit_msgs::msg::Constraints joint_alt;
  for (const auto& name : joint_names)
  {
    moveit_msgs::msg::JointConstraint jc;
    jc.joint_name = name;
    jc.position = 0.2;
    jc.weight = 1.0;
    joint_alt.joint_constraints.push_back(jc);
  }

  goal.motion_plan_request.goal_constraints = { cartesian_alt, joint_alt };

  // 3. Send it, with a feedback callback, and wait for the result.
  rclcpp_action::Client<ParallelMultigoalPlan>::SendGoalOptions options;
  options.feedback_callback =
      [node](GoalHandle::SharedPtr, const std::shared_ptr<const ParallelMultigoalPlan::Feedback> fb)
  {
    RCLCPP_INFO(node->get_logger(), "[%.2f] %s", fb->progress, fb->status.c_str());
  };

  auto goal_handle_future = action_client->async_send_goal(goal, options);
  if (rclcpp::spin_until_future_complete(node, goal_handle_future) != rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Failed to send goal");
    return 1;
  }

  auto goal_handle = goal_handle_future.get();
  if (!goal_handle)
  {
    RCLCPP_ERROR(node->get_logger(), "Goal was rejected");
    return 1;
  }

  auto result_future = action_client->async_get_result(goal_handle);
  rclcpp::spin_until_future_complete(node, result_future);
  auto wrapped_result = result_future.get();

  const auto& response = wrapped_result.result->motion_plan_response;
  if (response.error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed, error_code=%d", response.error_code.val);
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Got a trajectory with %zu points.",
              response.trajectory.joint_trajectory.points.size());

  rclcpp::shutdown();
  return 0;
}
```

Add to a `CMakeLists.txt` for a scratch client executable like this one:

```cmake
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(parallel_multigoal_planner_msgs REQUIRED)
find_package(ik_solver_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(my_client my_client.cpp)
ament_target_dependencies(my_client
  rclcpp rclcpp_action parallel_multigoal_planner_msgs ik_solver_msgs moveit_msgs)
```

### Cancelling

```cpp
auto cancel_future = action_client->async_cancel_goal(goal_handle);
rclcpp::spin_until_future_complete(node, cancel_future);
```

`wrapped_result.code` will be `rclcpp_action::ResultCode::CANCELED` and
`response.error_code.val` will be `moveit_msgs::msg::MoveItErrorCodes::PREEMPTED`.

Next: [4. Architecture](04-architecture.md).
