#include <parallel_multigoal_planner/goal_set_builder.hpp>
#include <parallel_multigoal_planner/joint_order_utils.hpp>

#include <ik_solver_msgs/msg/ik_target.hpp>
#include <ik_solver_msgs/msg/configuration.hpp>

namespace parallel_multigoal_planner
{

GoalSetBuilder::GoalSetBuilder(rclcpp::Client<ik_solver_msgs::srv::GetBound>::SharedPtr bounds_client,
                               rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client,
                               rclcpp::Client<ik_solver_msgs::srv::CollisionChecking>::SharedPtr collision_client,
                               const cnr_logger::TraceLoggerPtr& logger, double service_timeout_s)
  : bounds_client_(bounds_client)
  , ik_client_(ik_client)
  , collision_client_(collision_client)
  , logger_(logger)
  , service_timeout_s_(service_timeout_s)
{
}

bool GoalSetBuilder::fetchJointModel(std::vector<std::string>& joint_names, Eigen::VectorXd& lower_bound,
                                     Eigen::VectorXd& upper_bound, std::string& error)
{
  if (!bounds_client_->wait_for_service(std::chrono::duration<double>(service_timeout_s_)))
  {
    error = "get_bounds service not available";
    return false;
  }

  auto request = std::make_shared<ik_solver_msgs::srv::GetBound::Request>();
  auto future = bounds_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) != std::future_status::ready)
  {
    error = "Timeout waiting for get_bounds response";
    return false;
  }

  auto response = future.get();
  if (response->boundaries.empty())
  {
    error = "get_bounds returned an empty joint set";
    return false;
  }

  joint_names.clear();
  joint_names.reserve(response->boundaries.size());
  lower_bound.resize(static_cast<Eigen::Index>(response->boundaries.size()));
  upper_bound.resize(static_cast<Eigen::Index>(response->boundaries.size()));

  for (size_t i = 0; i < response->boundaries.size(); ++i)
  {
    const auto& b = response->boundaries[i];
    joint_names.push_back(b.joint_name);
    lower_bound(static_cast<Eigen::Index>(i)) = b.lower_bound;
    upper_bound(static_cast<Eigen::Index>(i)) = b.upper_bound;
  }

  return true;
}

bool GoalSetBuilder::jointGoalFromConstraints(const moveit_msgs::msg::Constraints& c,
                                              const std::vector<std::string>& canonical_joint_names,
                                              Eigen::VectorXd& out_config, std::string& error)
{
  std::vector<std::string> names;
  std::vector<double> values;
  names.reserve(c.joint_constraints.size());
  values.reserve(c.joint_constraints.size());
  for (const auto& jc : c.joint_constraints)
  {
    names.push_back(jc.joint_name);
    values.push_back(jc.position);
  }

  return reorderByJointName(names, values, canonical_joint_names, out_config, error);
}

std::vector<bool> GoalSetBuilder::filterCollisionFree(const std::vector<Eigen::VectorXd>& configs,
                                                      const std::vector<std::string>& joint_names)
{
  std::vector<bool> feasible(configs.size(), false);
  if (configs.empty())
    return feasible;

  if (!collision_client_->wait_for_service(std::chrono::duration<double>(service_timeout_s_)))
  {
    CNR_ERROR(logger_, "/collision_check service is not available.");
    return feasible;
  }

  auto request = std::make_shared<ik_solver_msgs::srv::CollisionChecking::Request>();
  request->joint_names = joint_names;
  request->detailed = false;
  request->solutions.reserve(configs.size());
  for (const auto& q : configs)
  {
    ik_solver_msgs::msg::Configuration cfg;
    cfg.configuration.assign(q.data(), q.data() + q.size());
    request->solutions.push_back(std::move(cfg));
  }

  auto future = collision_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) != std::future_status::ready)
  {
    CNR_ERROR(logger_, "Timeout waiting for /collision_check response.");
    return feasible;
  }

  auto response = future.get();
  if (response->result.size() != configs.size())
  {
    CNR_ERROR(logger_, "/collision_check returned " << response->result.size() << " results for " << configs.size()
                                                      << " requested configurations (joint_names mismatch?).");
    return feasible;
  }

  for (size_t i = 0; i < response->result.size(); ++i)
    feasible[i] = (response->result[i].feasibility > 0.5) && !response->result[i].out_of_bound;

  return feasible;
}

bool GoalSetBuilder::solveCartesianGoal(const moveit_msgs::msg::Constraints& c,
                                        const std::vector<std::string>& canonical_joint_names,
                                        std::vector<Eigen::VectorXd>& out_configs, std::string& error)
{
  const auto& pc = c.position_constraints.front();
  const auto& oc = c.orientation_constraints.front();

  if (pc.constraint_region.primitive_poses.empty())
  {
    error = "position_constraints[0].constraint_region.primitive_poses is empty";
    return false;
  }

  if (!ik_client_->wait_for_service(std::chrono::duration<double>(service_timeout_s_)))
  {
    error = "get_task_redundant_ik_array service is not available";
    return false;
  }

  ik_solver_msgs::msg::IkTarget target;
  target.pose.header = pc.header;
  target.pose.pose.position = pc.constraint_region.primitive_poses.front().position;
  target.pose.pose.orientation = oc.orientation;

  auto request = std::make_shared<ik_solver_msgs::srv::GetIkArray::Request>();
  request->targets.push_back(target);
  request->seed_joint_names = canonical_joint_names;

  auto future = ik_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(service_timeout_s_)) != std::future_status::ready)
  {
    error = "Timeout waiting for get_task_redundant_ik_array response";
    return false;
  }

  auto response = future.get();

  std::vector<Eigen::VectorXd> ik_configs;
  for (const auto& sol : response->solutions)
  {
    for (const auto& cfg : sol.configurations)
    {
      Eigen::VectorXd q;
      std::string reorder_err;
      if (!reorderByJointName(response->joint_names, cfg.configuration, canonical_joint_names, q, reorder_err))
      {
        CNR_WARN(logger_, "Skipping an IK solution: " << reorder_err);
        continue;
      }
      ik_configs.push_back(std::move(q));
    }
  }

  if (ik_configs.empty())
  {
    CNR_WARN(logger_, "get_task_redundant_ik_array found no IK solution for this Cartesian goal.");
    return true;  // not an error: this goal element simply contributes nothing
  }

  std::vector<bool> feasible = filterCollisionFree(ik_configs, canonical_joint_names);
  for (size_t i = 0; i < ik_configs.size(); ++i)
    if (feasible[i])
      out_configs.push_back(ik_configs[i]);

  return true;
}

bool GoalSetBuilder::buildGoalConfigurations(const std::vector<moveit_msgs::msg::Constraints>& goal_constraints,
                                             const std::vector<std::string>& canonical_joint_names,
                                             std::vector<Eigen::VectorXd>& goal_configurations, std::string& error)
{
  goal_configurations.clear();

  for (size_t i = 0; i < goal_constraints.size(); ++i)
  {
    const auto& c = goal_constraints[i];
    const bool has_joint_goal = !c.joint_constraints.empty();
    const bool has_pose_goal = !c.position_constraints.empty() && !c.orientation_constraints.empty();

    if (has_joint_goal)
    {
      Eigen::VectorXd q;
      std::string local_err;
      if (!jointGoalFromConstraints(c, canonical_joint_names, q, local_err))
      {
        error = "goal_constraints[" + std::to_string(i) + "]: " + local_err;
        return false;
      }
      goal_configurations.push_back(std::move(q));
    }
    else if (has_pose_goal)
    {
      std::string local_err;
      if (!solveCartesianGoal(c, canonical_joint_names, goal_configurations, local_err))
      {
        error = "goal_constraints[" + std::to_string(i) + "]: " + local_err;
        return false;
      }
    }
    else
    {
      error = "goal_constraints[" + std::to_string(i) +
             "]: must specify either joint_constraints, or both position_constraints and orientation_constraints";
      return false;
    }
  }

  return true;
}

}  // namespace parallel_multigoal_planner
