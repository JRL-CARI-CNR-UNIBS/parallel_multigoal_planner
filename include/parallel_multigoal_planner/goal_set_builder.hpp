#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <Eigen/Core>

#include <moveit_msgs/msg/constraints.hpp>

#include <ik_solver_msgs/srv/get_bound.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>

#include <cnr_logger/cnr_logger.h>
#include <graph_core/collision_checkers/collision_checker_base.h>

namespace parallel_multigoal_planner
{

/**
 * @brief Turns the (Cartesian XOR joint-space) elements of a
 * moveit_msgs/Constraints[] goal_constraints array into a flat set of
 * candidate joint-space goal configurations for MultigoalSolver.
 *
 * For a Cartesian element (position_constraints + orientation_constraints):
 *   1. calls ik_solver's get_task_redundant_ik_array to get IK solutions,
 *   2. keeps only the collision-free ones, using the planner's own
 *      collision checker (local MoveIt planning scene),
 *   3. adds the survivors to the candidate set.
 * For a joint-space element (joint_constraints): the configuration is added
 * directly, no IK/collision check performed (mirrt_star validates it itself
 * when the goal is registered with the solver).
 *
 * All ik_solver service calls are synchronous (blocking) from the caller's point of
 * view, but are implemented with async_send_request()+future.wait_for() so
 * they never spin an executor themselves: the caller must ensure the node is
 * being spun by an executor on a different thread while this class is used.
 */
class GoalSetBuilder
{
public:
  /**
   * @brief Constructor.
   * @param bounds_client Client of the ik_solver @c get_bounds service.
   * @param ik_client Client of the ik_solver @c get_ik_array service.
   * @param logger Logger.
   * @param service_timeout_s Timeout of each service call [s].
   */
  GoalSetBuilder(rclcpp::Client<ik_solver_msgs::srv::GetBound>::SharedPtr bounds_client,
                 rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client,
                 const cnr_logger::TraceLoggerPtr& logger, double service_timeout_s = 5.0);

  /**
   * @brief Query ik_solver's get_bounds service for the canonical joint set
   * (names + position bounds) that defines the search space ordering used
   * throughout the planner.
   */
  bool fetchJointModel(std::vector<std::string>& joint_names, Eigen::VectorXd& lower_bound,
                       Eigen::VectorXd& upper_bound, std::string& error);

  /**
   * @brief Build the candidate joint-space goal set for one goal_constraints
   * array, in `canonical_joint_names` order. IK solutions are filtered with
   * `checker`, which must use the same joint order.
   * @return false only on a hard error (e.g. malformed constraints, service
   * unavailable); an element that legitimately yields zero candidates (e.g.
   * unreachable Cartesian target) is not an error, it simply contributes
   * nothing to `goal_configurations`.
   */
  bool buildGoalConfigurations(const std::vector<moveit_msgs::msg::Constraints>& goal_constraints,
                               const std::vector<std::string>& canonical_joint_names,
                               const graph::core::CollisionCheckerPtr& checker,
                               std::vector<Eigen::VectorXd>& goal_configurations, std::string& error);

private:
  /**
   * @brief Compute the collision-free IK solutions of a Cartesian goal (position + orientation constraint).
   * @param c Goal constraints.
   * @param canonical_joint_names Joint order of the planner.
   * @param checker Collision checker used to filter the IK solutions.
   * @param[out] out_configs Collision-free IK solutions.
   * @param[out] error Error message.
   * @return False on a hard error.
   */
  bool solveCartesianGoal(const moveit_msgs::msg::Constraints& c, const std::vector<std::string>& canonical_joint_names,
                          const graph::core::CollisionCheckerPtr& checker, std::vector<Eigen::VectorXd>& out_configs,
                          std::string& error);

  /**
   * @brief Build a joint configuration from the joint constraints of a goal.
   * @param c Goal constraints.
   * @param canonical_joint_names Joint order of the planner.
   * @param[out] out_config Configuration.
   * @param[out] error Error message.
   * @return False if a joint of the planner is missing.
   */
  bool jointGoalFromConstraints(const moveit_msgs::msg::Constraints& c,
                                const std::vector<std::string>& canonical_joint_names, Eigen::VectorXd& out_config,
                                std::string& error);

  rclcpp::Client<ik_solver_msgs::srv::GetBound>::SharedPtr bounds_client_;
  rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client_;
  cnr_logger::TraceLoggerPtr logger_;
  double service_timeout_s_;
};

}  // namespace parallel_multigoal_planner
