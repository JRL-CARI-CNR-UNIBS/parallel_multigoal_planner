#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <parallel_multigoal_planner_msgs/action/parallel_multigoal_plan.hpp>

#include <ik_solver_msgs/srv/get_bound.hpp>
#include <ik_solver_msgs/srv/get_ik_array.hpp>
#include <ik_solver_msgs/srv/collision_checking.hpp>

#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>

#include <mirrt_star/solvers/mirrt_star.h>

#include <parallel_multigoal_planner/goal_set_builder.hpp>
#include <parallel_multigoal_planner/service_collision_checker.hpp>

namespace parallel_multigoal_planner
{

/**
 * @brief Action server for parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan.
 *
 * On each accepted goal (executed on its own detached thread, see
 * handle_accepted()):
 *  1. Resolves the canonical joint set from ik_solver's get_bounds (cached
 *     after the first call).
 *  2. Turns every element of motion_plan_request.goal_constraints into zero
 *     or more candidate joint-space goal configurations, via GoalSetBuilder
 *     (IK + collision-check ROS services for Cartesian elements, straight
 *     pass-through for joint-space elements).
 *  3. Feeds the candidate set to a fresh graph::mirrt_star::MultigoalSolver
 *     and drives its update() loop directly (rather than the blocking
 *     TreeSolver::solve()) so that goal cancellation and feedback publishing
 *     both work.
 *  4. Converts the resulting graph::core::Path into a MotionPlanResponse.
 */
class ParallelMultigoalPlannerActionServer
{
public:
  using Action = parallel_multigoal_planner_msgs::action::ParallelMultigoalPlan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  /**
   * @brief Declare the parameters, create the service clients and start the action server.
   *
   * Parameters: @c ik_solver_ns ("ik_solver"), @c collision_check_service ("/collision_check"),
   * @c service_timeout_s (5.0), @c default_allowed_planning_time (5.0), @c direct_connect_time (0.2),
   * @c max_iterations (1000000), @c feedback_period_iterations (200), @c waypoint_dt_s (0.5),
   * @c checker_min_distance (0.05), @c mirrt_star_param_ns ("/parallel_multigoal_planner/mirrt_star"),
   * @c logger_config_file. See docs/05-tuning-and-parameters.md.
   * @param node Node used for parameters, clients and the action server; it must be spun by a
   * multi-threaded executor.
   */
  explicit ParallelMultigoalPlannerActionServer(const rclcpp::Node::SharedPtr& node);

private:
  /**
   * @brief Accept every goal that has at least one goal constraint.
   */
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                          std::shared_ptr<const Action::Goal> goal);
  /**
   * @brief Accept cancellation; it is checked at every planner iteration.
   */
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> gh);
  /**
   * @brief Run execute() in a detached thread.
   */
  void handle_accepted(const std::shared_ptr<GoalHandle> gh);

  /**
   * @brief Plan for one goal: build the candidate goal set, run MI-RRT*, and return the path.
   */
  void execute(const std::shared_ptr<GoalHandle> gh);

  /**
   * @brief Read joint names and bounds from ik_solver the first time they are needed.
   * @param[out] error Error message.
   * @return False if the ik_solver bounds service is not available.
   */
  bool ensureJointModel(std::string& error);

  /**
   * @brief Extract the joint positions of a JointState in a given joint order.
   * @param js Joint state.
   * @param joint_order Wanted order.
   * @param[out] ok False if a joint is missing.
   * @param[out] error Error message.
   * @return Joint positions.
   */
  static Eigen::VectorXd vectorFromJointState(const sensor_msgs::msg::JointState& js,
                                              const std::vector<std::string>& joint_order, bool& ok,
                                              std::string& error);

  /**
   * @brief Convert the solution path to a MotionPlanResponse (waypoints spaced by @c waypoint_dt_s).
   * @param req Original request.
   * @param start_q Start configuration.
   * @param solution Solution path.
   * @param planning_time Planning time [s].
   * @return Response with error code SUCCESS.
   */
  moveit_msgs::msg::MotionPlanResponse pathToMotionPlanResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                                 const Eigen::VectorXd& start_q,
                                                                 const graph::core::PathPtr& solution,
                                                                 double planning_time) const;

  /**
   * @brief Build a MotionPlanResponse with an error code and no trajectory.
   * @param req Original request.
   * @param error_code moveit_msgs::msg::MoveItErrorCodes value.
   * @return Response.
   */
  static moveit_msgs::msg::MotionPlanResponse errorResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                             int32_t error_code);

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Server<Action>::SharedPtr server_;

  // Dedicated callback groups so service responses are never stuck behind
  // the action server's own goal/cancel callbacks (or each other).
  rclcpp::CallbackGroup::SharedPtr cb_group_bounds_;
  rclcpp::CallbackGroup::SharedPtr cb_group_ik_;
  rclcpp::CallbackGroup::SharedPtr cb_group_collision_;

  rclcpp::Client<ik_solver_msgs::srv::GetBound>::SharedPtr bounds_client_;
  rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client_;
  rclcpp::Client<ik_solver_msgs::srv::CollisionChecking>::SharedPtr collision_client_;

  std::unique_ptr<GoalSetBuilder> goal_builder_;
  std::shared_ptr<ServiceCollisionChecker> checker_;

  cnr_logger::TraceLoggerPtr logger_;

  // Canonical joint set (order used for every Eigen::VectorXd in the
  // planner), lazily resolved from ik_solver on first use.
  std::mutex joint_model_mtx_;
  bool joint_model_ready_ = false;
  std::vector<std::string> joint_names_;
  Eigen::VectorXd lower_bound_;
  Eigen::VectorXd upper_bound_;

  // Parameters
  double service_timeout_s_;
  double default_allowed_planning_time_;
  double direct_connect_time_;
  unsigned int max_iterations_;
  unsigned int feedback_period_iterations_;
  double waypoint_dt_s_;
  double checker_min_distance_;
  std::string mirrt_param_ns_;
};

}  // namespace parallel_multigoal_planner
