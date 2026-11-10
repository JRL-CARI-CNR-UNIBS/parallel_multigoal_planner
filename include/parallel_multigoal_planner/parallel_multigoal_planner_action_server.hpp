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

#include <sensor_msgs/msg/joint_state.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>

#if __has_include(<moveit/planning_scene/planning_scene.hpp>)
#include <moveit/planning_scene/planning_scene.hpp>  // MoveIt >= Jazzy
#else
#include <moveit/planning_scene/planning_scene.h>  // MoveIt Humble
#endif
#if __has_include(<moveit/robot_model_loader/robot_model_loader.hpp>)
#include <moveit/robot_model_loader/robot_model_loader.hpp>
#else
#include <moveit/robot_model_loader/robot_model_loader.h>
#endif

#include <mirrt_star/solvers/mirrt_star.h>

#include <parallel_multigoal_planner/goal_set_builder.hpp>

namespace parallel_multigoal_planner
{

/**
 * @brief Action server for parallel_multigoal_planner_msgs/action/ParallelMultigoalPlan.
 *
 * On each accepted goal (executed on its own detached thread, see
 * handle_accepted()):
 *  1. Takes the joint set of motion_plan_request.group_name from the MoveIt
 *     robot model, and the joint bounds from ik_solver's get_bounds (cached
 *     after the first call).
 *  2. Fetches the current planning scene from move_group (@c get_planning_scene)
 *     and builds a local graph::collision_check::ParallelMoveitCollisionChecker
 *     on it: all collision checks are done in-process, no collision service.
 *  3. Turns every element of motion_plan_request.goal_constraints into zero
 *     or more candidate joint-space goal configurations, via GoalSetBuilder
 *     (IK service + local collision check for Cartesian elements, straight
 *     pass-through for joint-space elements).
 *  4. Feeds the candidate set to a fresh graph::mirrt_star::MultigoalSolver
 *     and drives its update() loop directly (rather than the blocking
 *     TreeSolver::solve()) so that goal cancellation and feedback publishing
 *     both work.
 *  5. Converts the resulting graph::core::Path into a MotionPlanResponse.
 */
class ParallelMultigoalPlannerActionServer
{
public:
  using Action = parallel_multigoal_planner_msgs::action::ParallelMultigoalPlan;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;

  /// Prefix of the node parameters passed to the mirrt* solver.
  static constexpr const char* kMirrtStarPrefix = "mirrt_star";

  /**
   * @brief Declare the parameters, load the robot model, create the service clients and start the action server.
   *
   * Parameters: @c ik_solver_ns ("ik_solver"), @c planning_scene_service ("/get_planning_scene"),
   * @c service_timeout_s (5.0), @c default_allowed_planning_time (5.0), @c direct_connect_time (0.2),
   * @c max_iterations (1000000), @c feedback_period_iterations (200), @c waypoint_dt_s (0.5),
   * @c checker_resolution (0.02), @c checker_threads (4).
   * The parameters of the mirrt* solver are the node parameters under @c mirrt_star. (e.g.
   * @c mirrt_star.max_distance); they are read at every request.
   * See docs/05-tuning-and-parameters.md.
   * @param node Node used for parameters, clients and the action server. It must not be added to an
   * executor yet: the robot model is loaded in the constructor.
   * @throw std::runtime_error if the robot model cannot be loaded.
   */
  explicit ParallelMultigoalPlannerActionServer(const rclcpp::Node::SharedPtr& node);

private:
  /**
   * @brief Accept every goal that has a group name and at least one goal constraint.
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
  bool ensureIkBounds(std::string& error);

  /**
   * @brief Get the active joints of a planning group, with their bounds from ik_solver.
   * @param group_name MoveIt planning group.
   * @param[out] joint_names Active joints of the group (order used by the planner and the collision checker).
   * @param[out] lower_bound Lower bounds, in @p joint_names order.
   * @param[out] upper_bound Upper bounds, in @p joint_names order.
   * @param[out] error Error message.
   * @return False if the group does not exist or ik_solver does not provide bounds for one of its joints.
   */
  bool resolveGroup(const std::string& group_name, std::vector<std::string>& joint_names,
                    Eigen::VectorXd& lower_bound, Eigen::VectorXd& upper_bound, std::string& error);

  /**
   * @brief Build a planning scene from the current move_group scene.
   * @param[out] scene New planning scene.
   * @param[out] error Error message.
   * @return False if the planning scene service is not available or the scene cannot be applied.
   */
  bool fetchPlanningScene(planning_scene::PlanningScenePtr& scene, std::string& error);

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
   * @param joint_names Joint order of the path.
   * @param start_q Start configuration.
   * @param solution Solution path.
   * @param planning_time Planning time [s].
   * @return Response with error code SUCCESS.
   */
  moveit_msgs::msg::MotionPlanResponse pathToMotionPlanResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                                 const std::vector<std::string>& joint_names,
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
  rclcpp::CallbackGroup::SharedPtr cb_group_scene_;

  rclcpp::Client<ik_solver_msgs::srv::GetBound>::SharedPtr bounds_client_;
  rclcpp::Client<ik_solver_msgs::srv::GetIkArray>::SharedPtr ik_client_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr scene_client_;

  std::unique_ptr<GoalSetBuilder> goal_builder_;

  /// Logger passed to graph_core / mirrt* (child "graph_core" of the node logger).
  graph::core::LoggerPtr logger_;

  // Robot model, loaded once from robot_description.
  std::shared_ptr<robot_model_loader::RobotModelLoader> robot_model_loader_;
  moveit::core::RobotModelConstPtr robot_model_;

  // Joint bounds from ik_solver (its own joint order), lazily fetched on first use.
  std::mutex ik_bounds_mtx_;
  bool ik_bounds_ready_ = false;
  std::vector<std::string> ik_joint_names_;
  std::vector<double> ik_lower_bound_;
  std::vector<double> ik_upper_bound_;

  // Parameters
  double service_timeout_s_;
  double default_allowed_planning_time_;
  double direct_connect_time_;
  unsigned int max_iterations_;
  unsigned int feedback_period_iterations_;
  double waypoint_dt_s_;
  double checker_resolution_;
  int checker_threads_;
};

}  // namespace parallel_multigoal_planner
