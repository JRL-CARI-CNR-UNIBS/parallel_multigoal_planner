#include <chrono>
#include <thread>
#include <unordered_map>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <parallel_multigoal_planner/parallel_multigoal_planner_action_server.hpp>
#include <parallel_multigoal_planner/joint_order_utils.hpp>

#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/metrics/euclidean_metrics.h>
#include <graph_core/metrics/goal_cost_function_base.h>
#include <graph_core/samplers/uniform_sampler.h>

#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace parallel_multigoal_planner
{

ParallelMultigoalPlannerActionServer::ParallelMultigoalPlannerActionServer(const rclcpp::Node::SharedPtr& node)
  : node_(node)
{
  const std::string ik_solver_ns = node_->declare_parameter<std::string>("ik_solver_ns", "ik_solver");
  const std::string collision_check_service =
      node_->declare_parameter<std::string>("collision_check_service", "/collision_check");

  service_timeout_s_ = node_->declare_parameter<double>("service_timeout_s", 5.0);
  default_allowed_planning_time_ = node_->declare_parameter<double>("default_allowed_planning_time", 5.0);
  direct_connect_time_ = node_->declare_parameter<double>("direct_connect_time", 0.2);
  max_iterations_ = static_cast<unsigned int>(node_->declare_parameter<int>("max_iterations", 1000000));
  feedback_period_iterations_ =
      static_cast<unsigned int>(node_->declare_parameter<int>("feedback_period_iterations", 200));
  waypoint_dt_s_ = node_->declare_parameter<double>("waypoint_dt_s", 0.5);
  checker_min_distance_ = node_->declare_parameter<double>("checker_min_distance", 0.05);
  mirrt_param_ns_ = node_->declare_parameter<std::string>("mirrt_star_param_ns", "/parallel_multigoal_planner/mirrt_star");

  std::string default_logger_config;
  try
  {
    default_logger_config =
        ament_index_cpp::get_package_share_directory("parallel_multigoal_planner") + "/config/logger_param.yaml";
  }
  catch (const std::exception&)
  {
    default_logger_config = "";
  }
  const std::string logger_config = node_->declare_parameter<std::string>("logger_config_file", default_logger_config);
  logger_ = std::make_shared<cnr_logger::TraceLogger>("parallel_multigoal_planner", logger_config);

  cb_group_bounds_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_ik_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_collision_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  bounds_client_ = node_->create_client<ik_solver_msgs::srv::GetBound>(
      "/" + ik_solver_ns + "/get_bounds", rclcpp::QoS(10), cb_group_bounds_);
  ik_client_ = node_->create_client<ik_solver_msgs::srv::GetIkArray>(
      "/" + ik_solver_ns + "/get_task_redundant_ik_array", rclcpp::QoS(10), cb_group_ik_);
  collision_client_ = node_->create_client<ik_solver_msgs::srv::CollisionChecking>(
      collision_check_service, rclcpp::QoS(10), cb_group_collision_);

  goal_builder_ =
      std::make_unique<GoalSetBuilder>(bounds_client_, ik_client_, collision_client_, logger_, service_timeout_s_);

  using std::placeholders::_1;
  using std::placeholders::_2;

  server_ = rclcpp_action::create_server<Action>(node_, "parallel_multigoal_plan",
                                                 std::bind(&ParallelMultigoalPlannerActionServer::handle_goal, this, _1, _2),
                                                 std::bind(&ParallelMultigoalPlannerActionServer::handle_cancel, this, _1),
                                                 std::bind(&ParallelMultigoalPlannerActionServer::handle_accepted, this, _1));

  RCLCPP_INFO(node_->get_logger(), "parallel_multigoal_plan action server ready (ik_solver_ns='%s', collision_check_service='%s').",
              ik_solver_ns.c_str(), collision_check_service.c_str());
}

bool ParallelMultigoalPlannerActionServer::ensureJointModel(std::string& error)
{
  std::lock_guard<std::mutex> lk(joint_model_mtx_);
  if (joint_model_ready_)
    return true;

  if (!goal_builder_->fetchJointModel(joint_names_, lower_bound_, upper_bound_, error))
    return false;

  checker_ =
      std::make_shared<ServiceCollisionChecker>(collision_client_, joint_names_, logger_, checker_min_distance_,
                                                 service_timeout_s_);

  joint_model_ready_ = true;
  return true;
}

rclcpp_action::GoalResponse ParallelMultigoalPlannerActionServer::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                                              std::shared_ptr<const Action::Goal> goal)
{
  if (!goal)
    return rclcpp_action::GoalResponse::REJECT;

  if (goal->motion_plan_request.goal_constraints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected: goal_constraints is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ParallelMultigoalPlannerActionServer::handle_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ParallelMultigoalPlannerActionServer::handle_accepted(const std::shared_ptr<GoalHandle> gh)
{
  std::thread{ std::bind(&ParallelMultigoalPlannerActionServer::execute, this, gh) }.detach();
}

Eigen::VectorXd ParallelMultigoalPlannerActionServer::vectorFromJointState(const sensor_msgs::msg::JointState& js,
                                                                           const std::vector<std::string>& joint_order,
                                                                           bool& ok, std::string& error)
{
  Eigen::VectorXd out;
  ok = reorderByJointName(js.name, js.position, joint_order, out, error);
  return out;
}

void ParallelMultigoalPlannerActionServer::execute(const std::shared_ptr<GoalHandle> gh)
{
  const auto goal = gh->get_goal();
  const auto& req = goal->motion_plan_request;

  auto feedback = std::make_shared<Action::Feedback>();
  auto result = std::make_shared<Action::Result>();

  auto publish_feedback = [&](const std::string& status, float progress)
  {
    feedback->status = status;
    feedback->progress = progress;
    gh->publish_feedback(feedback);
  };

  publish_feedback("Resolving joint model", 0.02f);

  std::string err;
  if (!ensureJointModel(err))
  {
    RCLCPP_ERROR(node_->get_logger(), "Cannot resolve joint model: %s", err.c_str());
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::FAILURE);
    gh->abort(result);
    return;
  }

  checker_->setVerbose(goal->verbose);

  bool ok = false;
  Eigen::VectorXd start_q = vectorFromJointState(req.start_state.joint_state, joint_names_, ok, err);
  if (!ok)
  {
    RCLCPP_ERROR(node_->get_logger(), "Invalid start_state: %s", err.c_str());
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::INVALID_ROBOT_STATE);
    gh->abort(result);
    return;
  }

  publish_feedback("Building candidate goal set", 0.10f);

  std::vector<Eigen::VectorXd> goal_configs;
  if (!goal_builder_->buildGoalConfigurations(req.goal_constraints, joint_names_, goal_configs, err))
  {
    RCLCPP_ERROR(node_->get_logger(), "Failed to build goal set: %s", err.c_str());
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
    gh->abort(result);
    return;
  }

  if (goal_configs.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "No candidate goal configuration survived IK/collision filtering.");
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::NO_IK_SOLUTION);
    gh->abort(result);
    return;
  }

  if (goal->verbose)
    RCLCPP_INFO(node_->get_logger(), "Registered %zu candidate goal configuration(s) from %zu goal_constraints element(s).",
                goal_configs.size(), req.goal_constraints.size());

  publish_feedback("Planning", 0.30f);

  const double allowed_time = (req.allowed_planning_time > 0.0) ? req.allowed_planning_time : default_allowed_planning_time_;

  auto metrics = std::make_shared<graph::core::EuclideanMetrics>(logger_);
  auto sampler = std::make_shared<graph::core::UniformSampler>(lower_bound_, upper_bound_, logger_);
  auto goal_cost_fcn = std::make_shared<graph::core::GoalCostFunctionBase>();

  auto solver =
      std::make_shared<graph::mirrt_star::MultigoalSolver>(metrics, checker_, sampler, goal_cost_fcn, logger_);
  solver->config(mirrt_param_ns_);

  auto start_node = std::make_shared<graph::core::Node>(start_q, logger_);
  if (!solver->addStart(start_node))
  {
    RCLCPP_ERROR(node_->get_logger(), "MultigoalSolver::addStart() failed (start state in collision?).");
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::START_STATE_IN_COLLISION);
    gh->abort(result);
    return;
  }

  unsigned int registered_goals = 0;
  for (const auto& q : goal_configs)
  {
    auto goal_node = std::make_shared<graph::core::Node>(q, logger_);
    if (solver->addGoal(goal_node, direct_connect_time_, 0.0))
      ++registered_goals;
  }

  if (registered_goals == 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "None of the %zu candidate goals could be registered with the solver.",
                 goal_configs.size());
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::INVALID_GOAL_CONSTRAINTS);
    gh->abort(result);
    return;
  }

  solver->finalizeProblem();

  graph::core::PathPtr solution;
  bool solved = solver->solved();  // finalizeProblem()/addGoal() may already have found a direct connection

  const auto tic = std::chrono::steady_clock::now();
  unsigned int iter = 0;
  for (; !solved && iter < max_iterations_; ++iter)
  {
    if (gh->is_canceling())
    {
      result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::PREEMPTED);
      gh->canceled(result);
      return;
    }

    if (solver->update(solution))
    {
      solved = true;
      break;
    }

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - tic).count();
    if (elapsed >= allowed_time)
      break;

    if (iter % feedback_period_iterations_ == 0)
      publish_feedback("Planning (iteration " + std::to_string(iter) + ")",
                        0.30f + 0.60f * static_cast<float>(std::min(1.0, elapsed / allowed_time)));
  }

  const double planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - tic).count();

  // solved_ can already be true here without `solution` having been set
  // locally, e.g. when a candidate goal was directly connected to the start
  // tree inside addGoal(), before the update() loop ran at all.
  if (solved && !solution)
    solution = solver->getSolution();

  if (!solved || !solution)
  {
    RCLCPP_ERROR(node_->get_logger(), "No solution found within %.2f s (%u iterations).", allowed_time, iter);
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
    gh->abort(result);
    return;
  }

  publish_feedback("Converting solution", 0.95f);

  result->motion_plan_response = pathToMotionPlanResponse(req, start_q, solution, planning_time);

  publish_feedback("Done", 1.0f);
  gh->succeed(result);
}

moveit_msgs::msg::MotionPlanResponse
ParallelMultigoalPlannerActionServer::pathToMotionPlanResponse(const moveit_msgs::msg::MotionPlanRequest& req,
                                                               const Eigen::VectorXd& start_q,
                                                               const graph::core::PathPtr& solution,
                                                               double planning_time) const
{
  moveit_msgs::msg::MotionPlanResponse out;
  out.trajectory_start = req.start_state;
  out.group_name = req.group_name;
  out.planning_time = planning_time;

  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(start_q);
  for (const auto& wp : solution->getWaypoints())
    waypoints.push_back(wp);

  trajectory_msgs::msg::JointTrajectory jt;
  jt.joint_names = joint_names_;
  jt.points.reserve(waypoints.size());

  rclcpp::Duration time_from_start = rclcpp::Duration::from_seconds(0.0);
  for (const auto& wp : waypoints)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(static_cast<size_t>(wp.size()));
    for (Eigen::Index i = 0; i < wp.size(); ++i)
      pt.positions[static_cast<size_t>(i)] = wp(i);

    const int64_t ns = time_from_start.nanoseconds();
    pt.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
    pt.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    jt.points.push_back(std::move(pt));

    time_from_start = time_from_start + rclcpp::Duration::from_seconds(waypoint_dt_s_);
  }

  out.trajectory.joint_trajectory = std::move(jt);
  out.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return out;
}

moveit_msgs::msg::MotionPlanResponse
ParallelMultigoalPlannerActionServer::errorResponse(const moveit_msgs::msg::MotionPlanRequest& req, int32_t error_code)
{
  moveit_msgs::msg::MotionPlanResponse out;
  out.trajectory_start = req.start_state;
  out.group_name = req.group_name;
  out.planning_time = 0.0;
  out.error_code.val = error_code;
  return out;
}

}  // namespace parallel_multigoal_planner
