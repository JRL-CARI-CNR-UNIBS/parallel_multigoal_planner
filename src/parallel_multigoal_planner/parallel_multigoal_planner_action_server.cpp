#include <chrono>
#include <sstream>
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
#include <rclcpp/version.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

namespace parallel_multigoal_planner
{

namespace
{
// Humble's create_client() only takes rmw_qos_profile_t; Jazzy (rclcpp 28+) takes rclcpp::QoS
// and deprecates the rmw overload.
#if RCLCPP_VERSION_MAJOR >= 28
rclcpp::QoS serviceQoS() {return rclcpp::ServicesQoS();}
#else
const rmw_qos_profile_t & serviceQoS() {return rmw_qos_profile_services_default;}
#endif
}  // namespace

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

  RCLCPP_DEBUG(node_->get_logger(),
               "Parameters: service_timeout_s=%.3f default_allowed_planning_time=%.3f direct_connect_time=%.3f "
               "max_iterations=%u feedback_period_iterations=%u waypoint_dt_s=%.3f checker_min_distance=%.4f "
               "mirrt_star_param_ns='%s' logger_config_file='%s'",
               service_timeout_s_, default_allowed_planning_time_, direct_connect_time_, max_iterations_,
               feedback_period_iterations_, waypoint_dt_s_, checker_min_distance_, mirrt_param_ns_.c_str(),
               logger_config.c_str());

  cb_group_bounds_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_ik_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_group_collision_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  bounds_client_ = node_->create_client<ik_solver_msgs::srv::GetBound>(
      "/" + ik_solver_ns + "/get_bounds", serviceQoS(), cb_group_bounds_);
  ik_client_ = node_->create_client<ik_solver_msgs::srv::GetIkArray>(
      "/" + ik_solver_ns + "/get_task_redundant_ik_array", serviceQoS(), cb_group_ik_);
  collision_client_ = node_->create_client<ik_solver_msgs::srv::CollisionChecking>(
      collision_check_service, serviceQoS(), cb_group_collision_);

  RCLCPP_DEBUG(node_->get_logger(), "Service clients created: '%s', '%s', '%s'", bounds_client_->get_service_name(),
               ik_client_->get_service_name(), collision_client_->get_service_name());

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
  {
    RCLCPP_DEBUG(node_->get_logger(), "Joint model already cached (%zu joints).", joint_names_.size());
    return true;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Fetching joint model from get_bounds service...");
  if (!goal_builder_->fetchJointModel(joint_names_, lower_bound_, upper_bound_, error))
    return false;

  for (size_t i = 0; i < joint_names_.size(); ++i)
    RCLCPP_DEBUG(node_->get_logger(), "  joint[%zu] '%s' bounds=[%.4f, %.4f]", i, joint_names_[i].c_str(),
                 lower_bound_(static_cast<Eigen::Index>(i)), upper_bound_(static_cast<Eigen::Index>(i)));

  checker_ =
      std::make_shared<ServiceCollisionChecker>(collision_client_, joint_names_, logger_, checker_min_distance_,
                                                 service_timeout_s_);

  joint_model_ready_ = true;
  RCLCPP_DEBUG(node_->get_logger(), "Joint model ready (%zu joints), collision checker created.", joint_names_.size());
  return true;
}

rclcpp_action::GoalResponse ParallelMultigoalPlannerActionServer::handle_goal(const rclcpp_action::GoalUUID& /*uuid*/,
                                                                              std::shared_ptr<const Action::Goal> goal)
{
  if (!goal)
  {
    RCLCPP_DEBUG(node_->get_logger(), "handle_goal: null goal, rejecting.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_DEBUG(node_->get_logger(),
               "handle_goal: group='%s' goal_constraints=%zu allowed_planning_time=%.3f verbose=%d",
               goal->motion_plan_request.group_name.c_str(), goal->motion_plan_request.goal_constraints.size(),
               goal->motion_plan_request.allowed_planning_time, static_cast<int>(goal->verbose));

  if (goal->motion_plan_request.goal_constraints.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Rejected: goal_constraints is empty");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_DEBUG(node_->get_logger(), "handle_goal: accepted.");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
ParallelMultigoalPlannerActionServer::handle_cancel(const std::shared_ptr<GoalHandle> /*gh*/)
{
  RCLCPP_DEBUG(node_->get_logger(), "handle_cancel: cancel request accepted.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void ParallelMultigoalPlannerActionServer::handle_accepted(const std::shared_ptr<GoalHandle> gh)
{
  RCLCPP_DEBUG(node_->get_logger(), "handle_accepted: spawning execution thread.");
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

  RCLCPP_DEBUG(node_->get_logger(), "execute: started.");

  auto publish_feedback = [&](const std::string& status, float progress)
  {
    RCLCPP_DEBUG(node_->get_logger(), "execute: feedback '%s' (%.2f)", status.c_str(), progress);
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

  {
    std::stringstream ss;
    ss << start_q.transpose();
    RCLCPP_DEBUG(node_->get_logger(), "execute: start_q = [%s]", ss.str().c_str());
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

  RCLCPP_DEBUG(node_->get_logger(), "execute: %zu candidate goal configuration(s) built.", goal_configs.size());

  publish_feedback("Planning", 0.30f);

  const double allowed_time = (req.allowed_planning_time > 0.0) ? req.allowed_planning_time : default_allowed_planning_time_;

  auto metrics = std::make_shared<graph::core::EuclideanMetrics>(logger_);
  auto sampler = std::make_shared<graph::core::UniformSampler>(lower_bound_, upper_bound_, logger_);
  auto goal_cost_fcn = std::make_shared<graph::core::GoalCostFunctionBase>();

  auto solver =
      std::make_shared<graph::mirrt_star::MultigoalSolver>(metrics, checker_, sampler, goal_cost_fcn, logger_);
  RCLCPP_DEBUG(node_->get_logger(), "execute: configuring MultigoalSolver from '%s' (allowed_time=%.3f s).",
               mirrt_param_ns_.c_str(), allowed_time);
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
    {
      ++registered_goals;
    }
    else
    {
      std::stringstream ss;
      ss << q.transpose();
      RCLCPP_DEBUG(node_->get_logger(), "execute: addGoal() rejected candidate [%s]", ss.str().c_str());
    }
  }
  RCLCPP_DEBUG(node_->get_logger(), "execute: %u/%zu goal(s) registered with the solver.", registered_goals,
               goal_configs.size());

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
  RCLCPP_DEBUG(node_->get_logger(), "execute: problem finalized, solved before loop=%d. Entering planning loop.",
               static_cast<int>(solved));

  const auto tic = std::chrono::steady_clock::now();
  unsigned int iter = 0;
  for (; !solved && iter < max_iterations_; ++iter)
  {
    if (gh->is_canceling())
    {
      RCLCPP_DEBUG(node_->get_logger(), "execute: goal canceled at iteration %u.", iter);
      result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::PREEMPTED);
      gh->canceled(result);
      return;
    }

    if (solver->update(solution))
    {
      RCLCPP_DEBUG(node_->get_logger(), "execute: solver->update() reported a solution at iteration %u.", iter);
      solved = true;
      break;
    }

    const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - tic).count();
    if (elapsed >= allowed_time)
    {
      RCLCPP_DEBUG(node_->get_logger(), "execute: time budget exhausted (%.3f s) at iteration %u.", elapsed, iter);
      break;
    }

    if (iter % feedback_period_iterations_ == 0)
      publish_feedback("Planning (iteration " + std::to_string(iter) + ")",
                        0.30f + 0.60f * static_cast<float>(std::min(1.0, elapsed / allowed_time)));
  }

  const double planning_time = std::chrono::duration<double>(std::chrono::steady_clock::now() - tic).count();
  RCLCPP_DEBUG(node_->get_logger(), "execute: planning loop ended after %u iteration(s), %.3f s, solved=%d.", iter,
               planning_time, static_cast<int>(solved));

  // solved_ can already be true here without `solution` having been set
  // locally, e.g. when a candidate goal was directly connected to the start
  // tree inside addGoal(), before the update() loop ran at all.
  if (solved && !solution)
  {
    RCLCPP_DEBUG(node_->get_logger(), "execute: retrieving solution via getSolution().");
    solution = solver->getSolution();
  }

  if (!solved || !solution)
  {
    RCLCPP_ERROR(node_->get_logger(), "No solution found within %.2f s (%u iterations).", allowed_time, iter);
    result->motion_plan_response = errorResponse(req, moveit_msgs::msg::MoveItErrorCodes::PLANNING_FAILED);
    gh->abort(result);
    return;
  }

  RCLCPP_DEBUG(node_->get_logger(), "execute: solution cost=%.4f, %zu waypoint(s).", solution->cost(),
               solution->getWaypoints().size());

  publish_feedback("Converting solution", 0.95f);

  result->motion_plan_response = pathToMotionPlanResponse(req, start_q, solution, planning_time);

  publish_feedback("Done", 1.0f);
  gh->succeed(result);
  RCLCPP_DEBUG(node_->get_logger(), "execute: goal succeeded.");
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

  RCLCPP_DEBUG(node_->get_logger(), "pathToMotionPlanResponse: %zu trajectory point(s), dt=%.3f s.", jt.points.size(),
               waypoint_dt_s_);
  out.trajectory.joint_trajectory = std::move(jt);
  out.error_code.val = moveit_msgs::msg::MoveItErrorCodes::SUCCESS;
  return out;
}

moveit_msgs::msg::MotionPlanResponse
ParallelMultigoalPlannerActionServer::errorResponse(const moveit_msgs::msg::MotionPlanRequest& req, int32_t error_code)
{
  RCLCPP_DEBUG(rclcpp::get_logger("parallel_multigoal_planner"), "errorResponse: error_code=%d", error_code);
  moveit_msgs::msg::MotionPlanResponse out;
  out.trajectory_start = req.start_state;
  out.group_name = req.group_name;
  out.planning_time = 0.0;
  out.error_code.val = error_code;
  return out;
}

}  // namespace parallel_multigoal_planner
