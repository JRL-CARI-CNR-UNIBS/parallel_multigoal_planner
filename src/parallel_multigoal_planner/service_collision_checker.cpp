#include <cmath>
#include <algorithm>

#include <parallel_multigoal_planner/service_collision_checker.hpp>

namespace parallel_multigoal_planner
{

ServiceCollisionChecker::ServiceCollisionChecker(const ClientT::SharedPtr& client,
                                                  const std::vector<std::string>& joint_names,
                                                  const cnr_logger::TraceLoggerPtr& logger,
                                                  const double& min_distance, const double& service_timeout_s)
  : graph::core::CollisionCheckerBase(logger, min_distance)
  , client_(client)
  , joint_names_(joint_names)
  , service_timeout_s_(service_timeout_s)
{
}

std::vector<bool> ServiceCollisionChecker::batchCheck(const std::vector<Eigen::VectorXd>& configurations)
{
  std::vector<bool> feasible(configurations.size(), false);

  auto request = std::make_shared<ik_solver_msgs::srv::CollisionChecking::Request>();
  request->joint_names = joint_names_;
  request->detailed = false;
  request->solutions.reserve(configurations.size());
  for (const auto& q : configurations)
  {
    ik_solver_msgs::msg::Configuration cfg;
    cfg.configuration.assign(q.data(), q.data() + q.size());
    request->solutions.push_back(std::move(cfg));
  }

  if (!client_->service_is_ready())
  {
    CNR_ERROR(logger_, "/collision_check service is not available.");
    return feasible;
  }

  auto future = client_->async_send_request(request);
  auto status = future.wait_for(std::chrono::duration<double>(service_timeout_s_));
  if (status != std::future_status::ready)
  {
    CNR_ERROR(logger_, "Timeout waiting for /collision_check response.");
    return feasible;
  }

  auto response = future.get();
  if (response->result.size() != configurations.size())
  {
    CNR_ERROR(logger_, "/collision_check returned " << response->result.size() << " results for "
                                                      << configurations.size()
                                                      << " requested configurations (joint_names mismatch?).");
    return feasible;
  }

  for (size_t i = 0; i < response->result.size(); ++i)
    feasible[i] = (response->result[i].feasibility > 0.5) && !response->result[i].out_of_bound;

  return feasible;
}

bool ServiceCollisionChecker::check(const Eigen::VectorXd& configuration)
{
  auto res = batchCheck({ configuration });
  return res.front();
}

bool ServiceCollisionChecker::checkConnection(const Eigen::VectorXd& configuration1,
                                              const Eigen::VectorXd& configuration2)
{
  double dist = (configuration2 - configuration1).norm();

  std::vector<Eigen::VectorXd> points;
  if (dist < min_distance_)
  {
    points.push_back(configuration1);
    points.push_back(configuration2);
  }
  else
  {
    unsigned int n = static_cast<unsigned int>(std::ceil(dist / min_distance_));
    points.reserve(n + 1);
    for (unsigned int i = 0; i <= n; ++i)
      points.push_back(configuration1 + (configuration2 - configuration1) * (static_cast<double>(i) / n));
  }

  std::vector<bool> feasible = batchCheck(points);
  return std::all_of(feasible.begin(), feasible.end(), [](bool f) { return f; });
}

graph::core::CollisionCheckerPtr ServiceCollisionChecker::clone()
{
  return std::make_shared<ServiceCollisionChecker>(client_, joint_names_, logger_, min_distance_, service_timeout_s_);
}

}  // namespace parallel_multigoal_planner
