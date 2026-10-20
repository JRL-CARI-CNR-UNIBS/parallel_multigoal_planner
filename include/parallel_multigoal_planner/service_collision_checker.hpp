#pragma once

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <graph_core/collision_checkers/collision_checker_base.h>

#include <ik_solver_msgs/srv/collision_checking.hpp>

namespace parallel_multigoal_planner
{

/**
 * @brief graph_core CollisionCheckerBase implementation that delegates every
 * collision query to the /collision_check ROS 2 service exposed by
 * collision_checker_server (ik_solver_msgs/srv/CollisionChecking), instead of
 * maintaining a local MoveIt planning scene.
 *
 * check() sends a single-configuration request; checkConnection() batches all
 * the sampled points of a tree edge into a single service call, since this is
 * the hot path called by mirrt_star while extending the tree.
 */
class ServiceCollisionChecker : public graph::core::CollisionCheckerBase
{
public:
  /**
   * @brief Client type of the collision checking service.
   */
  using ClientT = rclcpp::Client<ik_solver_msgs::srv::CollisionChecking>;

  /**
   * @brief Constructor.
   * @param client Client of the collision checking service.
   * @param joint_names Joint names, in the order of the configurations passed to check().
   * @param logger Logger.
   * @param min_distance Distance between the points checked along a connection.
   * @param service_timeout_s Timeout of each service call [s].
   */
  ServiceCollisionChecker(const ClientT::SharedPtr& client, const std::vector<std::string>& joint_names,
                           const cnr_logger::TraceLoggerPtr& logger, const double& min_distance = 0.05,
                           const double& service_timeout_s = 5.0);

  /**
   * @brief Check one configuration with one service call.
   * @param configuration Configuration.
   * @return True if collision-free; false if in collision or if the service fails.
   */
  bool check(const Eigen::VectorXd& configuration) override;

  /**
   * @brief Check the segment between two configurations, sampled every @c min_distance, with one batched service call.
   * @param configuration1 First configuration.
   * @param configuration2 Second configuration.
   * @return True if all the sampled points are collision-free.
   */
  bool checkConnection(const Eigen::VectorXd& configuration1, const Eigen::VectorXd& configuration2) override;

  /**
   * @brief Copy that shares the same service client.
   */
  graph::core::CollisionCheckerPtr clone() override;

private:
  /**
   * @brief Send one batched CollisionChecking request for all the given
   * configurations and return, for each of them (same order), whether it is
   * feasible. On service failure/timeout every entry is conservatively false.
   */
  std::vector<bool> batchCheck(const std::vector<Eigen::VectorXd>& configurations);

  ClientT::SharedPtr client_;
  std::vector<std::string> joint_names_;
  double service_timeout_s_;
};

}  // namespace parallel_multigoal_planner
