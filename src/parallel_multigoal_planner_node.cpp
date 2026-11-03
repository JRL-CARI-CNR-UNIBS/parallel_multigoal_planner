#include <rclcpp/rclcpp.hpp>
#include <rclcpp/version.h>

#include <parallel_multigoal_planner/parallel_multigoal_planner_action_server.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("parallel_multigoal_planner");

  auto action_server = std::make_shared<parallel_multigoal_planner::ParallelMultigoalPlannerActionServer>(node);

  // Humble's rclcpp_action server is not safe under a MultiThreadedExecutor (data race in the
  // action server waitable, fixed in later releases): it segfaults when a goal completes.
  // All blocking work runs in the detached execute() thread, so a single executor thread is enough.
#if RCLCPP_VERSION_MAJOR >= 28
  rclcpp::executors::MultiThreadedExecutor executor;
#else
  rclcpp::executors::SingleThreadedExecutor executor;
#endif
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "parallel_multigoal_planner node spinning.");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
