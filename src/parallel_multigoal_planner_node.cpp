#include <rclcpp/rclcpp.hpp>

#include <parallel_multigoal_planner/parallel_multigoal_planner_action_server.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("parallel_multigoal_planner");

  auto action_server = std::make_shared<parallel_multigoal_planner::ParallelMultigoalPlannerActionServer>(node);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "parallel_multigoal_planner node spinning.");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
