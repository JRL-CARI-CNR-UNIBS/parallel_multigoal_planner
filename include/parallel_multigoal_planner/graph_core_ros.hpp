#pragma once

/**
 * @file graph_core_ros.hpp
 * @brief ROS 2 glue for graph_core: logger adapter and parameters.
 *
 * graph_core is ROS-free: it logs through a graph::core::LoggerPtr and reads its
 * parameters from a graph::core::ParamMap. This header provides
 * - RclcppGraphLogger, a graph::core::Logger writing to an rclcpp::Logger;
 * - graphParamsFromNode(), which builds a ParamMap from the ROS 2 parameters of
 *   a node under a prefix.
 *
 * The same file is copied in each package that uses graph_core from ROS 2.
 */

#include <graph_core/logger.h>
#include <graph_core/params.h>

#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace parallel_multigoal_planner
{
/**
 * @brief graph::core::Logger writing to an rclcpp::Logger. TRACE is mapped to
 * DEBUG; the message keeps the function and line of the graph_core code.
 */
class RclcppGraphLogger : public graph::core::Logger
{
public:
  explicit RclcppGraphLogger(const rclcpp::Logger& logger) : logger_(logger)
  {
  }

  bool enabled(graph::core::LogLevel level) const override
  {
    return rcutils_logging_logger_is_enabled_for(logger_.get_name(), severity(level));
  }

  void log(graph::core::LogLevel level, const char* func, int line, const std::string& msg) override
  {
    rcutils_log_location_t location = { func, "graph_core", static_cast<size_t>(line) };
    rcutils_log(&location, severity(level), logger_.get_name(), "%s", msg.c_str());
  }

private:
  static int severity(graph::core::LogLevel level)
  {
    switch (level)
    {
      case graph::core::LogLevel::Trace:
      case graph::core::LogLevel::Debug:
        return RCUTILS_LOG_SEVERITY_DEBUG;
      case graph::core::LogLevel::Info:
        return RCUTILS_LOG_SEVERITY_INFO;
      case graph::core::LogLevel::Warn:
        return RCUTILS_LOG_SEVERITY_WARN;
      case graph::core::LogLevel::Error:
        return RCUTILS_LOG_SEVERITY_ERROR;
      default:
        return RCUTILS_LOG_SEVERITY_FATAL;
    }
  }

  rclcpp::Logger logger_;
};

/**
 * @brief Returns the ROS 2 parameters of `node` under `prefix` as a ParamMap.
 *
 * Parameters given to the node (e.g. with a --params-file) under `prefix` are
 * declared if they are not declared yet, so the node does not need to know
 * their names. Keys are relative to the prefix: "mirrt_star.max_distance" ->
 * "max_distance"; deeper levels keep the '.' separator. Boolean and byte arrays
 * are not supported by ParamMap and are skipped with a warning.
 *
 * Call it every time the parameters are needed to pick up the changes made
 * with `ros2 param set`.
 *
 * @param node the node.
 * @param prefix parameter prefix, without the trailing '.'; empty for all the parameters.
 */
inline graph::core::ParamMap graphParamsFromNode(const rclcpp::Node::SharedPtr& node, const std::string& prefix)
{
  const std::string full_prefix = prefix.empty() ? "" : prefix + ".";

  // Declare the overrides under the prefix
  for (const auto& [name, value] : node->get_node_parameters_interface()->get_parameter_overrides())
  {
    if (name.compare(0, full_prefix.size(), full_prefix) == 0 && !node->has_parameter(name))
      node->declare_parameter(name, value);
  }

  graph::core::ParamMap params;
  const auto list = node->list_parameters(prefix.empty() ? std::vector<std::string>{} : std::vector<std::string>{ prefix },
                                          rcl_interfaces::srv::ListParameters::Request::DEPTH_RECURSIVE);
  for (const std::string& name : list.names)
  {
    if (name.compare(0, full_prefix.size(), full_prefix) != 0)
      continue;
    const std::string key = name.substr(full_prefix.size());
    const rclcpp::Parameter p = node->get_parameter(name);
    switch (p.get_type())
    {
      case rclcpp::ParameterType::PARAMETER_BOOL:
        params[key] = p.as_bool();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER:
        params[key] = static_cast<int64_t>(p.as_int());
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE:
        params[key] = p.as_double();
        break;
      case rclcpp::ParameterType::PARAMETER_STRING:
        params[key] = p.as_string();
        break;
      case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
        params[key] = p.as_double_array();
        break;
      case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
      {
        const std::vector<int64_t>& v = p.as_integer_array();
        params[key] = std::vector<int64_t>(v.begin(), v.end());
        break;
      }
      case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
        params[key] = p.as_string_array();
        break;
      case rclcpp::ParameterType::PARAMETER_NOT_SET:
        break;
      default:
        RCLCPP_WARN(node->get_logger(), "Parameter '%s' has a type not supported by graph_core: skipped", name.c_str());
        break;
    }
  }
  return params;
}

}  // namespace parallel_multigoal_planner
