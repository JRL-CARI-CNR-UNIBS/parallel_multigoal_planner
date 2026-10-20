#pragma once

#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Core>

namespace parallel_multigoal_planner
{

/**
 * @brief Reorder a (names, values) pair into the given canonical joint order.
 * @return false (with `error` populated) if any canonical joint is missing
 * from `names`.
 */
inline bool reorderByJointName(const std::vector<std::string>& names, const std::vector<double>& values,
                               const std::vector<std::string>& canonical_order, Eigen::VectorXd& out,
                               std::string& error)
{
  if (names.size() != values.size())
  {
    error = "names/values size mismatch";
    return false;
  }

  std::unordered_map<std::string, double> map;
  map.reserve(names.size());
  for (size_t i = 0; i < names.size(); ++i)
    map[names[i]] = values[i];

  out.resize(static_cast<Eigen::Index>(canonical_order.size()));
  for (size_t i = 0; i < canonical_order.size(); ++i)
  {
    auto it = map.find(canonical_order[i]);
    if (it == map.end())
    {
      error = "Missing joint '" + canonical_order[i] + "' in provided joint set";
      return false;
    }
    out(static_cast<Eigen::Index>(i)) = it->second;
  }
  return true;
}

}  // namespace parallel_multigoal_planner
