#ifndef TRAJOPT_COMMON_UTILS_HPP
#define TRAJOPT_COMMON_UTILS_HPP

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
template <typename T>
std::vector<T> concat(const std::vector<T>& a, const std::vector<T>& b)
{
  std::vector<T> out;
  out.reserve(a.size() + b.size());
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

/**
 * @brief Apply a twist for dt to a given transform
 * @param t1 The transform to apply twist.
 * @param twist The twist to apply.
 * @param dt The delta time for which the twist is applied
 * @return Transform result of applying a twist for dt.
 */
Eigen::Isometry3d addTwist(const Eigen::Isometry3d& t1,
                           const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist,
                           double dt);
}  // namespace trajopt_common

#endif  // TRAJOPT_COMMON_UTILS_HPP
