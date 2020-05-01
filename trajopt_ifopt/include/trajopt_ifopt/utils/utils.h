#ifndef TRAJOPT_IFOPT_UTILS_H
#define TRAJOPT_IFOPT_UTILS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/cost_term.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
/**
 * @brief Converts a MatrixX2d (e.g. from forward_kinematics->getLimits()) to a vector of ifopt Boounds
 * @param limits MatrixX2d of bounds. Column 0 will be lower bound. Column 1 will be upper bound
 * @return Vector of ifopt::Bounds
 */
inline std::vector<ifopt::Bounds> toBounds(const Eigen::Ref<Eigen::MatrixX2d>& limits)
{
  std::vector<ifopt::Bounds> bounds;
  for (Eigen::Index i = 0; i < limits.rows(); i++)
    bounds.emplace_back(limits(i, 0), limits(i, 1));
  return bounds;
}

/**
 * @brief Interpolates between two Eigen::VectorXds
 * @param start Start vector. This will be the first vector in the results
 * @param end End vector. This will be the last vector in the results
 * @param steps Length of the returned vector
 * @return A vector of Eigen vectors interpolated from start to end
 */
inline std::vector<Eigen::VectorXd> interpolate(const Eigen::Ref<Eigen::VectorXd>& start,
                                                const Eigen::Ref<Eigen::VectorXd>& end,
                                                Eigen::Index steps)
{
  assert(start.size() == end.size());
  Eigen::VectorXd delta = (end - start) / (steps - 1);
  Eigen::VectorXd running = start;
  std::vector<Eigen::VectorXd> results;
  for (Eigen::Index i = 0; i < steps; i++)
  {
    results.push_back(running);
    running += delta;
  }
  return results;
}

}  // namespace trajopt
#endif
