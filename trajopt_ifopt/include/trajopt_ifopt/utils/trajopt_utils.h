#ifndef TRAJOPT_IFOPT_TRAJOPT_UTILS_H
#define TRAJOPT_IFOPT_TRAJOPT_UTILS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/cost_term.h>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt/typedefs.hpp>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt
{
/**
 * @brief Converts a vector of trajopt variables into the legacy TrajArray format
 * @param joint_positions Vector of joint positions. Must be in order and all the same length
 * @return TrajArray - size = [joint_positions.size(), joint_positions.n_dof]
 */
inline trajopt::TrajArray toTrajArray(const std::vector<trajopt::JointPosition::ConstPtr>& joint_positions)
{
  trajopt::TrajArray traj_array;
  if (!joint_positions.empty())
    traj_array.resize(static_cast<Eigen::Index>(joint_positions.size()), joint_positions.front()->GetRows());
  for (Eigen::Index i = 0; i < traj_array.rows(); i++)
    traj_array.row(i) = joint_positions[static_cast<std::size_t>(i)]->GetValues().transpose();
  return traj_array;
}

}  // namespace trajopt
#endif
