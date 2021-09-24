/**
 * @file trajopt_utils.cpp
 * @brief Contains utilities for converting to trajopt_ifopt types to trajopt_sco or Tesseract types
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <trajopt_ifopt/utils/trajopt_utils.h>

namespace trajopt_ifopt
{
tesseract_common::TrajArray toTrajArray(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& joint_positions)
{
  tesseract_common::TrajArray traj_array;
  if (!joint_positions.empty())
    traj_array.resize(static_cast<Eigen::Index>(joint_positions.size()), joint_positions.front()->GetRows());
  for (Eigen::Index i = 0; i < traj_array.rows(); i++)
    traj_array.row(i) = joint_positions[static_cast<std::size_t>(i)]->GetValues().transpose();
  return traj_array;
}

tesseract_common::JointTrajectory
toJointTrajectory(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& joint_positions)
{
  tesseract_common::JointTrajectory joint_trajectory;

  if (!joint_positions.empty())
    joint_trajectory.reserve(joint_positions.size());

  for (const auto& jp : joint_positions)
    joint_trajectory.emplace_back(jp->GetJointNames(), jp->GetValues());

  return joint_trajectory;
}

}  // namespace trajopt_ifopt
