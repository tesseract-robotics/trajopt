/**
 * @file joint_position_variable.cpp
 * @brief Contains the joint position variable
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
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/utils/ifopt_utils.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
JointPosition::JointPosition(const Eigen::Ref<const Eigen::VectorXd>& init_value,
                             std::vector<std::string> joint_names,
                             const std::string& name)
  : ifopt::VariableSet(static_cast<int>(init_value.size()), name), joint_names_(std::move(joint_names))
{
  bounds_ = std::vector<ifopt::Bounds>(static_cast<size_t>(init_value.size()), ifopt::NoBound);
  values_ = init_value;
}

JointPosition::JointPosition(const Eigen::Ref<const Eigen::VectorXd>& init_value,
                             std::vector<std::string> joint_names,
                             const ifopt::Bounds& bounds,
                             const std::string& name)
  : ifopt::VariableSet(static_cast<int>(init_value.size()), name), joint_names_(std::move(joint_names))
{
  /** @todo Print warning if init value is not within bounds */
  bounds_ = std::vector<ifopt::Bounds>(static_cast<size_t>(init_value.size()), bounds);
  values_ = trajopt_ifopt::getClosestValidPoint(init_value, bounds_);

  if (!values_.isApprox(init_value, 1e-10))
  {
    CONSOLE_BRIDGE_logWarn("The initial values are not within the provided bounds. Adjusting to be within the "
                           "bounds.");
  }
}

JointPosition::JointPosition(const Eigen::Ref<const Eigen::VectorXd>& init_value,
                             std::vector<std::string> joint_names,
                             const tesseract_common::KinematicLimits& bounds,
                             const std::string& name)
  : ifopt::VariableSet(static_cast<int>(init_value.size()), name), joint_names_(std::move(joint_names))
{
  /** @todo Print warning if init value is not within bounds */
  bounds_ = std::vector<ifopt::Bounds>(static_cast<size_t>(init_value.size()), ifopt::NoBound);
  for (Eigen::Index i = 0; i < init_value.size(); ++i)
    bounds_[static_cast<std::size_t>(i)] = ifopt::Bounds(bounds.joint_limits(i, 0), bounds.joint_limits(i, 1));

  values_ = trajopt_ifopt::getClosestValidPoint(init_value, bounds_);

  if (!values_.isApprox(init_value, 1e-10))
  {
    CONSOLE_BRIDGE_logWarn("The initial values are not within the provided bounds. Adjusting to be within the "
                           "bounds.");
  }
}

void JointPosition::SetVariables(const Eigen::VectorXd& x) { values_ = x; }

Eigen::VectorXd JointPosition::GetValues() const { return values_; }

JointPosition::VecBound JointPosition::GetBounds() const { return bounds_; }

void JointPosition::SetBounds(VecBound& new_bounds) { bounds_ = new_bounds; }

void JointPosition::SetBounds(const Eigen::Ref<const Eigen::MatrixX2d>& bounds) { bounds_ = toBounds(bounds); }

std::vector<std::string> JointPosition::GetJointNames() const { return joint_names_; }
}  // namespace trajopt_ifopt
