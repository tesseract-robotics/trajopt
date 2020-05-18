/**
 * @file cartesian_error_plotter.h
 * @brief A callback to plot cartesian constraints
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
#include <trajopt_sqp/callbacks/cartesian_error_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

CartesianErrorPlottingCallback::CartesianErrorPlottingCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

void CartesianErrorPlottingCallback::plot(const ifopt::Problem& /*nlp*/)
{
  for (const auto& cnt : cart_position_cnts_)
  {
    Eigen::Isometry3d current_pose = cnt->GetCurrentPose();
    Eigen::Isometry3d target_pose = cnt->GetTargetPose();

    if (plotter_)
    {
      plotter_->plotAxis(current_pose, 0.05);
      plotter_->plotAxis(target_pose, 0.05);
      plotter_->plotArrow(current_pose.translation(), target_pose.translation(), Eigen::Vector4d(1, 0, 1, 1), 0.005);
    }
  }
}

void CartesianErrorPlottingCallback::addConstraintSet(const trajopt::CartPosConstraint::ConstPtr& cart_position_cnt)
{
  cart_position_cnts_.push_back(cart_position_cnt);
};

void CartesianErrorPlottingCallback::addConstraintSet(
    const std::vector<trajopt::CartPosConstraint::ConstPtr>& cart_position_cnts)
{
  for (const auto& cnt : cart_position_cnts)
    cart_position_cnts_.push_back(cnt);
}

bool CartesianErrorPlottingCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults&)
{
  plot(nlp);
  return true;
};
