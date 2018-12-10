/*
Copyright 2018 Southwest Research Institute

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <functional>
#include <fstream>
#include <ros/ros.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/file_write_callback.hpp>

namespace trajopt
{
void WriteFile(std::shared_ptr<std::ofstream> file,
               const Eigen::Isometry3d& change_base,
               const tesseract::BasicKinConstPtr manip,
               const trajopt::VarArray& vars,
               const sco::OptResults& results)
{
  // Loop over time steps
  TrajArray traj = getTraj(results.x, vars);
  for (auto i = 0; i < traj.rows(); i++)
  {
    // Calc/Write joint values
    Eigen::VectorXd joint_angles(traj.cols());
    for (auto j = 0; j < traj.cols(); j++)
    {
      if (j != 0)
      {
        *file << ',';
      }
      *file << traj(i, j);

      joint_angles(j) = traj(i, j);
    }

    // Calc cartesian pose
    Eigen::Isometry3d pose;
    manip->calcFwdKin(pose, change_base, joint_angles);

    Eigen::Vector4d rot_vec;
    Eigen::Quaterniond q(pose.rotation());
    rot_vec(0) = q.w();
    rot_vec(1) = q.x();
    rot_vec(2) = q.y();
    rot_vec(3) = q.z();

    // Write cartesian pose to file
    Eigen::VectorXd pose_vec = concat(pose.translation(), rot_vec);
    for (auto i = 0; i < pose_vec.size(); i++)
    {
      *file << ',' << pose_vec(i);
    }

    // Write costs to file
    const DblVec& costs = results.cost_vals;
    for (const auto& cost : costs)
    {
      *file << ',' << cost;
    }

    // Write constraints to file
    const DblVec& constraints = results.cnt_viols;
    for (const auto& constraint : constraints)
    {
      *file << ',' << constraint;
    }

    *file << std::endl;
  }
  *file << std::endl;
}  // namespace trajopt

sco::Optimizer::Callback WriteCallback(std::shared_ptr<std::ofstream> file, const TrajOptProbPtr& prob)
{
  if (!file->good())
  {
    ROS_WARN("ofstream passed to create callback not in 'good' state");
  }

  // Write joint names
  std::vector<std::string> joint_names = prob->GetEnv()->getJointNames();
  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (i != 0)
    {
      *file << ',';
    }
    *file << joint_names.at(i);
  }

  // Write cartesian pose labels
  std::vector<std::string> pose_str = std::vector<std::string>{ "x", "y", "z", "q_w", "q_x", "q_y", "q_z" };
  for (size_t i = 0; i < pose_str.size(); i++)
  {
    *file << ',' << pose_str.at(i);
  }

  // Write cost names
  const std::vector<sco::CostPtr>& costs = prob->getCosts();
  for (const auto& cost : costs)
  {
    *file << ',' << cost->name();
  }

  // Write constraint names
  const std::vector<sco::ConstraintPtr>& cnts = prob->getConstraints();
  for (const auto& cnt : cnts)
  {
    *file << ',' << cnt->name();
  }

  *file << std::endl;

  // return callback function
  const tesseract::BasicKinConstPtr manip = prob->GetKin();
  const Eigen::Isometry3d change_base = prob->GetEnv()->getLinkTransform(manip->getBaseLinkName());
  return bind(&WriteFile, file, change_base, manip, std::ref(prob->GetVars()), std::placeholders::_2);
}
}  // namespace trajopt
