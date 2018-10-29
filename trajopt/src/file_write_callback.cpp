/*
Copyright 2018, Southwest Research Institute

    Redistribution and use in source and binary forms, with or without modification,
    are permitted provided that the following conditions are met :

    1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation and / or
        other materials provided with the distribution.

        THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
        "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
    INCLUDING, BUT NOT LIMITED TO,
    THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN
        NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
    INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
    OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
                             LOSS OF USE, DATA, OR PROFITS;
                             OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
    WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
    EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    */
#include <functional>
#include <fstream>
#include <trajopt/file_write_callback.hpp>
#include <ros/ros.h>

using namespace Eigen;
using namespace util;
using namespace std;
namespace trajopt
{
void WriteFile(shared_ptr<ofstream> file,
               const Eigen::Isometry3d& change_base,
               const tesseract::BasicKinConstPtr manip,
               const trajopt::VarArray& vars,
               const OptResults& results)
{
  // Loop over time steps
  TrajArray traj = getTraj(results.x, vars);
  for (auto i = 0; i < traj.rows(); i++)
  {
    // Calc/Write joint values
    VectorXd joint_angles(traj.cols());
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
    Isometry3d pose;
    manip->calcFwdKin(pose, change_base, joint_angles);

    Vector4d rot_vec;
    Quaterniond q(pose.rotation());
    rot_vec(0) = q.w();
    rot_vec(1) = q.x();
    rot_vec(2) = q.y();
    rot_vec(3) = q.z();

    // Write cartesian pose to file
    VectorXd pose_vec = concat(pose.translation(), rot_vec);
    for (auto i = 0; i < pose_vec.size(); i++)
    {
      *file << ',' << pose_vec(i);
    }

    // Write costs to file
    std::vector<double> costs = results.cost_vals;
    for (auto cost : costs)
    {
      *file << ',' << cost;
    }

    // Write constraints to file
    std::vector<double> constraints = results.cnt_viols;
    for (auto constraint : constraints)
    {
      *file << ',' << constraint;
    }

    *file << endl;
  }
  *file << endl;
}  // namespace trajopt

Optimizer::Callback WriteCallback(shared_ptr<ofstream> file, TrajOptProbPtr prob)
{
  if (!file->good())
  {
    ROS_WARN("ofstream passed to create callback not in 'good' state");
  }

  // Write joint names
  vector<string> joint_names = prob->GetEnv()->getJointNames();
  for (size_t i = 0; i < joint_names.size(); i++)
  {
    if (i != 0)
    {
      *file << ',';
    }
    *file << joint_names.at(i);
  }

  // Write cartesian pose labels
  vector<string> pose_str = vector<string>{ "x", "y", "z", "q_w", "q_x", "q_y", "q_z" };
  for (size_t i = 0; i < pose_str.size(); i++)
  {
    *file << ',' << pose_str.at(i);
  }

  // Write cost names
  vector<CostPtr> costs = prob->getCosts();
  for (auto cost : costs)
  {
    *file << ',' << cost->name();
  }

  // Write constraint names
  vector<ConstraintPtr> cnts = prob->getConstraints();
  for (auto cnt : cnts)
  {
    *file << ',' << cnt->name();
  }

  *file << endl;

  // return callback function
  const tesseract::BasicKinConstPtr manip = prob->GetKin();
  const Eigen::Isometry3d change_base = prob->GetEnv()->getLinkTransform(manip->getBaseLinkName());
  return bind(&WriteFile, file, change_base, manip, std::ref(prob->GetVars()), placeholders::_2);
}
}  // namespace trajopt
