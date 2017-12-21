#include "trajopt/ros_env.h"

namespace trajopt
{

using Eigen::MatrixXd;
using Eigen::VectorXd;


void ROSEnv::calcDistances(const Eigen::VectorXd &joint_angles)
{

}

void ROSEnv::calcDistances(const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names)
{

}

void ROSEnv::calcCollisions(const Eigen::VectorXd &joint_angles)
{

}

void ROSEnv::calcCollisions(const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names)
{

}

Eigen::VectorXd ROSEnv::getCurrentJointValues(std::string manipulator_name) const
{

  std::vector<std::string> joint_names = env_->getRobotModel()->getJointModelGroup(manipulator_name)->getActiveJointModelNames();
  Eigen::VectorXd start_pos(joint_names.size());

  for(auto j = 0u; j < joint_names.size(); j++)
  {
    start_pos(j) = env_->getCurrentState().getVariablePosition(joint_names[j]);
  }

  return start_pos;
}

bool ROSEnv::hasManipulator(std::string manipulator_name) const
{
  return env_->getRobotModel()->hasJointModelGroup(manipulator_name);
}

BasicKinPtr ROSEnv::getManipulatorKin(std::string manipulator_name) const
{
  ROSKinPtr manip(new ROSKin());
  manip->init(env_->getRobotModel()->getJointModelGroup(manipulator_name));
  return manip;
}

}
