/**
 * @file ros_kin.cpp
 * @brief ROS implementation of kinematics functions.
 *
 * Typically, just wrappers around the equivalent KDL calls.
 *
 * @author dsolomon
 * @date Sep 15, 2013
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#include "trajopt/ros_kin.h"
#include <ros/ros.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>

namespace trajopt
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool ROSKin::calcFwdKinHelper(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose, int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // run FK solver
  KDL::Frame kdl_pose;
  if (fk_solver_->JntToCart(kdl_joints, kdl_pose, segment_num) < 0)
  {
    ROS_ERROR("Failed to calculate FK");
    return false;
  }

  KDLToEigen(kdl_pose, pose);
  return true;
}

bool ROSKin::calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const
{
  if (!checkInitialized()) return false;
  if (!checkJoints(joint_angles)) return false;

  return calcFwdKinHelper(joint_angles, pose);
}

bool ROSKin::calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose, const std::string &link_name) const
{
  if (!checkInitialized()) return false;
  if (!checkJoints(joint_angles)) return false;

  int link_num = getLinkNum(link_name);
  return calcFwdKinHelper(joint_angles, pose, link_num < 0 ? -1 : link_num + 1);/*root=0, link1=1, therefore add +1 to link num*/
}

bool ROSKin::calcFwdKin(const Eigen::VectorXd &joint_angles,
                        const std::string &base,
                        const std::string &tip,
                        Eigen::Affine3d &pose) const
{
  // note, because the base and tip are different, fk_solver gets updated
    KDL::Chain chain;
    if (!kdl_tree_.getChain(base, tip, chain))
    {
      ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                       base << "' and '" << tip <<"'");
      return false;
    }

    if (joint_angles.size() != chain.getNrOfJoints())
    {
        ROS_ERROR_STREAM("Number of joint angles [" << joint_angles.size() <<
                            "] must match number of joints [" << chain.getNrOfJoints() << "].");
        return false;
    }

    KDL::ChainFkSolverPos_recursive subchain_fk_solver(chain);

    KDL::JntArray joints;
    joints.data = joint_angles;
    KDL::Frame kdl_pose;
    tf::transformEigenToKDL(pose, kdl_pose);
    if (subchain_fk_solver.JntToCart(joints, kdl_pose) < 0)
        return false;
    return true;
}

bool ROSKin::calcJacobianHelper(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian, int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // compute jacobian
  KDL::Jacobian kdl_jacobian(joint_angles.size());
  if (jac_solver_->JntToJac(kdl_joints, kdl_jacobian, segment_num) < 0)
  {
    ROS_ERROR("Failed to calculate jacobian");
    return false;
  }

  KDLToEigen(kdl_jacobian, jacobian);
  return true;
}

bool ROSKin::calcJacobian(const VectorXd &joint_angles, MatrixXd &jacobian) const
{
  if (!checkInitialized()) return false;
  if (!checkJoints(joint_angles)) return false;

  return calcJacobianHelper(joint_angles, jacobian);
}

bool ROSKin::calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian, const std::string &link_name) const
{
  if (!checkInitialized()) return false;
  if (!checkJoints(joint_angles)) return false;

  int link_num = getLinkNum(link_name);
  return calcJacobianHelper(joint_angles, jacobian, link_num < 0 ? -1 : link_num + 1); /*root=0, link1=1, therefore add +1 to link num*/
}

bool ROSKin::checkJoints(const VectorXd &vec) const
{
  if (vec.size() != robot_chain_.getNrOfJoints())
  {
    ROS_ERROR("Number of joint angles (%d) don't match robot_model (%d)",
              (int)vec.size(), robot_chain_.getNrOfJoints());
    return false;
  }

  bool jnt_bounds_ok = true;
  for (int i=0; i<vec.size(); ++i)
    if ( (vec[i] < joint_limits_(i,0)) || (vec(i) > joint_limits_(i,1)) )
    {
      ROS_ERROR("Joint %d is out-of-range (%g < %g < %g)",
                i, joint_limits_(i,0), vec(i), joint_limits_(i,1));
      jnt_bounds_ok = false;
    }
  if (jnt_bounds_ok == false) return false;

  return true;
}

bool ROSKin::getJointNames(std::vector<std::string> &names) const
{
    if (!initialized_)
    {
        ROS_ERROR("Kinematics must be initialized before retrieving joint names");
        return false;
    }
    names = joint_list_;
    return true;
}

bool ROSKin::getLinkNames(std::vector<std::string> &names) const
{
    if (!initialized_)
    {
        ROS_ERROR("Kinematics must be initialized before retrieving link names");
        return false;
    }
    names = link_list_;
    return true;
}

int ROSKin::getJointNum(const std::string &joint_name) const
{
    std::vector<std::string>::const_iterator it = find(joint_list_.begin(), joint_list_.end(), joint_name);
    if (it != joint_list_.end())
    {
        return it-joint_list_.begin();
    }
    return it-joint_list_.begin()+1;
}

int ROSKin::getLinkNum(const std::string &link_name) const
{
    std::vector<std::string>::const_iterator it = find(link_list_.begin(), link_list_.end(), link_name);
    if (it != link_list_.end())
    {
        return it-link_list_.begin();
    }
    return it-link_list_.begin()+1;
}

bool ROSKin::init(const moveit::core::JointModelGroup* group)
{
  initialized_ = false;

  if(group == NULL)
  {
    ROS_ERROR_STREAM("Null pointer to JointModelGroup");
    return false;
  }

  const robot_model::RobotModel& r  = group->getParentModel();
  const boost::shared_ptr<const urdf::ModelInterface> urdf = group->getParentModel().getURDF();
  base_name_ = group->getLinkModels().front()->getParentLinkModel()->getName();
  tip_name_ = group->getLinkModels().back()->getName();

  if (!urdf->getRoot())
  {
    ROS_ERROR("Invalid URDF in ROSKin::init call");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*urdf, kdl_tree_))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return false;
  }

  if (!kdl_tree_.getChain(base_name_, tip_name_, robot_chain_))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                     base_name_ << "' and '" << tip_name_ <<"'");
    return false;
  }

  joint_list_.resize(robot_chain_.getNrOfJoints());
  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);

  link_list_ = group->getLinkModelNames();

  for (int i=0, j=0; i<robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment &seg = robot_chain_.getSegment(i);
    const KDL::Joint   &jnt = seg.getJoint();
    if (jnt.getType() == KDL::Joint::None) continue;

    joint_list_[j] = jnt.getName();
    joint_limits_(j,0) = urdf->getJoint(jnt.getName())->limits->lower;
    joint_limits_(j,1) = urdf->getJoint(jnt.getName())->limits->upper;
    j++;
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  group_ = group;

  return true;
}

void ROSKin::KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform)
{
  transform.setIdentity();

  // translation
  for (size_t i=0; i<3; ++i)
    transform(i,3) = frame.p[i];

  // rotation matrix
  for (size_t i=0; i<9; ++i)
    transform(i/3, i%3) = frame.M.data[i];
}

void ROSKin::KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix)
{
  matrix.resize(jacobian.rows(), jacobian.columns());

  for (size_t i=0; i<jacobian.rows(); ++i)
    for (size_t j=0; j<jacobian.columns(); ++j)
      matrix(i,j) = jacobian(i,j);
}

bool ROSKin::getSubChain(const std::string link_name, KDL::Chain &chain) const
{
  if (!kdl_tree_.getChain(base_name_, link_name, chain))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" <<
                     base_name_ << "' and '" << link_name <<"'");
    return false;
  }
  else
  {
    return true;
  }
}

ROSKin& ROSKin::operator=(const ROSKin& rhs)
{
  initialized_  = rhs.initialized_;
  robot_chain_  = rhs.robot_chain_;
  kdl_tree_ = rhs.kdl_tree_;
  joint_limits_ = rhs.joint_limits_;
  joint_list_ = rhs.joint_list_;
  link_list_ = rhs.link_list_;
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
  group_ = rhs.group_;
  base_name_ = rhs.base_name_;
  tip_name_ = rhs.tip_name_;

  return *this;
}

} // namespace trajopt


