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
#include "trajopt/ros_kin_chain.h"
#include <ros/ros.h>
#include <eigen_conversions/eigen_kdl.h>
#include <kdl_parser/kdl_parser.hpp>
#include <moveit/robot_model/robot_model.h>
#include <urdf/model.h>
#include <kdl/segment.hpp>

namespace trajopt
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool ROSKinChain::calcFwdKinHelper(Eigen::Affine3d &pose, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, int segment_num) const
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
  pose = change_base * pose;

  return true;
}

bool ROSKinChain::calcFwdKin(Eigen::Affine3d &pose, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  return calcFwdKinHelper(pose, change_base, joint_angles);
}

bool ROSKinChain::calcFwdKin(Eigen::Affine3d &pose, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, const std::string &link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(link_name_too_segment_index_.find(link_name) != link_name_too_segment_index_.end());

  int segment_nr = link_name_too_segment_index_.at(link_name);
  return calcFwdKinHelper(pose, change_base, joint_angles, segment_nr);
//  int joint_index = getLinkParentJointIndex(link_name);
//  return calcFwdKinHelper(pose, change_base, joint_angles, joint_index < 0 ? -1 : joint_index + 1);
}

bool ROSKinChain::calcJacobianHelper(KDL::Jacobian &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, int segment_num) const
{
  KDL::JntArray kdl_joints;
  EigenToKDL(joint_angles, kdl_joints);

  // compute jacobian
  jacobian.resize(joint_angles.size());
  if (jac_solver_->JntToJac(kdl_joints, jacobian, segment_num) < 0)
  {
    ROS_ERROR("Failed to calculate jacobian");
    return false;
  }

  if (!change_base.matrix().isIdentity())
  {
    KDL::Frame frame;
    EigenToKDL(change_base, frame);
    jacobian.changeBase(frame.M);
  }

  return true;
}

bool ROSKinChain::calcJacobian(Eigen::MatrixXd &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles))
  {
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }
  else
  {
    return false;
  }
}

bool ROSKinChain::calcJacobian(Eigen::MatrixXd &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, const std::string &link_name) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  assert(link_name_too_segment_index_.find(link_name) != link_name_too_segment_index_.end());

  int segment_nr = link_name_too_segment_index_.at(link_name);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles, segment_nr))
  {
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }
  else
  {
    return false;
  }

//  int joint_index = getLinkParentJointIndex(link_name);
//  KDL::Jacobian kdl_jacobian;
//  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles, joint_index < 0 ? -1 : joint_index + 1))
//  {
//    KDLToEigen(kdl_jacobian, jacobian);
//    return true;
//  }
//  else
//  {
//    return false;
//  }
}

bool ROSKinChain::calcJacobian(Eigen::MatrixXd &jacobian, const Eigen::Affine3d change_base, const Eigen::VectorXd &joint_angles, const std::string &link_name, const Eigen::Vector3d link_point) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));
  assert(link_name_too_segment_index_.find(link_name) != link_name_too_segment_index_.end());

  int segment_nr = link_name_too_segment_index_.at(link_name);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles, segment_nr))
  {
    // When changing ref point you must provide a vector from the current ref point
    // to the new ref point. This is why the forward kin calculation is required, but
    // need to figure out if there is a more direct way to get this information from KDL.
    Eigen::Affine3d refFrame;
    calcFwdKinHelper(refFrame, change_base, joint_angles, segment_nr);

    Eigen::VectorXd refPoint = refFrame.translation();

    KDL::Vector pt(link_point(0) - refPoint(0), link_point(1) - refPoint(1), link_point(2) - refPoint(2));
    kdl_jacobian.changeRefPoint(pt);
    KDLToEigen(kdl_jacobian, jacobian);
    return true;
  }
  else
  {
    return false;
  }

//  int joint_index = getLinkParentJointIndex(link_name);
//  KDL::Jacobian kdl_jacobian;
//  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles, joint_index < 0 ? -1 : joint_index + 1))
//  {
//    KDL::Vector pt(link_point(0), link_point(1), link_point(2));
//    kdl_jacobian.changeRefPoint(pt);
//    KDLToEigen(kdl_jacobian, jacobian);
//    return true;
//  }
//  else
//  {
//    return false;
//  }
}

bool ROSKinChain::checkJoints(const Eigen::VectorXd &vec) const
{
  if (vec.size() != robot_chain_.getNrOfJoints())
  {
    ROS_ERROR("Number of joint angles (%d) don't match robot_model (%d)",
              (int)vec.size(), robot_chain_.getNrOfJoints());
    return false;
  }

  for (int i=0; i<vec.size(); ++i)
  {
    if ( (vec[i] < joint_limits_(i,0)) || (vec(i) > joint_limits_(i,1)) )
    {
      ROS_WARN("Joint %s is out-of-range (%g < %g < %g)",
                joint_list_[i].c_str(), joint_limits_(i,0), vec(i), joint_limits_(i,1));
    }
  }

  return true;
}

bool ROSKinChain::getJointNames(std::vector<std::string> &names) const
{
    assert(checkInitialized());

    names = joint_list_;
    return true;
}

bool ROSKinChain::getLinkNames(std::vector<std::string> &names) const
{
    assert(checkInitialized());

    names = link_list_;
    return true;
}

//int ROSKin::getLinkParentJointIndex(const std::string &link_name) const
//{
//  std::vector<std::string>::const_iterator it_link = find(link_list_.begin(), link_list_.end(), link_name);
//  if (it_link == link_list_.end())
//  {
//    ROS_ERROR_STREAM("Unable to find link: " << link_name);
//    return -1;
//  }
//  return link_name_too_joint_index_.at(link_name);



////  std::string joint_name = group_->getUpdatedLinkModels()[index]->getParentJointModel()->getName();
////  std::vector<std::string>::const_iterator it = find(joint_list_.begin(), joint_list_.end(), joint_name);
////  if (it != joint_list_.end())
////  {
////    return it-joint_list_.begin();
////  }
////  return -1;
//}

bool ROSKinChain::init(const moveit::core::JointModelGroup* group)
{
  initialized_ = false;

  if(group == NULL)
  {
    ROS_ERROR_STREAM("Null pointer to JointModelGroup");
    return false;
  }

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

  std::map<std::string, int> link_name_too_segment_index;
  for (auto i=0; i < robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment &s = robot_chain_.getSegment(i);
    link_name_too_segment_index[s.getName()] = i;
//    if (i == (robot_chain_.getNrOfSegments() - 1))
//    {
//      link_name_too_segment_index[s.getName()] = -1;
//    }
//    else
//    {
//      link_name_too_segment_index[s.getName()] = i + 1;
//    }
  }

  joint_list_.resize(robot_chain_.getNrOfJoints());
  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);
  std::vector<int> joint_too_segment;
  joint_too_segment.resize(robot_chain_.getNrOfJoints());
  joint_too_segment.back() = -1;

  link_list_ = group->getUpdatedLinkModelNames();
//  link_list_with_geom_ = group->getUpdatedLinkModelsWithGeometryNames();

  for (int i=0, j=0; i<robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment &seg = robot_chain_.getSegment(i);
    const KDL::Joint   &jnt = seg.getJoint();
    if (jnt.getType() == KDL::Joint::None) continue;

    joint_list_[j] = jnt.getName();
    joint_limits_(j,0) = urdf->getJoint(jnt.getName())->limits->lower;
    joint_limits_(j,1) = urdf->getJoint(jnt.getName())->limits->upper;
    if (j > 0)
    {
      joint_too_segment[j - 1] = i;
    }

    // Need to set limits for continuous joints. TODO: This may not be required by the optization library but may be nice to have
    if (urdf->getJoint(jnt.getName())->type == urdf::Joint::CONTINUOUS && std::abs(joint_limits_(j,0) - joint_limits_(j,1)) <= std::numeric_limits<float>::epsilon())
    {
      joint_limits_(j,0) = -4 * M_PI;
      joint_limits_(j,1) = +4 * M_PI;
    }

    j++;
  }

  for (int i=0; i<link_list_.size(); ++i)
    {
      bool found = false;
      const moveit::core::LinkModel *link_model = group->getParentModel().getLinkModel(link_list_[i]);
      while(!found)
      {
        std::string joint_name = link_model->getParentJointModel()->getName();
        std::vector<std::string>::const_iterator it = std::find(joint_list_.begin(), joint_list_.end(), joint_name);
        if (it != joint_list_.end())
        {
          int joint_index = it-joint_list_.begin();
          link_name_too_segment_index_[link_list_[i]] = joint_too_segment[joint_index];
          found = true;
        }
        else
        {
          link_model = link_model->getParentLinkModel();
        }
      }
    }

//  for (int i=0; i<link_list_.size(); ++i)
//  {
//    bool found = false;
////    const moveit::core::LinkModel *link_model = group->getParentModel().getLinkModel(link_list_[i]);
//    std::string link_name = link_list_[i];
//    while(!found)
//    {
////      std::string joint_name = link_model->getParentJointModel()->getName();
////      std::vector<std::string>::const_iterator it = find(joint_list_.begin(), joint_list_.end(), joint_name);
//      auto it = link_name_too_segment_index.find(link_name);
//      if (it != link_name_too_segment_index.end())
//      {
//        link_name_too_segment_index_[link_list_[i]] = it->second;
//        found = true;
//      }
//      else
//      {
//        link_name = group->getParentModel().getLinkModel(link_name)->getParentLinkModel()->getName();
//      }
//    }
//  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  group_ = group;

  return true;
}

void ROSKinChain::KDLToEigen(const KDL::Frame &frame, Eigen::Affine3d &transform)
{
  transform.setIdentity();

  // translation
  for (size_t i=0; i<3; ++i)
    transform(i,3) = frame.p[i];

  // rotation matrix
  for (size_t i=0; i<9; ++i)
    transform(i/3, i%3) = frame.M.data[i];
}

void ROSKinChain::EigenToKDL(const Eigen::Affine3d &transform, KDL::Frame &frame)
{
  frame.Identity();

  for (unsigned int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (unsigned int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i/3, i%3);
}

void ROSKinChain::KDLToEigen(const KDL::Jacobian &jacobian, Eigen::MatrixXd &matrix)
{
  matrix.resize(jacobian.rows(), jacobian.columns());

  for (size_t i=0; i<jacobian.rows(); ++i)
    for (size_t j=0; j<jacobian.columns(); ++j)
      matrix(i,j) = jacobian(i,j);
}

ROSKinChain& ROSKinChain::operator=(const ROSKinChain& rhs)
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
  link_name_too_segment_index_ = rhs.link_name_too_segment_index_;

  return *this;
}

} // namespace trajopt


