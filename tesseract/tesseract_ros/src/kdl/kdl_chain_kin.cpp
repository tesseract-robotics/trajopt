/**
 * @file kdl_chain_kin.cpp
 * @brief Tesseract ROS KDL Chain kinematics implementation.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include "tesseract_ros/kdl/kdl_chain_kin.h"
#include "tesseract_ros/kdl/kdl_utils.h"
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/segment.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>
#include <urdf/model.h>

namespace tesseract
{
namespace tesseract_ros
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool KDLChainKin::calcFwdKinHelper(Eigen::Affine3d& pose,
                                   const Eigen::Affine3d& change_base,
                                   const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                   int segment_num) const
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

bool KDLChainKin::calcFwdKin(Eigen::Affine3d& pose,
                             const Eigen::Affine3d& change_base,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  return calcFwdKinHelper(pose, change_base, joint_angles);
}

bool KDLChainKin::calcFwdKin(Eigen::Affine3d& pose,
                             const Eigen::Affine3d& change_base,
                             const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                             const std::string& link_name,
                             const EnvState& state) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  const std::string& chain_link_name = link_name_too_chain_link_name_.at(link_name);
  assert(segment_index_.find(chain_link_name) != segment_index_.end());

  int segment_nr = segment_index_.at(chain_link_name);
  if (calcFwdKinHelper(pose, change_base, joint_angles, segment_nr))
  {
    // This is required because manipulators are not aware of branches off the chain
    // so it needs the current state to make calculations for links affected by the chain
    // but not directly part of the chain.
    if (chain_link_name != link_name)
      pose = pose * (state.transforms.at(chain_link_name).inverse() * state.transforms.at(link_name));

    return true;
  }

  return false;
}

bool KDLChainKin::calcJacobianHelper(KDL::Jacobian& jacobian,
                                     const Eigen::Affine3d& change_base,
                                     const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                     int segment_num) const
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

bool KDLChainKin::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                               const Eigen::Affine3d& change_base,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const
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

bool KDLChainKin::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                               const Eigen::Affine3d& change_base,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name,
                               const EnvState& state) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  const std::string& chain_link_name = link_name_too_chain_link_name_.at(link_name);
  assert(segment_index_.find(chain_link_name) != segment_index_.end());

  int segment_nr = segment_index_.at(chain_link_name);
  KDL::Jacobian kdl_jacobian;

  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles, segment_nr))
  {
    if (chain_link_name == link_name)
    {
      KDLToEigen(kdl_jacobian, jacobian);
      return true;
    }
    else
    {
      Eigen::Vector3d temp =
          (state.transforms.at(chain_link_name).inverse() * state.transforms.at(link_name)).translation();
      KDL::Vector pt(temp[0], temp[1], temp[2]);
      kdl_jacobian.changeRefPoint(pt);
      KDLToEigen(kdl_jacobian, jacobian);
      return true;
    }
  }

  return false;
}

bool KDLChainKin::calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                               const Eigen::Affine3d& change_base,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                               const std::string& link_name,
                               const EnvState& /*state*/,
                               const Eigen::Ref<const Eigen::Vector3d>& link_point) const
{
  assert(checkInitialized());
  assert(checkJoints(joint_angles));

  const std::string& chain_link_name = link_name_too_chain_link_name_.at(link_name);
  assert(segment_index_.find(chain_link_name) != segment_index_.end());

  int segment_nr = segment_index_.at(chain_link_name);
  KDL::Jacobian kdl_jacobian;
  if (calcJacobianHelper(kdl_jacobian, change_base, joint_angles, segment_nr))
  {
    // When changing ref point you must provide a vector from the current ref
    // point
    // to the new ref point. This is why the forward kin calculation is
    // required, but
    // need to figure out if there is a more direct way to get this information
    // from KDL.
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
}

bool KDLChainKin::checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const
{
  if (vec.size() != robot_chain_.getNrOfJoints())
  {
    ROS_ERROR(
        "Number of joint angles (%d) don't match robot_model (%d)", (int)vec.size(), robot_chain_.getNrOfJoints());
    return false;
  }

  for (int i = 0; i < vec.size(); ++i)
  {
    if ((vec[i] < joint_limits_(i, 0)) || (vec(i) > joint_limits_(i, 1)))
    {
      ROS_WARN("Joint %s is out-of-range (%g < %g < %g)",
               joint_list_[i].c_str(),
               joint_limits_(i, 0),
               vec(i),
               joint_limits_(i, 1));
    }
  }

  return true;
}

const std::vector<std::string>& KDLChainKin::getJointNames() const
{
  assert(checkInitialized());
  return joint_list_;
}

const std::vector<std::string>& KDLChainKin::getLinkNames() const
{
  assert(checkInitialized());
  return link_list_;
}

const Eigen::MatrixX2d& KDLChainKin::getLimits() const { return joint_limits_; }
void KDLChainKin::addChildrenRecursive(const std::string& chain_link_name,
                                       urdf::LinkConstSharedPtr urdf_link,
                                       const std::string& next_chain_segment)
{
  // recursively build child links
  link_list_.push_back(urdf_link->name);
  link_name_too_chain_link_name_[urdf_link->name] = chain_link_name;
  for (std::size_t i = 0; i < urdf_link->child_links.size(); ++i)
  {
    // Don't process the next chain link
    if (urdf_link->child_links[i]->name != next_chain_segment)
    {
      addChildrenRecursive(chain_link_name, urdf_link->child_links[i], next_chain_segment);
    }
  }
}

bool KDLChainKin::init(urdf::ModelInterfaceConstSharedPtr model,
                       const std::string& base_link,
                       const std::string& tip_link,
                       const std::string name)
{
  initialized_ = false;

  if (model == nullptr)
  {
    ROS_ERROR_STREAM("Null pointer to URDF Model");
    return false;
  }

  model_ = model;
  base_name_ = base_link;
  tip_name_ = tip_link;
  name_ = name;

  if (!model_->getRoot())
  {
    ROS_ERROR("Invalid URDF in ROSKin::init call");
    return false;
  }

  if (!kdl_parser::treeFromUrdfModel(*model_, kdl_tree_))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return false;
  }

  if (!kdl_tree_.getChain(base_name_, tip_name_, robot_chain_))
  {
    ROS_ERROR_STREAM("Failed to initialize KDL between URDF links: '" << base_name_ << "' and '" << tip_name_ << "'");
    return false;
  }

  joint_list_.resize(robot_chain_.getNrOfJoints());
  joint_limits_.resize(robot_chain_.getNrOfJoints(), 2);
  std::vector<int> joint_too_segment;
  joint_too_segment.resize(robot_chain_.getNrOfJoints());
  joint_too_segment.back() = -1;

  for (unsigned i = 0, j = 0; i < robot_chain_.getNrOfSegments(); ++i)
  {
    const KDL::Segment& seg = robot_chain_.getSegment(i);
    const KDL::Joint& jnt = seg.getJoint();

    if (i != (robot_chain_.getNrOfSegments() - 1))
      addChildrenRecursive(seg.getName(), model_->getLink(seg.getName()), robot_chain_.getSegment(i + 1).getName());
    else
      addChildrenRecursive(seg.getName(), model_->getLink(seg.getName()), seg.getName());

    if (jnt.getType() == KDL::Joint::None)
      continue;

    joint_list_[j] = jnt.getName();
    urdf::JointConstSharedPtr joint = model_->getJoint(jnt.getName());
    joint_limits_(j, 0) = joint->limits->lower;
    joint_limits_(j, 1) = joint->limits->upper;
    if (j > 0)
      joint_too_segment[j - 1] = i;

    // Need to set limits for continuous joints. TODO: This may not be required
    // by the optization library but may be nice to have
    if (joint->type == urdf::Joint::CONTINUOUS &&
        std::abs(joint_limits_(j, 0) - joint_limits_(j, 1)) <= std::numeric_limits<float>::epsilon())
    {
      joint_limits_(j, 0) = -4 * M_PI;
      joint_limits_(j, 1) = +4 * M_PI;
    }
    ++j;
  }

  for (unsigned i = 0; i < robot_chain_.getNrOfSegments(); ++i)
  {
    bool found = false;
    const KDL::Segment& seg = robot_chain_.getSegment(i);
    urdf::LinkConstSharedPtr link_model = model_->getLink(seg.getName());
    while (!found)
    {
      std::string joint_name = link_model->parent_joint->name;
      std::vector<std::string>::const_iterator it = std::find(joint_list_.begin(), joint_list_.end(), joint_name);
      if (it != joint_list_.end())
      {
        int joint_index = it - joint_list_.begin();
        segment_index_[seg.getName()] = joint_too_segment[joint_index];
        found = true;
      }
      else
      {
        link_model = link_model->getParent();
      }
    }
  }

  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));

  initialized_ = true;
  return initialized_;
}

void KDLChainKin::addAttachedLink(const std::string& link_name, const std::string& parent_link_name)
{
  auto check = std::find(link_list_.begin(), link_list_.end(), link_name);

  if (check != link_list_.end())
  {
    ROS_WARN("Tried add attached link to manipulator that already exists!");
    return;
  }

  auto it = link_name_too_chain_link_name_.find(parent_link_name);
  if (it != link_name_too_chain_link_name_.end())
  {
    link_name_too_chain_link_name_[link_name] = it->second;
    attached_link_list_.push_back(link_name);
    link_list_.push_back(link_name);
  }
  else
  {
    ROS_WARN("Tried add attached link to manipulator, but parent link is not "
             "associated with this manipulator!");
  }
}

void KDLChainKin::removeAttachedLink(const std::string& link_name)
{
  auto check = std::find(attached_link_list_.begin(), attached_link_list_.end(), link_name);
  if (check != attached_link_list_.end())
  {
    attached_link_list_.erase(std::remove(attached_link_list_.begin(), attached_link_list_.end(), link_name),
                              attached_link_list_.end());
    link_list_.erase(std::remove(link_list_.begin(), link_list_.end(), link_name), link_list_.end());
    link_name_too_chain_link_name_.erase(link_name);
  }
  else
  {
    ROS_WARN("Tried to remove attached link from manipulator which is not "
             "attached!");
  }
}

void KDLChainKin::clearAttachedLinks()
{
  for (const auto& al : attached_link_list_)
  {
    link_list_.erase(std::remove(link_list_.begin(), link_list_.end(), al), link_list_.end());
    link_name_too_chain_link_name_.erase(al);
  }
  attached_link_list_.clear();
}

KDLChainKin& KDLChainKin::operator=(const KDLChainKin& rhs)
{
  initialized_ = rhs.initialized_;
  robot_chain_ = rhs.robot_chain_;
  kdl_tree_ = rhs.kdl_tree_;
  joint_limits_ = rhs.joint_limits_;
  joint_list_ = rhs.joint_list_;
  link_list_ = rhs.link_list_;
  fk_solver_.reset(new KDL::ChainFkSolverPos_recursive(robot_chain_));
  jac_solver_.reset(new KDL::ChainJntToJacSolver(robot_chain_));
  model_ = rhs.model_;
  base_name_ = rhs.base_name_;
  tip_name_ = rhs.tip_name_;
  segment_index_ = rhs.segment_index_;
  attached_link_list_ = rhs.attached_link_list_;
  link_name_too_chain_link_name_ = rhs.link_name_too_chain_link_name_;

  return *this;
}
}
}
