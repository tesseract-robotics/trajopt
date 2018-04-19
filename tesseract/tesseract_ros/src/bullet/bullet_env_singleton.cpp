/**
 * @file bullet_env.cpp
 * @brief Tesseract ROS Bullet environment implementation.
 *
 * @author John Schulman
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 * @copyright Copyright (c) 2013, John Schulman
 *
 * @par License
 * Software License Agreement (BSD-2-Clause)
 * @par
 * All rights reserved.
 * @par
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * @par
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * @par
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include "tesseract_ros/bullet/bullet_env_singleton.h"
#include "tesseract_ros/kdl/kdl_chain_kin.h"
#include "tesseract_ros/ros_tesseract_utils.h"
#include <geometric_shapes/shape_operations.h>
#include <eigen_conversions/eigen_msg.h>
#include <iostream>
#include <limits>
#include <octomap/octomap.h>

namespace tesseract
{
namespace tesseract_ros
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool BulletEnvSingleton::init(const urdf::ModelInterfaceConstSharedPtr urdf_model)
{
  init(urdf_model, nullptr);
}

bool BulletEnvSingleton::init(const urdf::ModelInterfaceConstSharedPtr urdf_model, const srdf::ModelConstSharedPtr srdf_model)
{
  initialized_ = false;
  urdf_model_ = urdf_model;

  if(urdf_model_ == nullptr)
  {
    ROS_ERROR_STREAM("Null pointer to URDF Model");
    return initialized_;
  }

  if (!urdf_model_->getRoot())
  {
    ROS_ERROR("Invalid URDF in ROSBulletEnvSingleton::init call");
    return initialized_;
  }

  KDL::Tree *kdl_tree = new KDL::Tree();
  if (!kdl_parser::treeFromUrdfModel(*urdf_model_, *kdl_tree))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return initialized_;
  }
  kdl_tree_ = std::shared_ptr<KDL::Tree>(kdl_tree);
  initialized_ = true;

  if (initialized_)
  {
    link_names_.reserve(urdf_model->links_.size());
    for (auto& link : urdf_model->links_)
    {
      link_names_.push_back(link.second->name);
      if (link.second->collision_array.size() > 0)
      {
        COWPtr new_cow(new COW(link.second.get()));
        if (new_cow)
        {
          setContactDistance(new_cow, BULLET_DEFAULT_CONTACT_DISTANCE);
          manager_.m_link2cow[new_cow->getID()] = new_cow;
          manager_.addCollisionObject(new_cow);
          ROS_DEBUG("Added collision object for link %s", link.second->name.c_str());
        }
        else
        {
          ROS_DEBUG("ignoring link %s", link.second->name.c_str());
        }
      }
    }

    current_state_ = EnvStatePtr(new EnvState());
    kdl_jnt_array_.resize(kdl_tree_->getNrOfJoints());
    joint_names_.resize(kdl_tree_->getNrOfJoints());
    int j = 0;
    for (const auto& seg : kdl_tree_->getSegments())
    {
      const KDL::Joint &jnt = seg.second.segment.getJoint();

      if (jnt.getType() == KDL::Joint::None) continue;
      joint_names_[j] = jnt.getName();
      joint_to_qnr_.insert(std::make_pair(jnt.getName(), seg.second.q_nr));
      kdl_jnt_array_(seg.second.q_nr) = 0.0;
      current_state_->joints.insert(std::make_pair(jnt.getName(), 0.0));

      j++;
    }

    calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());
  }

  if (srdf_model != nullptr)
  {
    srdf_model_ = srdf_model;
    for (const auto& group: srdf_model_->getGroups())
    {
      for (const auto& chain: group.chains_)
      {
        KDLChainKinPtr manip(new KDLChainKin());
        manip->init(urdf_model_, chain.first, chain.second, group.name_);
        manipulators_.insert(std::make_pair(group.name_, manip));
      }
    }

    // TODO: Need to add other options

    // Populate allowed collision matrix
    for (const auto& pair: srdf_model_->getDisabledCollisionPairs())
    {
      allowed_collision_matrix_->addAllowedCollision(pair.link1_, pair.link2_, pair.reason_);
    }
  }

  request_.acm = allowed_collision_matrix_;
  request_.contact_distance = BULLET_DEFAULT_CONTACT_DISTANCE;
  request_.link_names = link_names_;

  updateBulletObjects();

  getActiveLinkNamesRecursive(active_link_names_, urdf_model_->getRoot(), false);

  return initialized_;
}

void BulletEnvSingleton::setContactRequest(const ContactRequestBase &req)
{
  request_ = req;
  updateBulletObjects();
}

void BulletEnvSingleton::calcDistancesDiscrete(ContactResultVector &dists)
{
  BulletDistanceMap res;
  BulletDistanceData collisions(&request_, &res);

  for (const auto& obj : active_objects_)
  {
    const COWPtr& cow = manager_.m_link2cow[obj];
    assert(cow);

    manager_.contactDiscreteTest(cow, collisions);

    if (collisions.done) break;
  }

  dists.reserve(request_.link_names.size());
  for (BulletDistanceMap::iterator pair = res.begin(); pair != res.end(); ++pair)
  {
    for (auto it = pair->second.begin(); it != pair->second.end(); ++it)
    {
      ContactResult d;
      d.distance = it->distance;
      d.valid = true;

      // Note: for trajopt ROSBulletEnvSingleton is only aware of links in the urdf so if attached link set link name to parent link name
      if (it->body_types[0] == BodyType::ROBOT_ATTACHED)
      {
        d.link_names[0] = getAttachedBody(it->link_names[0])->info.parent_link_name;
      }
      else
      {
        d.link_names[0] = it->link_names[0];
      }

      // Note: for trajopt ROSBulletEnvSingleton is only aware of links in the urdf so if attached link set link name to parent link name
      if (it->body_types[1] == BodyType::ROBOT_ATTACHED)
      {
        d.link_names[1] = getAttachedBody(it->link_names[1])->info.parent_link_name;
      }
      else
      {
        d.link_names[1] = it->link_names[1];
      }

      d.nearest_points[0] = it->nearest_points[0];
      d.nearest_points[1] = it->nearest_points[1];
      d.normal = it->normal;

      if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() || (d.normal.array().isNaN()).all())
        d.valid = false;

      dists.push_back(d);
    }
  }
}

void BulletEnvSingleton::calcCollisionsDiscrete(std::vector<ContactResult> &collisions)
{
  calcDistancesDiscrete(collisions);
}

void BulletEnvSingleton::setState(const std::unordered_map<std::string, double> &joints)
{
  current_state_->joints.insert(joints.begin(), joints.end());

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
    {
      current_state_->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());

  updateBulletObjects();
}

void BulletEnvSingleton::setState(const std::vector<std::string> &joint_names, const std::vector<double> &joint_values)
{
  for (auto i = 0; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());

  updateBulletObjects();
}

void BulletEnvSingleton::setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values)
{
  for (auto i = 0; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());

  updateBulletObjects();
}

Eigen::VectorXd BulletEnvSingleton::getCurrentJointValues() const
{
  Eigen::VectorXd jv;
  jv.resize(joint_names_.size());
  for(auto j = 0u; j < joint_names_.size(); ++j)
  {
    jv(j)= current_state_->joints[joint_names_[j]];
  }
  return jv;
}

Eigen::VectorXd BulletEnvSingleton::getCurrentJointValues(const std::string &manipulator_name) const
{
  auto it = manipulators_.find(manipulator_name);
  if (it != manipulators_.end())
  {
    const std::vector<std::string>& joint_names = it->second->getJointNames();
    Eigen::VectorXd start_pos(joint_names.size());

    for(auto j = 0u; j < joint_names.size(); ++j)
    {
      start_pos(j) = current_state_->joints[joint_names[j]];
    }

    return start_pos;
  }

  return Eigen::VectorXd();
}

vector_Affine3d BulletEnvSingleton::getLinkTransforms() const
{
  vector_Affine3d link_tfs;
  link_tfs.resize(link_names_.size());
  for (const auto& link_name : link_names_)
  {
    link_tfs.push_back(current_state_->transforms[link_name]);
  }
  return link_tfs;
}

Eigen::Affine3d BulletEnvSingleton::getLinkTransform(const std::string& link_name) const
{
  return current_state_->transforms[link_name];
}

bool BulletEnvSingleton::addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name)
{
  if (!hasManipulator(manipulator_name))
  {
    KDLChainKinPtr manip(new KDLChainKin());
    manip->init(urdf_model_, base_link, tip_link, manipulator_name);

    manipulators_.insert(std::make_pair(manipulator_name, manip));
    return true;
  }
  return false;
}

bool BulletEnvSingleton::hasManipulator(const std::string &manipulator_name) const
{
  return manipulators_.find(manipulator_name) != manipulators_.end();
}

BasicKinConstPtr BulletEnvSingleton::getManipulator(const std::string &manipulator_name) const
{
  auto it = manipulators_.find(manipulator_name);
  if (it != manipulators_.end())
    return it->second;

  return nullptr;
}

std::string BulletEnvSingleton::getManipulatorName(const std::vector<std::string> &joint_names) const
{
  std::set<std::string> joint_names_set(joint_names.begin(), joint_names.end());
  for (const auto& manip : manipulators_)
  {
    const std::vector<std::string>& tmp_joint_names = manip.second->getJointNames();
    std::set<std::string> tmp_joint_names_set(tmp_joint_names.begin(), tmp_joint_names.end());
    if (joint_names_set == tmp_joint_names_set)
      return manip.first;

  }
  return "";
}

void BulletEnvSingleton::addAttachableObject(const AttachableObjectConstPtr attachable_object)
{
  const auto object = attachable_objects_.find(attachable_object->name);
  if (object != attachable_objects_.end())
    ROS_DEBUG("Replacing attachable object %s!", attachable_object->name.c_str());

  attachable_objects_[attachable_object->name] = attachable_object;
}

void BulletEnvSingleton::removeAttachableObject(const std::string& name)
{
  if (attachable_objects_.find(name) != attachable_objects_.end())
  {
    attachable_objects_.erase(name);
  }
}

void BulletEnvSingleton::clearAttachableObjects()
{
  attachable_objects_.clear();
}

const AttachedBodyConstPtr BulletEnvSingleton::getAttachedBody(const std::string& name) const
{
  const auto body = attached_bodies_.find(name);
  if (body == attached_bodies_.end())
    ROS_ERROR("Tried to get attached body %s which does not exist!", name.c_str());

  return body->second;
}

void BulletEnvSingleton::attachBody(const AttachedBodyInfo &attached_body_info)
{
  const auto body_info = attached_bodies_.find(attached_body_info.name);
  const auto obj = attachable_objects_.find(attached_body_info.object_name);

  if (body_info != attached_bodies_.end())
  {
    ROS_DEBUG("Tried to attached body %s which is already attached!", attached_body_info.name.c_str());
    return;
  }

  if (obj == attachable_objects_.end())
  {
    ROS_DEBUG("Tried to attached body %s with object %s which does not exist!", attached_body_info.name.c_str(), attached_body_info.object_name.c_str());
    return;
  }

  if (std::find(link_names_.begin(), link_names_.end(), attached_body_info.name) != link_names_.end())
  {
    ROS_DEBUG("Tried to attached body %s with the same name as an existing link!", attached_body_info.name.c_str());
    return;
  }

  link_names_.push_back(attached_body_info.name);
  if (std::find(active_link_names_.begin(), active_link_names_.end(), attached_body_info.parent_link_name) != active_link_names_.end())
  {
    active_link_names_.push_back(attached_body_info.name);
  }

  AttachedBodyPtr attached_body(new AttachedBody());
  attached_body->info = attached_body_info;
  attached_body->obj = obj->second;

  attached_bodies_.insert(std::make_pair(attached_body_info.name, attached_body));
  auto it = attached_bodies_.find(attached_body_info.name);
  COWPtr new_cow(new COW(it->second.get()));
  if (new_cow)
  {
    setContactDistance(new_cow, BULLET_DEFAULT_CONTACT_DISTANCE);
    manager_.m_link2cow[new_cow->getID()] = new_cow;
    manager_.addCollisionObject(new_cow);

    updateBulletObjects();
    ROS_DEBUG("Added collision object for attached body %s", attached_body_info.name.c_str());
  }
  else
  {
    ROS_ERROR("Error creating attached body %s", attached_body_info.name.c_str());
  }
}

void BulletEnvSingleton::detachBody(const std::string &name)
{
  if (attached_bodies_.find(name) != attached_bodies_.end())
  {
    manager_.removeCollisionObject(manager_.m_link2cow[name]);
    attached_bodies_.erase(name);
    manager_.m_link2cow.erase(name);
    link_names_.erase(std::remove(link_names_.begin(), link_names_.end(), name), link_names_.end());
    active_link_names_.erase(std::remove(active_link_names_.begin(), active_link_names_.end(), name), active_link_names_.end());

    updateBulletObjects();
  }
}

void BulletEnvSingleton::clearAttachedBodies()
{
  for (const auto& body : attached_bodies_)
  {
    std::string name = body.second->info.name;
    manager_.removeCollisionObject(manager_.m_link2cow[name]);
    manager_.m_link2cow.erase(name);
    link_names_.erase(std::remove(link_names_.begin(), link_names_.end(), name), link_names_.end());
    active_link_names_.erase(std::remove(active_link_names_.begin(), active_link_names_.end(), name), active_link_names_.end());
  }
  attached_bodies_.clear();
  updateBulletObjects();
}

bool BulletEnvSingleton::setJointValuesHelper(KDL::JntArray &q, const std::string &joint_name, const double &joint_value) const
{
  auto qnr = joint_to_qnr_.find(joint_name);
  if (qnr != joint_to_qnr_.end())
  {
    q(qnr->second) = joint_value;
    return true;
  }
  else
  {
    ROS_ERROR("Tried to set joint name %s which does not exist!", joint_name.c_str());
    return false;
  }
}

void BulletEnvSingleton::calculateTransformsHelper(std::unordered_map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const
{
  if (it != kdl_tree_->getSegments().end())
  {
    const KDL::TreeElementType& current_element = it->second;
    KDL::Frame current_frame = GetTreeElementSegment(current_element).pose(q_in(GetTreeElementQNr(current_element)));

    Eigen::Affine3d local_frame, global_frame;
    KDLChainKin::KDLToEigen(current_frame, local_frame);
    global_frame =  parent_frame * local_frame;
    transforms[current_element.segment.getName()] = global_frame;

    for (auto& child: current_element.children)
    {
      calculateTransformsHelper(transforms, q_in, child, global_frame);
    }
  }
}

void BulletEnvSingleton::calculateTransforms(std::unordered_map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const
{
  calculateTransformsHelper(transforms, q_in, it, parent_frame);

  // update attached objects location
  for (const auto& attached : attached_bodies_)
  {
    transforms[attached.first] = transforms[attached.second->info.parent_link_name];
  }

  // update collision object transforms
  for (std::pair<std::string, COWPtr> element : manager_.m_link2cow)
  {
    element.second->setWorldTransform(convertEigenToBt(transforms.find(element.first)->second));
  }
}

void BulletEnvSingleton::updateBulletObjects()
{
  active_objects_.clear();

  for (std::pair<std::string, COWPtr> element : manager_.m_link2cow)
  {
    // For descrete checks we can check static to kinematic and kinematic to kinematic
    element.second->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!request_.link_names.empty())
    {
      bool check1 = (std::find_if(request_.link_names.begin(), request_.link_names.end(), [&](std::string link) { return link == element.first; }) == request_.link_names.end());
      bool check2 = (element.second->m_type == BodyType::ROBOT_ATTACHED) ? (std::find_if(request_.link_names.begin(), request_.link_names.end(), [&](std::string link) { return link == element.second->ptr.m_ab->info.parent_link_name; }) == request_.link_names.end()) : true;
      if (check1 && check2)
      {
        element.second->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (element.second->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      element.second->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects_.push_back(element.first);
      element.second->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter;
    }

    setContactDistance(element.second, request_.contact_distance);
  }
}
}
}
