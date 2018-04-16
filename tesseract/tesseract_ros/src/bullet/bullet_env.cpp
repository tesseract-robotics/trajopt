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
#include "tesseract_ros/bullet/bullet_env.h"
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

bool BulletEnv::init(const urdf::ModelInterfaceConstSharedPtr urdf_model)
{
  init(urdf_model, nullptr);
}

bool BulletEnv::init(const urdf::ModelInterfaceConstSharedPtr urdf_model, const srdf::ModelConstSharedPtr srdf_model)
{
  ros::NodeHandle nh;
  initialized_ = false;
  model_ = urdf_model;

  if(model_ == nullptr)
  {
    ROS_ERROR_STREAM("Null pointer to URDF Model");
    return initialized_;
  }

  if (!model_->getRoot())
  {
    ROS_ERROR("Invalid URDF in ROSBulletEnv::init call");
    return initialized_;
  }

  KDL::Tree *kdl_tree = new KDL::Tree();
  if (!kdl_parser::treeFromUrdfModel(*model_, *kdl_tree))
  {
    ROS_ERROR("Failed to initialize KDL from URDF model");
    return initialized_;
  }
  kdl_tree_ = boost::shared_ptr<KDL::Tree>(kdl_tree);
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
          link2cow_[new_cow->getID()] = new_cow;
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
        manip->init(model_, chain.first, chain.second, group.name_);
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

  return initialized_;
}

void BulletEnv::calcDistancesDiscrete(const ContactRequest &req, ContactResultVector &dists) const
{
  BulletManager manager;
  BulletDistanceMap res;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> active_objects;

  EnvStateConstPtr state = getState(req.joint_names, req.joint_angles1);
  constructBulletObject(manager.m_link2cow, active_objects, req.contact_distance, state, req.link_names);

  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactDiscreteTest(cow, collisions);

    if (collisions.done) break;
  }

  dists.reserve(req.link_names.size());
  for (BulletDistanceMap::iterator pair = res.begin(); pair != res.end(); ++pair)
  {
    for (auto it = pair->second.begin(); it != pair->second.end(); ++it)
    {
      ContactResult d;
      d.distance = it->distance;
      d.valid = true;

      // Note: for trajopt ROSBulletEnv is only aware of links in the urdf so if attached link set link name to parent link name
      if (it->body_types[0] == BodyType::ROBOT_ATTACHED)
      {
        d.link_names[0] = getAttachedBody(it->link_names[0])->info.parent_link_name;
      }
      else
      {
        d.link_names[0] = it->link_names[0];
      }

      // Note: for trajopt ROSBulletEnv is only aware of links in the urdf so if attached link set link name to parent link name
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

void BulletEnv::calcDistancesContinuous(const ContactRequest &req, ContactResultVector &dists) const
{
  BulletManager manager;
  BulletDistanceMap res;
  BulletDistanceData collisions(&req, &res);

  std::vector<std::string> active_objects;

  EnvStateConstPtr state1 = getState(req.joint_names, req.joint_angles1);
  EnvStateConstPtr state2 = getState(req.joint_names, req.joint_angles2);

  constructBulletObject(manager.m_link2cow, active_objects, req.contact_distance, state1, state2, req.link_names);
  manager.processCollisionObjects();

  for (auto& obj : active_objects)
  {
    COWPtr cow = manager.m_link2cow[obj];
    assert(cow);

    manager.contactCastTest(cow, collisions);

    if (collisions.done) break;
  }

  dists.reserve(req.link_names.size());
  for (BulletDistanceMap::iterator pair = res.begin(); pair != res.end(); ++pair)
  {
    for (auto it = pair->second.begin(); it != pair->second.end(); ++it)
    {
      ContactResult d;
      d.distance = it->distance;
      d.valid = true;
      d.body_types[0] = it->body_types[0];
      d.body_types[1] = it->body_types[1];
      d.link_names[0] = it->link_names[0];
      d.link_names[1] = it->link_names[1];

      // Note: The kinematic library is only aware of links in the urdf so if attached link set the attached link name
      if (d.body_types[0] == BodyType::ROBOT_ATTACHED)
        d.attached_link_names[0] = getAttachedBody(d.link_names[0])->info.parent_link_name;

      // Note: The kinematic library is only aware of links in the urdf so if attached link set the attached link name
      if (d.body_types[1] == BodyType::ROBOT_ATTACHED)
        d.attached_link_names[1] = getAttachedBody(d.link_names[1])->info.parent_link_name;

      d.nearest_points[0] = it->nearest_points[0];
      d.nearest_points[1] = it->nearest_points[1];
      d.normal = it->normal;
      d.cc_type = it->cc_type;
      d.cc_nearest_points[0] = it->cc_nearest_points[0];
      d.cc_nearest_points[1] = it->cc_nearest_points[1];
      d.cc_time = it->cc_time;

      if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() || (d.normal.array().isNaN()).all())
        d.valid = false;

      if (d.cc_type != ContinouseCollisionType::CCType_None && ((d.cc_nearest_points[0].array().isNaN()).all() || (d.cc_nearest_points[1].array().isNaN()).all()))
        d.valid = false;

      dists.push_back(d);
    }
  }
}

//std::set<urdf::LinkConstSharedPtr> ROSBulletEnv::getLinkModels(const std::vector<std::string> &link_names) const
//{
//  std::set<urdf::LinkConstSharedPtr> list;

//  for(auto const& link_name: link_names)
//  {
//    list.insert(model_->getLink(link_name));
//  }

//  return list;
//}

void BulletEnv::calcCollisionsDiscrete(const ContactRequest &req, std::vector<ContactResult> &collisions) const
{
  calcDistancesDiscrete(req, collisions);
}

void BulletEnv::calcCollisionsContinuous(const ContactRequest &req, std::vector<ContactResult> &collisions) const
{
  calcDistancesContinuous(req, collisions);
}

bool BulletEnv::continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray &traj, ContactResult& collision) const
{
  ContactRequest req;
  req.type = ContactRequestType::SINGLE;
  req.joint_names = joint_names;
  req.link_names = link_names;
  req.acm = getAllowedCollisionMatrix();

  ContactResultVector collisions;
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    req.joint_angles1 = traj.row(iStep);
    req.joint_angles2 = traj.row(iStep + 1);
    calcCollisionsContinuous(req, collisions);
    if (collisions.size() > 0)
    {
      collision = collisions.front();
      return true;
    }
  }
  return false;
}

bool BulletEnv::continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, ContactResultVector& collisions) const
{
  ContactRequest req;
  req.type = ContactRequestType::ALL;
  req.joint_names = joint_names;
  req.link_names = link_names;
  req.acm = getAllowedCollisionMatrix();

  bool found = false;
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    req.joint_angles1 = traj.row(iStep);
    req.joint_angles2 = traj.row(iStep + 1);
    calcCollisionsContinuous(req, collisions);
    if (collisions.size() > 0)
    {
      found = true;
    }
  }
  return found;
}

void BulletEnv::setState(const std::unordered_map<std::string, double> &joints)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  current_state_->joints.insert(joints.begin(), joints.end());

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint.first, joint.second))
    {
      current_state_->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());
}

void BulletEnv::setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  for (auto i = 0; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(kdl_jnt_array_, joint_names[i], joint_values[i]))
    {
      current_state_->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(current_state_->transforms, kdl_jnt_array_, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());
}

EnvStatePtr BulletEnv::getState(const std::unordered_map<std::string, double> &joints) const
{
  EnvStatePtr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto& joint : joints)
  {
    if (setJointValuesHelper(jnt_array, joint.first, joint.second))
    {
      state->joints[joint.first] = joint.second;
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());

  return state;
}

EnvStatePtr BulletEnv::getState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values) const
{
  EnvStatePtr state(new EnvState(*current_state_));
  KDL::JntArray jnt_array = kdl_jnt_array_;

  for (auto i = 0; i < joint_names.size(); ++i)
  {
    if (setJointValuesHelper(jnt_array, joint_names[i], joint_values[i]))
    {
      state->joints[joint_names[i]] = joint_values[i];
    }
  }

  calculateTransforms(state->transforms, jnt_array, kdl_tree_->getRootSegment(), Eigen::Affine3d::Identity());

  return state;
}

Eigen::VectorXd BulletEnv::getCurrentJointValues() const
{
  Eigen::VectorXd jv;
  jv.resize(joint_names_.size());
  for(auto j = 0u; j < joint_names_.size(); ++j)
  {
    jv(j)= current_state_->joints[joint_names_[j]];
  }
  return jv;
}

Eigen::VectorXd BulletEnv::getCurrentJointValues(const std::string &manipulator_name) const
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

vector_Affine3d BulletEnv::getLinkTransforms() const
{
  vector_Affine3d link_tfs;
  link_tfs.resize(link_names_.size());
  for (const auto& link_name : link_names_)
  {
    link_tfs.push_back(current_state_->transforms[link_name]);
  }
  return link_tfs;
}

Eigen::Affine3d BulletEnv::getLinkTransform(const std::string& link_name) const
{
  return current_state_->transforms[link_name];
}

bool BulletEnv::addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  if (!hasManipulator(manipulator_name))
  {
    KDLChainKinPtr manip(new KDLChainKin());
    manip->init(model_, base_link, tip_link, manipulator_name);

    manipulators_.insert(std::make_pair(manipulator_name, manip));
    return true;
  }
  return false;
}

bool BulletEnv::hasManipulator(const std::string &manipulator_name) const
{
  return manipulators_.find(manipulator_name) != manipulators_.end();
}

BasicKinConstPtr BulletEnv::getManipulator(const std::string &manipulator_name) const
{
  auto it = manipulators_.find(manipulator_name);
  if (it != manipulators_.end())
    return it->second;

  return nullptr;
}

std::string BulletEnv::getManipulatorName(const std::vector<std::string> &joint_names) const
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

void BulletEnv::addAttachableObject(const AttachableObjectConstPtr attachable_object)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  const auto object = attachable_objects_.find(attachable_object->name);
  if (object != attachable_objects_.end())
    ROS_DEBUG("Replacing attachable object %s!", attachable_object->name.c_str());

  attachable_objects_[attachable_object->name] = attachable_object;
}

void BulletEnv::removeAttachableObject(const std::string& name)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  if (attachable_objects_.find(name) != attachable_objects_.end())
  {
    attachable_objects_.erase(name);
  }
}

void BulletEnv::clearAttachableObjects()
{
  boost::mutex::scoped_lock(modify_env_mutex_);
  attachable_objects_.clear();
}

const AttachedBodyConstPtr BulletEnv::getAttachedBody(const std::string& name) const
{
  const auto body = attached_bodies_.find(name);
  if (body == attached_bodies_.end())
    ROS_ERROR("Tried to get attached body %s which does not exist!", name.c_str());

  return body->second;
}

void BulletEnv::attachBody(const AttachedBodyInfo &attached_body_info)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

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
  AttachedBodyPtr attached_body(new AttachedBody());
  attached_body->info = attached_body_info;
  attached_body->obj = obj->second;

  attached_bodies_.insert(std::make_pair(attached_body_info.name, attached_body));
  auto it = attached_bodies_.find(attached_body_info.name);
  COWPtr new_cow(new COW(it->second.get()));
  if (new_cow)
  {
    setContactDistance(new_cow, BULLET_DEFAULT_CONTACT_DISTANCE);
    link2cow_[new_cow->getID()] = new_cow;
    ROS_DEBUG("Added collision object for attached body %s", attached_body_info.name.c_str());
  }
  else
  {
    ROS_ERROR("Error creating attached body %s", attached_body_info.name.c_str());
  }
}

void BulletEnv::detachBody(const std::string &name)
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  if (attached_bodies_.find(name) != attached_bodies_.end())
  {
    attached_bodies_.erase(name);
    link2cow_.erase(name);
    link_names_.erase(std::remove(link_names_.begin(), link_names_.end(), name), link_names_.end());
  }
}

void BulletEnv::clearAttachedBodies()
{
  boost::mutex::scoped_lock(modify_env_mutex_);

  for (const auto& body : attached_bodies_)
  {
    std::string name = body.second->info.name;
    link2cow_.erase(name);
    link_names_.erase(std::remove(link_names_.begin(), link_names_.end(), name), link_names_.end());
  }
  attached_bodies_.clear();
}

bool BulletEnv::setJointValuesHelper(KDL::JntArray &q, const std::string &joint_name, const double &joint_value) const
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

void BulletEnv::calculateTransformsHelper(std::unordered_map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const
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

void BulletEnv::calculateTransforms(std::unordered_map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const
{
  calculateTransformsHelper(transforms, q_in, it, parent_frame);

  // update attached objects location
  for (const auto& attached : attached_bodies_)
  {
    transforms[attached.first] = transforms[attached.second->info.parent_link_name];
  }
}

void BulletEnv::constructBulletObject(Link2Cow &collision_objects, std::vector<std::string> &active_objects, double contact_distance, const EnvStateConstPtr state, const std::vector<std::string> &active_links, bool continuous) const
{

  for (std::pair<std::string, COWConstPtr> element : link2cow_)
  {
    COWPtr new_cow(new COW(*(element.second.get())));
    assert(new_cow->getCollisionShape());

    new_cow->setWorldTransform(convertEigenToBt(state->transforms.find(element.first)->second));

    // For descrete checks we can check static to kinematic and kinematic to kinematic
    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!active_links.empty())
    {
      bool check1 = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) { return link == element.first; }) == active_links.end());
      bool check2 = (element.second->m_type == BodyType::ROBOT_ATTACHED) ? (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) { return link == element.second->ptr.m_ab->info.parent_link_name; }) == active_links.end()) : true;
      if (check1 && check2)
      {
        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(element.first);
      (continuous) ? (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter) : (new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter | btBroadphaseProxy::KinematicFilter);
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
  }
}

void BulletEnv::constructBulletObject(Link2Cow& collision_objects,
                                         std::vector<std::string> &active_objects,
                                         double contact_distance,
                                         const EnvStateConstPtr state1,
                                         const EnvStateConstPtr state2,
                                         const std::vector<std::string> &active_links) const
{
  for (std::pair<std::string, COWConstPtr> element : link2cow_)
  {
    COWPtr new_cow(new COW(*(element.second.get())));

    new_cow->m_collisionFilterGroup = btBroadphaseProxy::KinematicFilter;
    if (!active_links.empty())
    {
      bool check1 = (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) { return link == element.first; }) == active_links.end());
      bool check2 = (element.second->m_type == BodyType::ROBOT_ATTACHED) ? (std::find_if(active_links.begin(), active_links.end(), [&](std::string link) { return link == element.second->ptr.m_ab->info.parent_link_name; }) == active_links.end()) : true;
      if (check1 && check2)
      {
        new_cow->m_collisionFilterGroup = btBroadphaseProxy::StaticFilter;
      }
    }

    if (new_cow->m_collisionFilterGroup == btBroadphaseProxy::StaticFilter)
    {
      new_cow->setWorldTransform(convertEigenToBt(state1->transforms.find(element.first)->second));
      new_cow->m_collisionFilterMask = btBroadphaseProxy::KinematicFilter;
    }
    else
    {
      active_objects.push_back(element.first);

      if (btBroadphaseProxy::isConvex(new_cow->getCollisionShape()->getShapeType()))
      {
        btConvexShape* convex = static_cast<btConvexShape*>(new_cow->getCollisionShape());
        assert(convex != NULL);

        btTransform tf1 = convertEigenToBt(state1->transforms.find(element.first)->second);
        btTransform tf2 = convertEigenToBt(state2->transforms.find(element.first)->second);

        CastHullShape* shape = new CastHullShape(convex, tf1.inverseTimes(tf2));
        assert(shape != NULL);

        new_cow->manage(shape);
        new_cow->setCollisionShape(shape);
        new_cow->setWorldTransform(tf1);
      }
      else if (btBroadphaseProxy::isCompound(new_cow->getCollisionShape()->getShapeType()))
      {
        btCompoundShape* compound = static_cast<btCompoundShape*>(new_cow->getCollisionShape());
        const Eigen::Affine3d &tf1 = state1->transforms.find(element.first)->second;
        const Eigen::Affine3d &tf2 = state2->transforms.find(element.first)->second;

        btCompoundShape* new_compound = new btCompoundShape(/*dynamicAABBtree=*/false);

        for (int i = 0; i < compound->getNumChildShapes(); ++i)
        {
          btConvexShape* convex = static_cast<btConvexShape*>(compound->getChildShape(i));
          assert(convex != NULL);

          btTransform geomTrans = compound->getChildTransform(i);
          btTransform child_tf1 = convertEigenToBt(tf1) * geomTrans;
          btTransform child_tf2 = convertEigenToBt(tf2) * geomTrans;

          btCollisionShape* subshape = new CastHullShape(convex, child_tf1.inverseTimes(child_tf2));
          assert(subshape != NULL);

          if (subshape != NULL)
          {
            new_cow->manage(subshape);
            subshape->setMargin(BULLET_MARGIN);
            new_compound->addChildShape(geomTrans, subshape);
          }
        }

        new_compound->setMargin(BULLET_MARGIN); //margin: compound. seems to have no effect when positive but has an effect when negative
        new_cow->manage(new_compound);
        new_cow->setCollisionShape(new_compound);
        new_cow->setWorldTransform(convertEigenToBt(tf1));
      }
      else
      {
        ROS_ERROR("I can only continuous collision check convex shapes and compound shapes made of convex shapes");
      }

      new_cow->m_collisionFilterMask = btBroadphaseProxy::StaticFilter;
    }

    setContactDistance(new_cow, contact_distance);
    collision_objects[element.first] = new_cow;
  }
}

//moveit_msgs::RobotStatePtr BulletEnv::getRobotStateMsg() const
//{
//  moveit_msgs::RobotStatePtr msg(new moveit_msgs::RobotState());
//  msg->is_diff = false;
//  msg->joint_state.name.reserve(current_state_->joints.size());
//  msg->joint_state.position.reserve(current_state_->joints.size());
//  for (const auto& joint : current_state_->joints)
//  {
//    msg->joint_state.name.push_back(joint.first);
//    msg->joint_state.position.push_back(joint.second);
//  }

//  for (const auto& body : attached_bodies_)
//  {
//    moveit_msgs::AttachedCollisionObject obj;
//    obj.link_name = body.second->info.parent_link_name;
//    obj.touch_links = body.second->info.touch_links;

//    obj.object.id = body.second->obj->name;
//    obj.object.header.frame_id = body.second->info.parent_link_name;
//    obj.object.header.stamp = ros::Time::now();

//    for (auto i = 0; i < body.second->obj->shapes.size(); ++i)
//    {
//      const auto geom = body.second->obj->shapes[i];
//      const auto geom_pose = body.second->obj->shapes_trans[i];
//      if (geom->type == shapes::OCTREE)
//      {
//        const shapes::OcTree* g = static_cast<const shapes::OcTree*>(geom.get());
//        double occupancy_threshold = g->octree->getOccupancyThres();

//        for(auto it = g->octree->begin(g->octree->getTreeDepth()), end = g->octree->end(); it != end; ++it)
//        {
//          if(it->getOccupancy() >= occupancy_threshold)
//          {
//            double size = it.getSize();
//            shape_msgs::SolidPrimitive s;
//            s.type = shape_msgs::SolidPrimitive::BOX;
//            s.dimensions.resize(3);
//            s.dimensions[shape_msgs::SolidPrimitive::BOX_X] = size;
//            s.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = size;
//            s.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = size;
//            obj.object.primitives.push_back(s);

//            Eigen::Affine3d trans, final_trans;
//            trans.setIdentity();
//            trans.translation() = Eigen::Vector3d(it.getX(), it.getY(), it.getZ());
//            final_trans = geom_pose * trans;

//            geometry_msgs::Pose pose;
//            tf::poseEigenToMsg(final_trans, pose);
//            obj.object.primitive_poses.push_back(pose);
//          }
//        }
//      }
//      else if (geom->type == shapes::MESH)
//      {
//        shapes::ShapeMsg s;
//        shapes::constructMsgFromShape(geom.get(), s);

//        obj.object.meshes.push_back(boost::get<shape_msgs::Mesh>(s));

//        geometry_msgs::Pose pose;
//        tf::poseEigenToMsg(geom_pose, pose);
//        obj.object.mesh_poses.push_back(pose);
//      }
//      else if (geom->type == shapes::PLANE)
//      {
//        shapes::ShapeMsg s;
//        shapes::constructMsgFromShape(geom.get(), s);
//        obj.object.planes.push_back(boost::get<shape_msgs::Plane>(s));

//        geometry_msgs::Pose pose;
//        tf::poseEigenToMsg(geom_pose, pose);
//        obj.object.plane_poses.push_back(pose);
//      }
//      else //SolidPrimitive
//      {
//        shapes::ShapeMsg s;
//        shapes::constructMsgFromShape(geom.get(), s);
//        obj.object.primitives.push_back(boost::get<shape_msgs::SolidPrimitive>(s));

//        geometry_msgs::Pose pose;
//        tf::poseEigenToMsg(geom_pose, pose);
//        obj.object.primitive_poses.push_back(pose);
//      }
//    }
//    msg->attached_collision_objects.push_back(obj);
//  }
//  return msg;
//}

}
}
