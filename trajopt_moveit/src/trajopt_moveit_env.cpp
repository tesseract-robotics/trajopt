#include "trajopt_moveit/trajopt_moveit_env.h"
#include <iostream>
#include <limits>
#include <moveit/collision_detection/collision_common.h>
#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <tesseract_ros/ros_tesseract_utils.h>

namespace trajopt_moveit
{
using Eigen::MatrixXd;
using Eigen::VectorXd;

bool TrajOptMoveItEnv::init(planning_scene::PlanningScenePtr planning_scene)
{
  env_ = planning_scene;
  initialized_ = planning_scene ? true : false;

  if (initialized_)
  {
    collision_robot_ = planning_scene->getCollisionRobot();
    collision_world_ = planning_scene->getCollisionWorld();
  }

  // Now get the active link names
  tesseract::tesseract_ros::getActiveLinkNamesRecursive(
      urdf_active_link_names_, env_->getRobotModel()->getURDF()->getRoot(), false);

  return initialized_;
}

void TrajOptMoveItEnv::calcDistancesDiscrete(const tesseract::ContactRequest& req,
                                             tesseract::ContactResultVector& dists) const
{
  collision_detection::DistanceRequest distance_req;
  collision_detection::DistanceResult distance_res;

  distance_req.group_name = getManipulatorName(req.joint_names);
  distance_req.enable_nearest_points = true;
  distance_req.enable_signed_distance = true;

  if (req.type == tesseract::ContactRequestType::SINGLE)
    distance_req.type = collision_detection::DistanceRequestType::SINGLE;
  else
    distance_req.type = collision_detection::DistanceRequestType::ALL;

  std::set<const moveit::core::LinkModel*> list = getLinkModels(req.link_names);
  distance_req.active_components_only = &list;
  distance_req.distance_threshold = req.contact_distance;
  distance_req.acm = &env_->getAllowedCollisionMatrix();
  distance_req.max_contacts_per_body = 50;

  robot_state::RobotState state = env_->getCurrentState();
  int i = 0;
  for (auto const& joint_name : req.joint_names)
  {
    state.setVariablePosition(joint_name, req.joint_angles1(i));
    ++i;
  }
  state.update();

  collision_robot_->distanceSelf(distance_req, distance_res, state);
  collision_world_->distanceRobot(distance_req, distance_res, *collision_robot_, state);

  dists.reserve(req.link_names.size());
  for (collision_detection::DistanceMap::iterator pair = distance_res.distances.begin();
       pair != distance_res.distances.end();
       ++pair)
  {
    for (auto it = pair->second.begin(); it != pair->second.end(); ++it)
    {
      tesseract::ContactResult d;
      d.distance = it->distance;
      d.valid = true;
      d.link_names[0] = it->link_names[0];
      d.link_names[1] = it->link_names[1];
      d.body_types[0] = tesseract::BodyTypes::ROBOT_LINK;
      d.body_types[1] = tesseract::BodyTypes::ROBOT_LINK;

      // Note: for trajopt TrajOptMoveItEnv is only aware of links in the urdf
      // so if attached link set link name to parent link name
      if (it->body_types[0] == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        d.body_types[0] = tesseract::BodyTypes::ROBOT_ATTACHED;
        d.attached_link_names[0] = state.getAttachedBody(d.link_names[0])->getAttachedLinkName();
      }
      else if (it->body_types[0] == collision_detection::BodyTypes::WORLD_OBJECT)
      {
        d.body_types[0] = tesseract::BodyTypes::ROBOT_ATTACHED;
        d.attached_link_names[0] = env_->getPlanningFrame();
      }

      // Note: for trajopt TrajOptMoveItEnv is only aware of links in the urdf
      // so if attached link set link name to parent link name
      if (it->body_types[1] == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        d.body_types[1] = tesseract::BodyTypes::ROBOT_ATTACHED;
        d.attached_link_names[1] = state.getAttachedBody(d.link_names[1])->getAttachedLinkName();
      }
      else if (it->body_types[1] == collision_detection::BodyTypes::WORLD_OBJECT)
      {
        d.body_types[1] = tesseract::BodyTypes::ROBOT_ATTACHED;
        d.attached_link_names[1] = env_->getPlanningFrame();
      }

      d.nearest_points[0] = it->nearest_points[0];
      d.nearest_points[1] = it->nearest_points[1];
      d.normal = it->normal;

      if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() ||
          (d.normal.array().isNaN()).all())
        d.valid = false;

      dists.push_back(d);
    }
  }
}

void TrajOptMoveItEnv::calcDistancesContinuous(const tesseract::ContactRequest& req,
                                               tesseract::ContactResultVector& dists) const
{
  collision_detection::DistanceRequest distance_req;
  collision_detection::DistanceResult distance_res;

  distance_req.group_name = getManipulatorName(req.joint_names);
  distance_req.enable_nearest_points = true;
  distance_req.enable_signed_distance = true;
  distance_req.compute_gradient = true;

  if (req.type == tesseract::ContactRequestType::SINGLE)
    distance_req.type = collision_detection::DistanceRequestType::SINGLE;
  else
    distance_req.type = collision_detection::DistanceRequestType::ALL;

  std::set<const moveit::core::LinkModel*> list = getLinkModels(req.link_names);
  distance_req.active_components_only = &list;
  distance_req.distance_threshold = req.contact_distance;
  distance_req.acm = &env_->getAllowedCollisionMatrix();
  distance_req.max_contacts_per_body = 50;

  robot_state::RobotState state1 = env_->getCurrentState();
  robot_state::RobotState state2 = env_->getCurrentState();
  int i = 0;
  for (auto const& joint_name : req.joint_names)
  {
    state1.setVariablePosition(joint_name, req.joint_angles1(i));
    state2.setVariablePosition(joint_name, req.joint_angles2(i));
    ++i;
  }
  state1.update();
  state2.update();

  collision_robot_->distanceSelf(distance_req, distance_res, state1, state2);
  collision_world_->distanceRobot(distance_req, distance_res, *collision_robot_, state1, state2);

  dists.reserve(req.link_names.size());
  for (collision_detection::DistanceMap::iterator pair = distance_res.distances.begin();
       pair != distance_res.distances.end();
       ++pair)
  {
    for (auto it = pair->second.begin(); it != pair->second.end(); ++it)
    {
      tesseract::ContactResult d;
      d.distance = it->distance;
      d.valid = true;

      // Note: for trajopt TrajOptMoveItEnv is only aware of links in the urdf
      // so if attached link set link name to parent link name
      if (it->body_types[0] == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        d.link_names[0] = state1.getAttachedBody(it->link_names[0])->getAttachedLinkName();
      }
      else
      {
        d.link_names[0] = it->link_names[0];
      }

      // Note: for trajopt TrajOptMoveItEnv is only aware of links in the urdf
      // so if attached link set link name to parent link name
      if (it->body_types[1] == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        d.link_names[1] = state1.getAttachedBody(it->link_names[1])->getAttachedLinkName();
      }
      else
      {
        d.link_names[1] = it->link_names[1];
      }

      d.nearest_points[0] = it->nearest_points[0];
      d.nearest_points[1] = it->nearest_points[1];
      d.normal = it->normal;

      switch (it->cc_type)
      {
        case collision_detection::ContinouseCollisionType::CCType_Between:
          d.cc_type = tesseract::ContinouseCollisionType::CCType_Between;
          break;
        case collision_detection::ContinouseCollisionType::CCType_Time0:
          d.cc_type = tesseract::ContinouseCollisionType::CCType_Time0;
          break;
        case collision_detection::ContinouseCollisionType::CCType_Time1:
          d.cc_type = tesseract::ContinouseCollisionType::CCType_Time1;
          break;
        default:
          d.cc_type = tesseract::ContinouseCollisionType::CCType_None;
          break;
      }

      d.cc_nearest_points[0] = it->cc_nearest_points[0];
      d.cc_nearest_points[1] = it->cc_nearest_points[1];
      d.cc_time = it->cc_time;

      if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() ||
          (d.normal.array().isNaN()).all())
        d.valid = false;

      if (d.cc_type != tesseract::ContinouseCollisionType::CCType_None &&
          ((d.cc_nearest_points[0].array().isNaN()).all() || (d.cc_nearest_points[1].array().isNaN()).all()))
        d.valid = false;

      dists.push_back(d);
    }
  }
}

std::set<const moveit::core::LinkModel*>
TrajOptMoveItEnv::getLinkModels(const std::vector<std::string>& link_names) const
{
  std::set<const moveit::core::LinkModel*> list;

  for (auto const& link_name : link_names)
  {
    list.insert(env_->getRobotModel()->getLinkModel(link_name));
  }

  return list;
}

void TrajOptMoveItEnv::calcCollisionsDiscrete(const tesseract::ContactRequest& req,
                                              tesseract::ContactResultVector& collisions) const
{
  calcDistancesDiscrete(req, collisions);
}

void TrajOptMoveItEnv::calcCollisionsContinuous(const tesseract::ContactRequest& req,
                                                tesseract::ContactResultVector& collisions) const
{
  calcDistancesContinuous(req, collisions);
}

bool TrajOptMoveItEnv::continuousCollisionCheckTrajectory(const std::vector<std::string>& joint_names,
                                                          const std::vector<std::string>& link_names,
                                                          const tesseract::TrajArray& traj,
                                                          tesseract::ContactResultVector& collisions) const
{
  tesseract::ContactRequest req;
  req.joint_names = joint_names;
  req.link_names = link_names;

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

bool TrajOptMoveItEnv::continuousCollisionCheckTrajectory(const std::vector<std::string>& joint_names,
                                                          const std::vector<std::string>& link_names,
                                                          const tesseract::TrajArray& traj,
                                                          tesseract::ContactResult& collision) const
{
  tesseract::ContactRequest req;
  req.joint_names = joint_names;
  req.link_names = link_names;
  tesseract::ContactResultVector collisions;

  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    req.joint_angles1 = traj.row(iStep);
    req.joint_angles2 = traj.row(iStep + 1);
    collision_detection::CollisionRequest collision_req;
    collision_detection::CollisionResult collisione_res;

    collision_req.group_name = getManipulatorName(req.joint_names);

    robot_state::RobotState state1 = env_->getCurrentState();
    robot_state::RobotState state2 = env_->getCurrentState();
    int i = 0;
    for (auto const& joint_name : req.joint_names)
    {
      state1.setVariablePosition(joint_name, req.joint_angles1(i));
      state2.setVariablePosition(joint_name, req.joint_angles2(i));
      ++i;
    }
    state1.update();
    state2.update();

    collision_robot_->checkSelfCollision(
        collision_req, collisione_res, state1, state2, env_->getAllowedCollisionMatrix());
    collision_world_->checkRobotCollision(
        collision_req, collisione_res, *collision_robot_, state1, state2, env_->getAllowedCollisionMatrix());

    if (collisione_res.contacts.begin() != collisione_res.contacts.end())
    {
      collision_detection::Contact& contact = collisione_res.contacts.begin()->second[0];
      collision.distance = contact.depth;
      collision.valid = true;

      // Note: for trajopt TrajOptMoveItEnv is only aware of links in the urdf
      // so if attached link set link name to parent link name
      if (contact.body_type_1 == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        collision.link_names[0] = state1.getAttachedBody(contact.body_name_1)->getAttachedLinkName();
      }
      else
      {
        collision.link_names[0] = contact.body_name_1;
      }

      // Note: for trajopt TrajOptMoveItEnv is only aware of links in the urdf
      // so if attached link set link name to parent link name
      if (contact.body_type_2 == collision_detection::BodyTypes::ROBOT_ATTACHED)
      {
        collision.link_names[1] = state1.getAttachedBody(contact.body_name_2)->getAttachedLinkName();
      }
      else
      {
        collision.link_names[1] = contact.body_name_2;
      }

      collision.nearest_points[0] = contact.pos;
      collision.nearest_points[1] = contact.pos;
      collision.normal = contact.normal;

      //    collision.cc_type = it->second.cc_type;
      collision.cc_nearest_points[0] = contact.pos;
      collision.cc_nearest_points[1] = contact.pos;
      //    collision.cc_time = it->second.cc_time;

      if (collision.cc_type != tesseract::ContinouseCollisionType::CCType_None &&
          ((collision.cc_nearest_points[0].array().isNaN()).all() ||
           (collision.cc_nearest_points[1].array().isNaN()).all()))
        collision.valid = false;

      return true;
    }
  }
  return false;
}

Eigen::VectorXd TrajOptMoveItEnv::getCurrentJointValues() const
{
  const std::vector<std::string>& joint_names = env_->getCurrentState().getVariableNames();
  Eigen::VectorXd start_pos(joint_names.size());

  for (auto j = 0u; j < joint_names.size(); ++j)
  {
    start_pos(j) = env_->getCurrentState().getVariablePosition(joint_names[j]);
  }

  return start_pos;
}

Eigen::VectorXd TrajOptMoveItEnv::getCurrentJointValues(const std::string& manipulator_name) const
{
  std::vector<std::string> joint_names =
      env_->getRobotModel()->getJointModelGroup(manipulator_name)->getActiveJointModelNames();
  Eigen::VectorXd start_pos(joint_names.size());

  for (auto j = 0u; j < joint_names.size(); ++j)
  {
    start_pos(j) = env_->getCurrentState().getVariablePosition(joint_names[j]);
  }

  return start_pos;
}

std::vector<std::string> TrajOptMoveItEnv::getLinkNames() const
{
  std::vector<std::string> object_names;
  const std::vector<std::string>& robot_links = env_->getRobotModel()->getLinkModelNames();
  const std::vector<std::string>& world_links = env_->getWorld()->getObjectIds();

  std::vector<const moveit::core::AttachedBody*> bodies;
  env_->getCurrentState().getAttachedBodies(bodies);

  object_names.reserve(robot_links.size() + world_links.size() + bodies.size());
  object_names.insert(object_names.end(), robot_links.begin(), robot_links.end());
  object_names.insert(object_names.end(), world_links.begin(), world_links.end());
  for (const auto& body : bodies)
  {
    object_names.push_back(body->getName());
  }
  return object_names;
}

std::vector<std::string> TrajOptMoveItEnv::getActiveLinkNames() const
{
  std::vector<std::string> active_link_names = urdf_active_link_names_;

  std::vector<const moveit::core::AttachedBody*> bodies;
  env_->getCurrentState().getAttachedBodies(bodies);

  for (const auto& body : bodies)
  {
    if (std::find(urdf_active_link_names_.begin(), urdf_active_link_names_.end(), body->getAttachedLinkName()) !=
        urdf_active_link_names_.end())
    {
      active_link_names.push_back(body->getName());
    }
  }

  return active_link_names;
}

tesseract::VectorIsometry3d TrajOptMoveItEnv::getLinkTransforms() const
{
  std::vector<std::string> link_names = getLinkNames();
  tesseract::VectorIsometry3d link_tfs;
  link_tfs.resize(link_names.size());
  for (const auto& link_name : link_names)
  {
    link_tfs.push_back(env_->getFrameTransform(link_name));
  }
  return link_tfs;
}

Eigen::Isometry3d TrajOptMoveItEnv::getLinkTransform(const std::string& link_name) const
{
  return env_->getFrameTransform(link_name);
}

bool TrajOptMoveItEnv::hasManipulator(const std::string& manipulator_name) const
{
  return env_->getRobotModel()->hasJointModelGroup(manipulator_name);
}

tesseract::BasicKinConstPtr TrajOptMoveItEnv::getManipulator(const std::string& manipulator_name) const
{
  tesseract::tesseract_ros::KDLChainKinPtr manip(new tesseract::tesseract_ros::KDLChainKin());
  auto jmg = env_->getRobotModel()->getJointModelGroup(manipulator_name);
  manip->init(env_->getRobotModel()->getURDF(),
              jmg->getLinkModels().front()->getParentLinkModel()->getName(),
              jmg->getLinkModels().back()->getName(),
              manipulator_name);
  return manip;
}

std::string TrajOptMoveItEnv::getManipulatorName(const std::vector<std::string>& joint_names) const
{
  const std::vector<const moveit::core::JointModelGroup*>& groups = env_->getRobotModel()->getJointModelGroups();
  std::set<std::string> joint_names_set(joint_names.begin(), joint_names.end());
  for (auto group : groups)
  {
    const std::vector<std::string>& tmp_joint_names = group->getActiveJointModelNames();
    std::set<std::string> tmp_joint_names_set(tmp_joint_names.begin(), tmp_joint_names.end());
    if (joint_names_set == tmp_joint_names_set)
      return group->getName();
  }
  return "";
}
}
