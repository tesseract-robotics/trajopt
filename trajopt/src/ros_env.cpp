#include "trajopt/ros_env.h"
#include <moveit/collision_detection/collision_common.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit/robot_state/conversions.h>
#include <iostream>
#include <limits>

namespace trajopt
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

bool ROSEnv::init(PlanningScenePtr planning_scene)
{
  ros::NodeHandle nh;
  env_ = planning_scene;
  initialized_ = planning_scene? true : false;

  if (initialized_)
  {
    collision_robot_ = planning_scene->getCollisionRobot();
    collision_world_ = planning_scene->getCollisionWorld();
  }

  trajectory_pub_ = nh.advertise<moveit_msgs::DisplayTrajectory>("/trajopt/display_planned_path", 1, true);
  collisions_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_collisions", 1, true);
  arrows_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_arrows", 1, true);
  axes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_axes", 1, true);
  return initialized_;
}

void ROSEnv::calcDistancesDiscrete(const DistanceRequest &req, std::vector<DistanceResult> &dists) const
{
  collision_detection::DistanceRequest distance_req;
  collision_detection::DistanceResult distance_res;

  distance_req.enable_nearest_points = true;
  distance_req.enable_signed_distance = true;
  distance_req.global_minimum_only = false;
  std::set<const moveit::core::LinkModel *> list = getLinkModels(req.link_names);
  distance_req.active_components_only = &list;
  distance_req.distance_threshold = req.contact_distance;
  distance_req.acm = &env_->getAllowedCollisionMatrix();

  robot_state::RobotState state = env_->getCurrentState();
  int i = 0;
  for(auto const& joint_name: req.joint_names)
  {
    state.setVariablePosition(joint_name, req.joint_angles1(i));
    ++i;
  }
  state.update();

  collision_robot_->distanceSelf(distance_req, distance_res, state);
  collision_world_->distanceRobot(distance_req, distance_res, *collision_robot_, state);

  dists.reserve(req.link_names.size());
  for (collision_detection::DistanceMap::iterator it = distance_res.distances.begin(); it!=distance_res.distances.end(); ++it)
  {
    DistanceResult d;
    d.distance = it->second.distance;
    d.valid = true;

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second.body_types[0] == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[0] = state.getAttachedBody(it->second.link_names[0])->getAttachedLinkName();
    }
    else
    {
      d.link_names[0] = it->second.link_names[0];
    }

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second.body_types[1] == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[1] = state.getAttachedBody(it->second.link_names[1])->getAttachedLinkName();
    }
    else
    {
      d.link_names[1] = it->second.link_names[1];
    }

    d.nearest_points[0] = it->second.nearest_points[0];
    d.nearest_points[1] = it->second.nearest_points[1];
    d.normal = it->second.normal;

    if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() || (d.normal.array().isNaN()).all())
      d.valid = false;

    dists.push_back(d);

  }
}

void ROSEnv::calcDistancesContinuous(const DistanceRequest &req, std::vector<DistanceResult> &dists) const
{
  collision_detection::DistanceRequest distance_req;
  collision_detection::DistanceResult distance_res;

  distance_req.enable_nearest_points = true;
  distance_req.enable_signed_distance = true;
  distance_req.global_minimum_only = false;
  std::set<const moveit::core::LinkModel *> list = getLinkModels(req.link_names);
  distance_req.active_components_only = &list;
  distance_req.distance_threshold = req.contact_distance;
  distance_req.acm = &env_->getAllowedCollisionMatrix();

  robot_state::RobotState state1 = env_->getCurrentState();
  robot_state::RobotState state2 = env_->getCurrentState();
  int i = 0;
  for(auto const& joint_name: req.joint_names)
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
  for (collision_detection::DistanceMap::iterator it = distance_res.distances.begin(); it!=distance_res.distances.end(); ++it)
  {
    DistanceResult d;
    d.distance = it->second.distance;
    d.valid = true;

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second.body_types[0] == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[0] = state1.getAttachedBody(it->second.link_names[0])->getAttachedLinkName();
    }
    else
    {
      d.link_names[0] = it->second.link_names[0];
    }

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second.body_types[1] == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[1] = state1.getAttachedBody(it->second.link_names[1])->getAttachedLinkName();
    }
    else
    {
      d.link_names[1] = it->second.link_names[1];
    }

    d.nearest_points[0] = it->second.nearest_points[0];
    d.nearest_points[1] = it->second.nearest_points[1];
    d.normal = it->second.normal;
    d.cc_type = it->second.cc_type;
    d.cc_nearest_points[0] = it->second.cc_nearest_points[0];
    d.cc_nearest_points[1] = it->second.cc_nearest_points[1];
    d.cc_time = it->second.cc_time;

    if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() || (d.normal.array().isNaN()).all())
      d.valid = false;

    if (d.cc_type != collision_detection::CCType_None && ((d.cc_nearest_points[0].array().isNaN()).all() || (d.cc_nearest_points[1].array().isNaN()).all()))
      d.valid = false;


    dists.push_back(d);

  }
}

std::set<const moveit::core::LinkModel *> ROSEnv::getLinkModels(const std::vector<std::string> &link_names) const
{
  std::set<const moveit::core::LinkModel *> list;

  for(auto const& link_name: link_names)
  {
    list.insert(env_->getRobotModel()->getLinkModel(link_name));
  }

  return list;
}

void ROSEnv::calcCollisionsDiscrete(const DistanceRequest &req, std::vector<DistanceResult> &collisions) const
{
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collisione_res;

  collision_req.group_name = getManipulatorName(req.joint_names);

  robot_state::RobotState state = env_->getCurrentState();
  int i = 0;
  for(auto const& joint_name: req.joint_names)
  {
    state.setVariablePosition(joint_name, req.joint_angles1(i));
    ++i;
  }
  state.update();

  collision_robot_->checkSelfCollision(collision_req, collisione_res, state, env_->getAllowedCollisionMatrix());
  collision_world_->checkRobotCollision(collision_req, collisione_res, *collision_robot_, state, env_->getAllowedCollisionMatrix());

  collisions.reserve(req.link_names.size());
  for (collision_detection::CollisionResult::ContactMap::iterator it = collisione_res.contacts.begin(); it!=collisione_res.contacts.end(); ++it)
  {
    DistanceResult d;
    d.distance = it->second[0].depth;
    d.valid = true;

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second[0].body_type_1 == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[0] = state.getAttachedBody(it->second[0].body_name_1)->getAttachedLinkName();
    }
    else
    {
      d.link_names[0] = it->second[0].body_name_1;
    }

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second[0].body_type_2 == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[1] = state.getAttachedBody(it->second[0].body_name_2)->getAttachedLinkName();
    }
    else
    {
      d.link_names[1] = it->second[0].body_name_2;
    }

    d.nearest_points[0] = it->second[0].pos;
    d.nearest_points[1] = it->second[0].pos;
    d.normal = it->second[0].normal;

    if ((d.nearest_points[0].array().isNaN()).all() || (d.nearest_points[1].array().isNaN()).all() || (d.normal.array().isNaN()).all())
      d.valid = false;

    collisions.push_back(d);
  }
}

void ROSEnv::calcCollisionsContinuous(const DistanceRequest &req, std::vector<DistanceResult> &collisions) const
{
  collision_detection::CollisionRequest collision_req;
  collision_detection::CollisionResult collisione_res;

  collision_req.group_name = getManipulatorName(req.joint_names);

  robot_state::RobotState state1 = env_->getCurrentState();
  robot_state::RobotState state2 = env_->getCurrentState();
  int i = 0;
  for(auto const& joint_name: req.joint_names)
  {
    state1.setVariablePosition(joint_name, req.joint_angles1(i));
    state2.setVariablePosition(joint_name, req.joint_angles2(i));
    ++i;
  }
  state1.update();
  state2.update();

  collision_robot_->checkSelfCollision(collision_req, collisione_res, state1, state2, env_->getAllowedCollisionMatrix());
  collision_world_->checkRobotCollision(collision_req, collisione_res, *collision_robot_, state1, state2, env_->getAllowedCollisionMatrix());

  collisions.reserve(req.link_names.size());
  for (collision_detection::CollisionResult::ContactMap::iterator it = collisione_res.contacts.begin(); it!=collisione_res.contacts.end(); ++it)
  {
    DistanceResult d;
    d.distance = it->second[0].depth;
    d.valid = true;

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second[0].body_type_1 == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[0] = state1.getAttachedBody(it->second[0].body_name_1)->getAttachedLinkName();
    }
    else
    {
      d.link_names[0] = it->second[0].body_name_1;
    }

    // Note: for trajopt RosEnv is only aware of links in the urdf so if attached link set link name to parent link name
    if (it->second[0].body_type_2 == collision_detection::BodyTypes::ROBOT_ATTACHED)
    {
      d.link_names[1] = state1.getAttachedBody(it->second[0].body_name_2)->getAttachedLinkName();
    }
    else
    {
      d.link_names[1] = it->second[0].body_name_2;
    }

    d.nearest_points[0] = it->second[0].pos;
    d.nearest_points[1] = it->second[0].pos;
    d.normal = it->second[0].normal;

//    d.cc_type = it->second.cc_type;
    d.cc_nearest_points[0] = it->second[0].pos;
    d.cc_nearest_points[1] = it->second[0].pos;
//    d.cc_time = it->second.cc_time;

    if (d.cc_type != collision_detection::CCType_None && ((d.cc_nearest_points[0].array().isNaN()).all() || (d.cc_nearest_points[1].array().isNaN()).all()))
      d.valid = false;


    collisions.push_back(d);
  }
}

bool ROSEnv::continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, std::vector<DistanceResult>& collisions) const
{
  BasicEnv::DistanceRequest req;
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

Eigen::VectorXd ROSEnv::getCurrentJointValues(const std::string &manipulator_name) const
{

  std::vector<std::string> joint_names = env_->getRobotModel()->getJointModelGroup(manipulator_name)->getActiveJointModelNames();
  Eigen::VectorXd start_pos(joint_names.size());

  for(auto j = 0u; j < joint_names.size(); ++j)
  {
    start_pos(j) = env_->getCurrentState().getVariablePosition(joint_names[j]);
  }

  return start_pos;
}

Eigen::VectorXd ROSEnv::getCurrentJointValues() const
{
  const double* vars = env_->getCurrentState().getVariablePositions();
  Eigen::Map<const Eigen::VectorXd> vect(vars, env_->getCurrentState().getVariableCount());
  return vect;
}

Eigen::Affine3d ROSEnv::getLinkTransform(const std::string& link_name) const
{
  return env_->getFrameTransform(link_name);
}

bool ROSEnv::hasManipulator(const std::string &manipulator_name) const
{
  return env_->getRobotModel()->hasJointModelGroup(manipulator_name);
}

BasicKinPtr ROSEnv::getManipulatorKin(const std::string &manipulator_name) const
{
  ROSKinPtr manip(new ROSKin());
  manip->init(env_->getRobotModel()->getJointModelGroup(manipulator_name));
  return manip;
}

std::string ROSEnv::getManipulatorName(const std::vector<std::string> &joint_names) const
{
  const std::vector<const moveit::core::JointModelGroup *> &groups = env_->getRobotModel()->getJointModelGroups();
  std::set<std::string> joint_names_set(joint_names.begin(), joint_names.end());
  for (auto group : groups)
  {
    const std::vector<std::string> &tmp_joint_names = group->getActiveJointModelNames();
    std::set<std::string> tmp_joint_names_set(tmp_joint_names.begin(), tmp_joint_names.end());
    if (joint_names_set == tmp_joint_names_set)
      return group->getName();

  }
  return "";
}

void ROSEnv::plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const TrajArray &traj)
{
  moveit_msgs::DisplayTrajectory msg;
  moveit_msgs::RobotTrajectory rt;
  rt.joint_trajectory.joint_names = joint_names;
  for (int i = 0; i < traj.rows(); ++i)
  {
    trajectory_msgs::JointTrajectoryPoint jtp;
    for (int j = 0; j < traj.cols(); ++j)
    {
      jtp.positions.push_back(traj(i, j));
    }
    jtp.time_from_start = ros::Duration(i);
    rt.joint_trajectory.points.push_back(jtp);
  }
  msg.trajectory.push_back(rt);
  moveit::core::robotStateToRobotStateMsg(env_->getCurrentState(), msg.trajectory_start);
  trajectory_pub_.publish(msg);

  plotWaitForInput();
}

visualization_msgs::Marker ROSEnv::getMarkerArrowMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = env_->getPlanningFrame();
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajopt";
  marker.id = ++marker_counter_;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  Eigen::Vector3d x, y, z;
  x = (pt2 - pt1).normalized();
  marker.pose.position.x = pt1(0);
  marker.pose.position.y = pt1(1);
  marker.pose.position.z = pt1(2);

  y = x.unitOrthogonal();
  z = (x.cross(y)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Quaterniond q(rot);
  q.normalize();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  marker.scale.x = std::abs((pt2 - pt1).norm());
  marker.scale.y = scale;
  marker.scale.z = scale;

  marker.color.r = rgba(0);
  marker.color.g = rgba(1);
  marker.color.b = rgba(2);
  marker.color.a = rgba(3);

  //    A Recently accepted PR in rviz has broken the use of start and end point
  //    geometry_msgs::Point start_pt, end_pt;
  //    start_pt.x = dist.nearest_points[1](0);
  //    start_pt.y = dist.nearest_points[1](1);
  //    start_pt.z = dist.nearest_points[1](2);

  //    end_pt.x = dist.nearest_points[0](0);
  //    end_pt.y = dist.nearest_points[0](1);
  //    end_pt.z = dist.nearest_points[0](2);

  //    auto it = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
  //    if (it != link_names.end())
  //    {
  //      marker.points.push_back(start_pt);
  //      marker.points.push_back(end_pt);
  //    }
  //    else
  //    {
  //      marker.points.push_back(end_pt);
  //      marker.points.push_back(start_pt);
  //    }
  //    marker.scale.x = 0.05;
  //    marker.scale.y = 0.01;
  //    marker.scale.z = 0;

  return marker;
}

visualization_msgs::Marker ROSEnv::getMarkerCylinderMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = env_->getPlanningFrame();
  marker.header.stamp = ros::Time::now();
  marker.ns = "trajopt";
  marker.id = ++marker_counter_;
  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  Eigen::Vector3d x, y, z;
  x = (pt2 - pt1).normalized();
  marker.pose.position.x = pt1(0);
  marker.pose.position.y = pt1(1);
  marker.pose.position.z = pt1(2);

  y = x.unitOrthogonal();
  z = (x.cross(y)).normalized();
  Eigen::Matrix3d rot;
  rot.col(0) = x;
  rot.col(1) = y;
  rot.col(2) = z;
  Eigen::Quaterniond q(rot);
  q.normalize();
  marker.pose.orientation.x = q.x();
  marker.pose.orientation.y = q.y();
  marker.pose.orientation.z = q.z();
  marker.pose.orientation.w = q.w();

  double length = std::abs((pt2 - pt1).norm());
  marker.scale.x = scale * length/20.0;
  marker.scale.y = scale * length/20.0;
  marker.scale.z = scale * length;

  marker.color.r = rgba(0);
  marker.color.g = rgba(1);
  marker.color.b = rgba(2);
  marker.color.a = rgba(3);

  return marker;
}

void ROSEnv::plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale)
{
  visualization_msgs::MarkerArray msg;
  msg.markers.push_back(getMarkerArrowMsg(pt1, pt2, rgba, scale));
  arrows_pub_.publish(msg);
}

void ROSEnv::plotAxis(const Eigen::Affine3d &axis, double scale)
{
  visualization_msgs::MarkerArray msg;
  Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
  Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
  Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
  Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

  msg.markers.push_back(getMarkerCylinderMsg(position, position + 0.1 * x_axis, Eigen::Vector4d(1, 0, 0, 1), scale));
  msg.markers.push_back(getMarkerCylinderMsg(position, position + 0.1 * y_axis, Eigen::Vector4d(0, 1, 0, 1), scale));
  msg.markers.push_back(getMarkerCylinderMsg(position, position + 0.1 * z_axis, Eigen::Vector4d(0, 0, 1, 1), scale));
  axes_pub_.publish(msg);
}

void ROSEnv::plotCollisions(const std::vector<std::string> &link_names, const std::vector<DistanceResult> &dist_results, double safe_dist)
{
  visualization_msgs::MarkerArray msg;
  for (int i = 0; i < dist_results.size(); ++i)
  {
    const DistanceResult &dist = dist_results[i];

    if (!dist.valid)
      continue;

    Eigen::Vector4d rgba;
    if (dist.distance < 0)
    {
      rgba << 1.0, 0.0, 0.0, 1.0;
    }
    else if (dist.distance < safe_dist)
    {
      rgba << 1.0, 1.0, 0.0, 1.0;
    }
    else
    {
      rgba << 0.0, 1.0, 0.0, 1.0;
    }

    Eigen::Vector3d ptA, ptB;
    ptA = dist.nearest_points[0];
    ptB = dist.nearest_points[1];

    auto it = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
    if (it != link_names.end())
    {
      ptA = dist.nearest_points[1];
      ptB = dist.nearest_points[0];
    }

    if(dist.cc_type == collision_detection::CCType_Between)
    {
      Eigen::Vector4d cc_rgba;
      cc_rgba << 0.0, 0.0, 0.0, 1.0;
      msg.markers.push_back(getMarkerArrowMsg(ptB, dist.cc_nearest_points[1], cc_rgba, 0.01));

      // DEGUG: This was added to see what the original contact point was for the cast continuous
      //        collision checking. Should be removed as everything has been integrated and tested.
      Eigen::Vector4d temp_rgba;
      temp_rgba << 0.0, 0.0, 1.0, 1.0;
      msg.markers.push_back(getMarkerArrowMsg(ptA, dist.cc_nearest_points[0], temp_rgba, 0.01));

      ptB = ((1 - dist.cc_time) * ptB + dist.cc_time * dist.cc_nearest_points[1]);

    }

     msg.markers.push_back(getMarkerArrowMsg(ptA, ptB, rgba, 0.01));
  }

  if (dist_results.size() > 0)
  {
    collisions_pub_.publish(msg);
  }
}

void ROSEnv::plotClear()
{
  // Remove old arrows
  marker_counter_ = 0;
  visualization_msgs::MarkerArray msg;
  visualization_msgs::Marker marker;
  marker.header.frame_id = env_->getPlanningFrame();
  marker.header.stamp = ros::Time();
  marker.ns = "trajopt";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(marker);
  collisions_pub_.publish(msg);
  arrows_pub_.publish(msg);
  axes_pub_.publish(msg);

  ros::Duration(0.5).sleep();
}

void ROSEnv::plotWaitForInput()
{
  ROS_ERROR("Hit enter key to step optimization!");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
}

}
