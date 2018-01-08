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
  return initialized_;
}

void ROSEnv::calcDistances(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names, std::vector<DistanceResult> &dists)
{
  collision_detection::DistanceRequest distance_req;
  collision_detection::DistanceResult distance_res;

  distance_req.enable_nearest_points = true;
  distance_req.enable_signed_distance = true;
  distance_req.global = false;
  std::set<const moveit::core::LinkModel *> list = getLinkModels(link_names);
  distance_req.active_components_only = &list;
  distance_req.distance_threshold = 0.075; // Same as original trajopt dist_pen + 0.04
  distance_req.acm = &env_->getAllowedCollisionMatrix();

  robot_state::RobotState state = env_->getCurrentState();
  int i = 0;
  for(auto const& joint_name: joint_names)
  {
    state.setVariablePosition(joint_name, joint_angles(i));
    ++i;
  }
  state.update();

  collision_robot_->distanceSelf(distance_req, distance_res, state);
  collision_world_->distanceRobot(distance_req, distance_res, *collision_robot_, state);

  dists.reserve(link_names.size());
  for (collision_detection::DistanceMap::iterator it = distance_res.distances.begin(); it!=distance_res.distances.end(); ++it)
  {
    DistanceResult d;
    d.distance = it->second.distance;
    d.valid = true;
    d.link_names[0] = it->second.link_names[0];
    d.link_names[1] = it->second.link_names[1];
    d.nearest_points[0] = it->second.nearest_points[0];
    d.nearest_points[1] = it->second.nearest_points[1];
    d.normal = it->second.normal;

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

void ROSEnv::calcCollisions(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names)
{

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
  ros::spinOnce();
  plotWaitForInput();
}

void ROSEnv::plotCollisions(const std::vector<std::string> &link_names, const std::vector<DistanceResult> &dist_results)
{
  visualization_msgs::MarkerArray msg;
  for (int i = 0; i < dist_results.size(); ++i)
  {
    const DistanceResult &dist = dist_results[i];
    visualization_msgs::Marker marker;
    marker.header.frame_id = env_->getPlanningFrame();
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajopt";
    marker.id = ++marker_counter_;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d x, y, z;

    auto it = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
    if (it != link_names.end())
    {
      x = (dist.nearest_points[0] - dist.nearest_points[1]).normalized();
      marker.pose.position.x = dist.nearest_points[1](0);
      marker.pose.position.y = dist.nearest_points[1](1);
      marker.pose.position.z = dist.nearest_points[1](2);
    }
    else
    {
      x = (dist.nearest_points[1] - dist.nearest_points[0]).normalized();
      marker.pose.position.x = dist.nearest_points[0](0);
      marker.pose.position.y = dist.nearest_points[0](1);
      marker.pose.position.z = dist.nearest_points[0](2);
    }
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

    marker.scale.x = std::abs(dist.distance);
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;

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

    if (dist.distance > 0)
    {
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
    }
    else
    {
      marker.color.a = 1.0;
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    }


    msg.markers.push_back(marker);
  }

  if (dist_results.size() > 0)
  {
    collisions_pub_.publish(msg);
    ros::spinOnce();
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
  ros::spinOnce();
}

void ROSEnv::plotWaitForInput()
{
  ROS_ERROR("Hit enter key to step optimization!");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
}

}
