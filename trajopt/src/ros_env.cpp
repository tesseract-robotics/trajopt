#include "trajopt/ros_env.h"
#include <industrial_collision_detection/collision_detection/collision_common.h>

namespace trajopt
{

using Eigen::MatrixXd;
using Eigen::VectorXd;

void ROSEnv::calcDistances(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names, std::vector<DistanceResult> &dists)
{
  collision_detection::DistanceRequest distance_req;
  collision_detection::DistanceResult distance_res;

  distance_req.detailed = true;
  distance_req.global = false;
  std::set<const moveit::core::LinkModel *> list = getLinkModels(link_names);
  distance_req.active_components_only = &list;
  distance_req.distance_threshold = 0.025;
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

//  collision_detection::CollisionRequest collision_req;
//  collision_detection::CollisionResult collision_res;
//  if (distance_res.collision)
//  {
//    collision_req.distance = false;
//    collision_req.contacts = true;
//    collision_req.max_contacts = link_names.size() * 2.0;
//    collision_robot_->checkSelfCollision(collision_req, collision_res, state, *distance_req.acm);
//    collision_world_->checkRobotCollision(collision_req, collision_res, *collision_robot_, state, *distance_req.acm);

//    // TODO: This needs to be improved to add penetration to distance_res.
//  }

  dists.reserve(link_names.size());
  for (collision_detection::DistanceMap::iterator it = distance_res.distance.begin(); it!=distance_res.distance.end(); ++it)
  {
    DistanceResult d;
    d.distance = it->second.min_distance;
    d.valid = true;
    d.link_name[0] = it->second.link_name[0];
    d.link_name[1] = it->second.link_name[1];
    d.nearest_point[0] = it->second.nearest_points[0];
    d.nearest_point[1] = it->second.nearest_points[1];

    if (d.distance <= std::numeric_limits<double>::min())
    {
      d.distance = -1.0 * (d.nearest_point[0] - d.nearest_point[1]).norm();
    }

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

}
