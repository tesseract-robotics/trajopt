#ifndef ROS_COLL_H
#define ROS_COLL_H
#include <trajopt/common.hpp>
#include <trajopt/basic_coll.h>
#include <trajopt/ros_kin.h>
#include <moveit/planning_scene/planning_scene.h>

namespace trajopt {

class ROSColl : public BasicColl
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSColl() : {}

  bool init(ROSKinPtr manip, PlanningScenePtr planning_scene) : manip_(manip), planning_scene_(planning_scene) { initialized_ = true; }

  void calcDistances(const Eigen::VectorXd &joint_angles);

  void calcDistances(const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names);

  void calcCollisions(const Eigen::VectorXd &joint_angles);

  void calcCollisions(const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names);

private:
  ROSKinPtr manip_;
  PlanningScenePtr planning_scene_;
};

}

#endif // ROS_COLL_H
