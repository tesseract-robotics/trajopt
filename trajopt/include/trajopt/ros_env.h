#ifndef ROS_COLL_H
#define ROS_COLL_H
#include <trajopt/common.hpp>
#include <trajopt/basic_env.h>
#include <trajopt/ros_kin.h>
#include <moveit/planning_scene/planning_scene.h>
#include <industrial_collision_detection/collision_detection/collision_robot_industrial.h>
#include <industrial_collision_detection/collision_detection/collision_world_industrial.h>

namespace trajopt {

class TRAJOPT_API ROSEnv : public BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSEnv() : BasicEnv(), initialized_(false) {}

  bool init(PlanningScenePtr planning_scene)
  {
    env_ = planning_scene;
    initialized_ = planning_scene? true : false;

    if (initialized_)
    {
      collision_robot_ = std::static_pointer_cast<const collision_detection::CollisionRobotIndustrial>(planning_scene->getCollisionRobot());
      collision_world_ = std::static_pointer_cast<const collision_detection::CollisionWorldIndustrial>(planning_scene->getCollisionWorld());
    }

    return initialized_;
  }

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  void calcDistances(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names, std::vector<DistanceResult> &dists);

  void calcCollisions(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names);

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string &manipulator_name) const;

  BasicKinPtr getManipulatorKin(const std::string &manipulator_name) const;

private:
  bool initialized_; /**< Identifies if the object has been initialized */
  PlanningScenePtr env_;
  collision_detection::CollisionRobotIndustrialConstPtr collision_robot_; /**< Pointer to the collision robot, some constraints require it */
  collision_detection::CollisionWorldIndustrialConstPtr collision_world_; /**< Pointer to the collision world, some constraints require it */

  std::set<const robot_model::LinkModel*> getLinkModels(const std::vector<std::string> &link_names) const;

};
typedef boost::shared_ptr<ROSEnv> ROSEnvPtr;
}

#endif // ROS_COLL_H
