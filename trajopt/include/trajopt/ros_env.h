#ifndef ROS_COLL_H
#define ROS_COLL_H
#include <trajopt/common.hpp>
#include <trajopt/basic_env.h>
#include <trajopt/ros_kin.h>
#include <moveit/planning_scene/planning_scene.h>

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
    return initialized_;
  }

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  void calcDistances(const Eigen::VectorXd &joint_angles);

  void calcDistances(const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names);

  void calcCollisions(const Eigen::VectorXd &joint_angles);

  void calcCollisions(const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names);

  Eigen::VectorXd getCurrentJointValues(std::string manipulator_name) const;

  bool hasManipulator(std::string manipulator_name) const;

  BasicKinPtr getManipulatorKin(std::string manipulator_name) const;

private:
  bool initialized_; /**< Identifies if the object has been initialized */
  PlanningScenePtr env_;
};
typedef boost::shared_ptr<ROSEnv> ROSEnvPtr;
}

#endif // ROS_COLL_H
