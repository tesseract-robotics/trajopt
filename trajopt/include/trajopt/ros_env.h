#ifndef ROS_COLL_H
#define ROS_COLL_H
#include <trajopt/common.hpp>
#include <trajopt/basic_env.h>
#include <trajopt/ros_kin.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>

namespace trajopt {

class TRAJOPT_API ROSEnv : public BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSEnv() : BasicEnv(), initialized_(false) {}

  bool init(PlanningScenePtr planning_scene);

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  void calcDistances(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names, std::vector<DistanceResult> &dists);

  void calcDistances(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles1, const Eigen::VectorXd &joint_angles2, const std::vector<std::string> &link_names, std::vector<DistanceResult> &dists);

  void calcCollisions(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles, const std::vector<std::string> &link_names);

  void calcCollisions(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_angles1, const Eigen::VectorXd &joint_angles2, const std::vector<std::string> &link_names);

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string &manipulator_name) const;

  BasicKinPtr getManipulatorKin(const std::string &manipulator_name) const;

  void enablePlotting(bool enable) { plotting_ = enable; }

  void plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const TrajArray &traj);

  void plotCollisions(const std::vector<std::string> &link_names, const std::vector<BasicEnv::DistanceResult> &dist_results);

  void plotClear();

  void plotWaitForInput();

private:
  bool initialized_; /**< Identifies if the object has been initialized */
  PlanningScenePtr env_;
  collision_detection::CollisionRobotConstPtr collision_robot_; /**< Pointer to the collision robot, some constraints require it */
  collision_detection::CollisionWorldConstPtr collision_world_; /**< Pointer to the collision world, some constraints require it */

  bool plotting_;                 /**< Enable plotting */
  int marker_counter_;            /**< Counter when plotting */
  ros::Publisher trajectory_pub_; /**< Trajectory publisher */
  ros::Publisher collisions_pub_; /**< Collision Data publisher */

  std::set<const robot_model::LinkModel*> getLinkModels(const std::vector<std::string> &link_names) const;

};
typedef boost::shared_ptr<ROSEnv> ROSEnvPtr;
}

#endif // ROS_COLL_H
