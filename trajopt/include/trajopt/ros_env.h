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

  void calcDistancesDiscrete(const BasicEnv::DistanceRequest &req, std::vector<DistanceResult> &dists) const;

  void calcDistancesContinuous(const BasicEnv::DistanceRequest &req, std::vector<DistanceResult> &dists) const;

  void calcCollisionsDiscrete(const BasicEnv::DistanceRequest &req, std::vector<DistanceResult> &collisions) const;

  void calcCollisionsContinuous(const BasicEnv::DistanceRequest &req, std::vector<DistanceResult> &collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, std::vector<DistanceResult>& collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, DistanceResult collision) const;

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string &manipulator_name) const;

  BasicKinPtr getManipulator(const std::string &manipulator_name) const;

  void enablePlotting(bool enable) { plotting_ = enable; }

  void plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const TrajArray &traj);

  void plotCollisions(const std::vector<std::string> &link_names, const std::vector<BasicEnv::DistanceResult> &dist_results, double safe_dist);

  void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  void plotAxis(const Eigen::Affine3d &axis, double scale);

  void plotClear();

  void plotWaitForInput();

private:
  bool initialized_;        /**< Identifies if the object has been initialized */
  PlanningScenePtr env_;
  collision_detection::CollisionRobotConstPtr collision_robot_; /**< Pointer to the collision robot, some constraints require it */
  collision_detection::CollisionWorldConstPtr collision_world_; /**< Pointer to the collision world, some constraints require it */

  bool plotting_;                 /**< Enable plotting */
  int marker_counter_;            /**< Counter when plotting */
  ros::Publisher trajectory_pub_; /**< Trajectory publisher */
  ros::Publisher collisions_pub_; /**< Collision Data publisher */
  ros::Publisher arrows_pub_;     /**< Used for publishing arrow markers */
  ros::Publisher axes_pub_;       /**< Used for publishing axis markers */

  std::set<const robot_model::LinkModel*> getLinkModels(const std::vector<std::string> &link_names) const;

  visualization_msgs::Marker getMarkerArrowMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  visualization_msgs::Marker getMarkerCylinderMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  std::string getManipulatorName(const std::vector<std::string> &joint_names) const;

};
typedef boost::shared_ptr<ROSEnv> ROSEnvPtr;
}

#endif // ROS_COLL_H
