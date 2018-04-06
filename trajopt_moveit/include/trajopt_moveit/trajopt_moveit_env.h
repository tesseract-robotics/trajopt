#ifndef ROS_COLL_H
#define ROS_COLL_H

#include <trajopt_scene/basic_env.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>

namespace trajopt_moveit {

class TrajOptMoveItEnv : public trajopt_scene::BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajOptMoveItEnv() : BasicEnv(), initialized_(false) {}

  bool init(planning_scene::PlanningScenePtr planning_scene);

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  void calcDistancesDiscrete(const trajopt_scene::DistanceRequest &req, trajopt_scene::DistanceResultVector &dists) const;

  void calcDistancesContinuous(const trajopt_scene::DistanceRequest &req, trajopt_scene::DistanceResultVector &dists) const;

  void calcCollisionsDiscrete(const trajopt_scene::DistanceRequest &req, trajopt_scene::DistanceResultVector &collisions) const;

  void calcCollisionsContinuous(const trajopt_scene::DistanceRequest &req, trajopt_scene::DistanceResultVector &collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const trajopt_scene::TrajArray& traj, trajopt_scene::DistanceResultVector& collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const trajopt_scene::TrajArray& traj, trajopt_scene::DistanceResult &collision) const;

  const trajopt_scene::EnvStateConstPtr getState() const
  {
    ROS_WARN("The getState method is not implemented because moveit has its own state representation");
    return nullptr;
  }

  void setState(const std::map<std::string, double> &joints)
  {
    ROS_WARN("The setState method is not implemented because moveit has its own state representation");
  }

  void setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values)
  {
    ROS_WARN("The setState method is not implemented because moveit has its own state representation");
  }

  trajopt_scene::EnvStatePtr getState(const std::map<std::string, double> &joints) const
  {
    ROS_WARN("The getState method is not implemented because moveit has its own state representation");
    return nullptr;
  }

  trajopt_scene::EnvStatePtr getState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values) const
  {
    ROS_WARN("The getState method is not implemented because moveit has its own state representation");
    return nullptr;
  }

  std::vector<std::string> getJointNames() const
  {
    return env_->getCurrentState().getVariableNames();
  }

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  std::vector<std::string> getLinkNames() const;

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string &manipulator_name) const;

  trajopt_scene::BasicKinConstPtr getManipulator(const std::string &manipulator_name) const;

  void updateVisualization() const
  {
    ROS_WARN("The updateVisualization method is not implemented because moveit has its own visualization update");
  }

  void enablePlotting(bool enable) { plotting_ = enable; }

  void plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const trajopt_scene::TrajArray &traj);

  void plotCollisions(const std::vector<std::string> &link_names, const trajopt_scene::DistanceResultVector &dist_results, const Eigen::VectorXd &safety_distances);

  void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  void plotAxis(const Eigen::Affine3d &axis, double scale);

  void plotClear();

  void plotWaitForInput();

private:
  bool initialized_;        /**< Identifies if the object has been initialized */
  planning_scene::PlanningScenePtr env_;
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
typedef boost::shared_ptr<TrajOptMoveItEnv> TrajOptMoveItEnvPtr;
}

#endif // ROS_COLL_H
