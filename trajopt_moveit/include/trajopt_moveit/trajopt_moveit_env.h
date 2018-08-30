#ifndef ROS_COLL_H
#define ROS_COLL_H

#include <tesseract_core/basic_env.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/collision_detection/collision_world.h>

namespace trajopt_moveit
{
class TrajOptMoveItEnv : public tesseract::BasicEnv
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
  void calcDistancesDiscrete(const tesseract::ContactRequest& req, tesseract::ContactResultVector& dists) const;

  void calcDistancesContinuous(const tesseract::ContactRequest& req, tesseract::ContactResultVector& dists) const;

  void calcCollisionsDiscrete(const tesseract::ContactRequest& req, tesseract::ContactResultVector& collisions) const;

  void calcCollisionsContinuous(const tesseract::ContactRequest& req, tesseract::ContactResultVector& collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string>& joint_names,
                                          const std::vector<std::string>& link_names,
                                          const tesseract::TrajArray& traj,
                                          tesseract::ContactResultVector& collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string>& joint_names,
                                          const std::vector<std::string>& link_names,
                                          const tesseract::TrajArray& traj,
                                          tesseract::ContactResult& collision) const;

  std::vector<std::string> getJointNames() const { return env_->getCurrentState().getVariableNames(); }
  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::VectorXd getCurrentJointValues(const std::string& manipulator_name) const;

  const std::string& getRootLinkName() const { return env_->getPlanningFrame(); }
  std::vector<std::string> getLinkNames() const;

  std::vector<std::string> getActiveLinkNames() const;

  tesseract::VectorIsometry3d getLinkTransforms() const;

  Eigen::Isometry3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string& manipulator_name) const;

  tesseract::BasicKinConstPtr getManipulator(const std::string& manipulator_name) const;

  tesseract::AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const
  {
    ROS_WARN("The getAllowedCollisions method is not implemented because moveit has its own allowed collision matrix");
    return nullptr;
  }

private:
  bool initialized_; /**< Identifies if the object has been initialized */
  planning_scene::PlanningScenePtr env_;
  std::vector<std::string> urdf_active_link_names_;             /**< A vector of active link names */
  collision_detection::CollisionRobotConstPtr collision_robot_; /**< Pointer to the collision robot, some constraints
                                                                   require it */
  collision_detection::CollisionWorldConstPtr collision_world_; /**< Pointer to the collision world, some constraints
                                                                   require it */

  std::set<const robot_model::LinkModel*> getLinkModels(const std::vector<std::string>& link_names) const;

  std::string getManipulatorName(const std::vector<std::string>& joint_names) const;
};
typedef std::shared_ptr<TrajOptMoveItEnv> TrajOptMoveItEnvPtr;
typedef std::shared_ptr<const TrajOptMoveItEnv> TrajOptMoveItEnvConstPtr;
}

#endif  // ROS_COLL_H
