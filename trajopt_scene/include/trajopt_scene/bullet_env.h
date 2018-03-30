#ifndef ROS_BULLET_ENV_H
#define ROS_BULLET_ENV_H

#include <trajopt_scene/bullet_utils.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace trajopt_scene
{

class BulletEnv : public BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BulletEnv() : BasicEnv(), initialized_(false) {}

  bool init(const urdf::ModelInterfaceConstSharedPtr urdf_model);
  bool init(const urdf::ModelInterfaceConstSharedPtr urdf_model, const srdf::ModelConstSharedPtr srdf_model);

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  bool checkInitialized() const { return initialized_; }

  void calcDistancesDiscrete(const DistanceRequest &req, DistanceResultVector &dists) const;

  void calcDistancesContinuous(const DistanceRequest &req, DistanceResultVector &dists) const;

  void calcCollisionsDiscrete(const DistanceRequest &req, DistanceResultVector &collisions) const;

  void calcCollisionsContinuous(const DistanceRequest &req, DistanceResultVector &collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, DistanceResultVector& collisions) const;

  bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, DistanceResult& collision) const;

  const EnvStateConstPtr getState() const {return current_state_; }

  void setState(const std::map<const std::string, double> &joints);
  void setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values);

  EnvStatePtr getState(const std::map<const std::string, double> &joints) const;
  EnvStatePtr getState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values) const;

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  Eigen::VectorXd getCurrentJointValues() const;

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  /**
   * @brief A a manipulator as a kinematic chain
   * @param base_link The base link of the chain
   * @param tip_link The tip link of the chain
   * @param name The name of the manipulator. This must be unique.
   * @return true if successfully created, otherwise false.
   */
  virtual bool addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name);

  bool hasManipulator(const std::string &manipulator_name) const;

  BasicKinConstPtr getManipulator(const std::string &manipulator_name) const;

  /**
   * @brief Add object so it may be attached/detached.
   *
   * This object is not part of the environment until attached to a link.
   *
   * @param attachable_object The object information
   */
  virtual void addAttachableObject(const AttachableObjectConstPtr &attachable_object);

  /**
   * @brief Get object attached to the manipulator or world
   * @param name The name of the object
   * @return AttachedBody
   */
  virtual const AttachedBodyConstPtr getAttachedBody(const std::string& name) const;

  /**
   * @brief Attached an attachable object to the environment
   * @param attached_body Information of attaching creating the attached body
   */
  virtual void attachBody(const AttachedBodyInfo &attached_body_info);

  /**
   * @brief Detach an attachable object from the environment
   * @param name The name given to the Attached Body when attached
   */
  virtual void detachBody(const std::string &name);

  void updateVisualization() const;

  void enablePlotting(bool enable) { plotting_ = enable; }

  void plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const TrajArray &traj);

  void plotCollisions(const std::vector<std::string> &link_names, const DistanceResultVector &dist_results, double safe_dist);

  void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  void plotAxis(const Eigen::Affine3d &axis, double scale);

  void plotClear();

  void plotWaitForInput();

private:
  bool initialized_;                                                         /**< Identifies if the object has been initialized */
  urdf::ModelInterfaceConstSharedPtr model_;                                 /**< URDF MODEL */
  srdf::ModelConstSharedPtr srdf_model_;                                     /**< SRDF MODEL */
  boost::shared_ptr<const KDL::Tree> kdl_tree_;                              /**< KDL tree object */
  Link2ConstCow robot_link2cow_;                                             /**< URDF Collision objects */
  Link2Cow attached_link2cow_;                                               /**< Collision objects not part of the URDF */
  EnvStatePtr current_state_;                                                /**< Current state of the robot */
  std::map<const std::string, unsigned int> joint_to_qnr_;                   /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                              /**< The kdl joint array */
  std::map<const std::string, AttachedBodyConstPtr> attached_bodies_;        /**< A map of attached bodies */
  std::map<const std::string, AttachableObjectConstPtr> attachable_objects_; /**< A map of objects that can be attached/detached from environment */
  std::map<const std::string, BasicKinConstPtr> manipulators_;               /**< A map of manipulator names to kinematics object */

  bool plotting_;                                         /**< Enable plotting */
  int marker_counter_;                                    /**< Counter when plotting */
  ros::Publisher scene_pub_;                              /**< Trajectory publisher */
  ros::Publisher trajectory_pub_;                         /**< Trajectory publisher */
  ros::Publisher collisions_pub_;                         /**< Collision Data publisher */
  ros::Publisher arrows_pub_;                             /**< Used for publishing arrow markers */
  ros::Publisher axes_pub_;                               /**< Used for publishing axis markers */


  void calculateTransforms(std::map<const std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const;

  bool setJointValuesHelper(KDL::JntArray &q, const std::string &joint_name, const double &joint_value) const;

  visualization_msgs::Marker getMarkerArrowMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  visualization_msgs::Marker getMarkerCylinderMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  std::string getManipulatorName(const std::vector<std::string> &joint_names) const;

  void constructBulletObject(Link2Cow &collision_objects, std::vector<std::string> &active_objects, double contact_distance, const EnvStateConstPtr state, const std::vector<std::string> &active_links, bool continuous = false) const;

  void constructBulletObject(Link2Cow &collision_objects, std::vector<std::string> &active_objects, double contact_distance, const EnvStateConstPtr state1, const EnvStateConstPtr state2, const std::vector<std::string> &active_links) const;

};
typedef boost::shared_ptr<BulletEnv> BulletEnvPtr;
typedef boost::shared_ptr<const BulletEnv> BulletEnvConstPtr;
}

#endif // ROS_BULLET_ENV_H
