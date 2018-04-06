#ifndef TESSERACT_ROS_BULLET_ENV_H
#define TESSERACT_ROS_BULLET_ENV_H

#include <tesseract_ros/bullet/bullet_utils.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/jntarray.hpp>
#include <sensor_msgs/JointState.h>
#include <ros/publisher.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <moveit_msgs/DisplayRobotState.h>

namespace tesseract
{

class BulletEnv : public ROSBasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BulletEnv() : ROSBasicEnv(), initialized_(false) {}

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

  void setState(const std::map<std::string, double> &joints);
  void setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values);

  EnvStatePtr getState(const std::map<std::string, double> &joints) const;
  EnvStatePtr getState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values) const;

  std::vector<std::string> getJointNames() const { return joint_names_; }

  Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const;

  std::vector<std::string> getLinkNames() const { return link_names_; }

  Eigen::Affine3d getLinkTransform(const std::string& link_name) const;

  bool hasManipulator(const std::string &manipulator_name) const;

  BasicKinConstPtr getManipulator(const std::string &manipulator_name) const;

  bool addManipulator(const std::string &base_link, const std::string &tip_link, const std::string &manipulator_name);

  void addAttachableObject(const AttachableObjectConstPtr &attachable_object);

  const AttachedBodyConstPtr getAttachedBody(const std::string& name) const;

  void attachBody(const AttachedBodyInfo &attached_body_info);

  void detachBody(const std::string &name);

  void updateVisualization() const;

  void enablePlotting(bool enable) { plotting_ = enable; }

  void plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const TrajArray &traj);

  void plotCollisions(const std::vector<std::string> &link_names, const DistanceResultVector &dist_results, const Eigen::VectorXd& safety_distances);

  void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale);

  void plotAxis(const Eigen::Affine3d &axis, double scale);

  void plotClear();

  void plotWaitForInput();

private:
  bool initialized_;                                                   /**< Identifies if the object has been initialized */
  urdf::ModelInterfaceConstSharedPtr model_;                           /**< URDF MODEL */
  srdf::ModelConstSharedPtr srdf_model_;                               /**< SRDF MODEL */
  boost::shared_ptr<const KDL::Tree> kdl_tree_;                        /**< KDL tree object */
  Link2ConstCow link2cow_;                                             /**< Collision objects */
  EnvStatePtr current_state_;                                          /**< Current state of the robot */
  std::map<std::string, unsigned int> joint_to_qnr_;                   /**< Map between joint name and kdl q index */
  KDL::JntArray kdl_jnt_array_;                                        /**< The kdl joint array */
  std::map<std::string, AttachedBodyConstPtr> attached_bodies_;        /**< A map of attached bodies */
  std::map<std::string, AttachableObjectConstPtr> attachable_objects_; /**< A map of objects that can be attached/detached from environment */
  std::map<std::string, BasicKinConstPtr> manipulators_;               /**< A map of manipulator names to kinematics object */
  std::vector<std::string> link_names_;                                /**< A vector of link names */
  std::vector<std::string> joint_names_;                               /**< A vector of joint names */

  bool plotting_;                                         /**< Enable plotting */
  int marker_counter_;                                    /**< Counter when plotting */
  ros::Publisher scene_pub_;                              /**< Scene publisher */
  ros::Publisher trajectory_pub_;                         /**< Trajectory publisher */
  ros::Publisher collisions_pub_;                         /**< Collision Data publisher */
  ros::Publisher arrows_pub_;                             /**< Used for publishing arrow markers */
  ros::Publisher axes_pub_;                               /**< Used for publishing axis markers */


  moveit_msgs::RobotStatePtr getRobotStateMsg() const;

  void calculateTransforms(std::map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const;
  void calculateTransformsHelper(std::map<std::string, Eigen::Affine3d> &transforms, const KDL::JntArray& q_in, const KDL::SegmentMap::const_iterator& it, const Eigen::Affine3d& parent_frame) const;

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

#endif // TESSERACT_ROS_BULLET_ENV_H
