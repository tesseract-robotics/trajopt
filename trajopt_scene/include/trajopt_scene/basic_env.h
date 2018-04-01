#ifndef BASIC_COLL_H
#define BASIC_COLL_H
/**
 * @file basic_env.h
 * @brief Basic low-level environment with collision and distance functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/shared_ptr.hpp>
#include <trajopt_scene/basic_kin.h>
#include <map>

namespace trajopt_scene
{

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
typedef std::map<std::pair<std::string, std::string>, std::string> AllowedCollisionMatrix;
typedef boost::shared_ptr<AllowedCollisionMatrix> AllowedCollisionMatrixPtr;
typedef boost::shared_ptr<const AllowedCollisionMatrix> AllowedCollisionMatrixConstPtr;

/**
 * @brief Get a key for two object to search the collision matrix
 * @param obj1 First collision object name
 * @param obj2 Second collision object name
 * @return The collision pair key
 */
static std::pair<std::string, std::string> getObjectPairKey(const std::string &obj1, const std::string &obj2)
{
  return obj1 < obj2 ? std::make_pair(obj1, obj2) : std::make_pair(obj2, obj1);
}

namespace BodyTypes
{
enum BodyType
{
  ROBOT_LINK,     /**< @brief These are links at the creation of the environment */
  ROBOT_ATTACHED  /**< @brief These are links that are added after initial creation */
};
}
typedef BodyTypes::BodyType BodyType;

namespace ContinouseCollisionTypes
{
enum ContinouseCollisionType
{
  CCType_None,
  CCType_Time0,
  CCType_Time1,
  CCType_Between
};
}
typedef ContinouseCollisionTypes::ContinouseCollisionType ContinouseCollisionType;

namespace DistanceRequestTypes
{
enum DistanceRequestType
{
  SINGLE, /**< Return the global minimum for a pair of objects */
  ALL  ,  /**< Return all contacts for a pair of objects */
  LIMITED /**< Return limited set of contacts for a pair of objects */
};
}
typedef DistanceRequestTypes::DistanceRequestType DistanceRequestType;

struct DistanceRequest
{
  DistanceRequestType type;             /**< The type of distance request */
  double contact_distance;              /**< The maximum distance between two objects for which distance data should be calculated */
  std::vector<std::string> joint_names; /**< Vector of joint names (size must match number of joints in robot chain) */
  std::vector<std::string> link_names;  /**< Name of the links to calculate distance data for. */
  Eigen::VectorXd joint_angles1;        /**< Vector of joint angles (size must match number of joints in robot chain/tree) */
  Eigen::VectorXd joint_angles2;        /**< Vector of joint angles (size must match number of joints in robot chain/tree) */
  AllowedCollisionMatrixConstPtr acm;   /**< The allowed collision matrix */

  DistanceRequest() : type(DistanceRequestType::SINGLE), contact_distance(0.0) {}
};

struct DistanceResult
{
  double distance;
  std::string link_names[2];
  BodyType body_types[2];
  Eigen::Vector3d nearest_points[2];
  Eigen::Vector3d normal;
  Eigen::Vector3d cc_nearest_points[2];
  double cc_time;
  ContinouseCollisionType cc_type;
  bool valid;

  DistanceResult() { clear(); }

  /// Clear structure data
  void clear()
  {
    distance = std::numeric_limits<double>::max();
    nearest_points[0].setZero();
    nearest_points[1].setZero();
    link_names[0] = "";
    link_names[1] = "";
    body_types[0] = BodyType::ROBOT_LINK;
    body_types[1] = BodyType::ROBOT_LINK;
    normal.setZero();
    cc_nearest_points[0].setZero();
    cc_nearest_points[1].setZero();
    cc_time = -1;
    cc_type = ContinouseCollisionType::CCType_None;
  }
};
typedef std::vector<DistanceResult> DistanceResultVector;

/** @brief This holds a state of the environment */
struct EnvState
{
  std::map<std::string, double> joints;
  std::map<std::string, Eigen::Affine3d> transforms;
};
typedef boost::shared_ptr<EnvState> EnvStatePtr;
typedef boost::shared_ptr<const EnvState> EnvStateConstPtr;


class BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicEnv() : allowed_collision_matrix_(new AllowedCollisionMatrix()) {}

  /**
   * @brief calcDistances Should return distance information for all links in list req.link_names (Discrete Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcDistancesDiscrete(const DistanceRequest &req, DistanceResultVector &dists) const = 0;

  /**
   * @brief calcDistances Should return distance information for all links in list link_names (Continuous Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcDistancesContinuous(const DistanceRequest &req, DistanceResultVector &dists) const = 0;

  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Discrete Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsDiscrete(const DistanceRequest &req, DistanceResultVector &collisions) const = 0;

  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Continuous Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsContinuous(const DistanceRequest &req, DistanceResultVector &collisions) const = 0;

  /**
   * @brief continuousCollisionCheckTrajectory Should perform a continuous collision check over the trajectory
   * and return every collision contact
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param link_names Name of the links to calculate collision data for.
   * @param traj The joint values at each time step
   * @param collisions The return collision data.
   * @return True if collision was found, otherwise false.
   */
  virtual bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, DistanceResultVector& collisions) const = 0;

  /**
   * @brief continuousCollisionCheckTrajectory Should perform a continuous collision check over the trajectory
   * and stop on first collision.
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param link_names Name of the links to calculate collision data for.
   * @param traj The joint values at each time step
   * @param collision The return collision data.
   * @return True if collision was found, otherwise false.
   */
  virtual bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, DistanceResult& collision) const = 0;

  /** @brief Set the current state of the environment */
  virtual void setState(const std::map<std::string, double> &joints) = 0;
  virtual void setState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values) = 0;

  /** @brief Get the current state of the environment */
  virtual const EnvStateConstPtr getState() const = 0;

  /**
   * @brief Get the state of the environment for a given set or subset of joint values.
   *
   * This does not change the internal state of the environment.
   *
   * @param joints A map of joint names to joint values to change.
   * @return A the state of the environment
   */
  virtual EnvStatePtr getState(const std::map<std::string, double> &joints) const = 0;
  virtual EnvStatePtr getState(const std::vector<std::string> &joint_names, const Eigen::VectorXd &joint_values) const = 0;

  /**
   * @brief getCurrentJointValues Get the current state of the manipulator
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const = 0;

  /**
   * @brief getCurrentJointValues Get the current state of all joints
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues() const = 0;

  /** @brief Get the transform corresponding to the link.
   *  @return Tranform and is identity when no transform is available.
   */
  virtual Eigen::Affine3d getLinkTransform(const std::string &link_name) const = 0;

  /**
   * @brief hasManipulator Check if a manipulator exist in the environment
   * @param manipulator_name Name of the manipulator
   * @return True if it exists otherwise false
   */
  virtual bool hasManipulator(const std::string &manipulator_name) const = 0;

  /**
   * @brief getManipulatorKin Get a kinematic object for the provided manipulator name.
   * @param manipulator_name Name of the manipulator
   * @return BasicKinPtr
   */
  virtual BasicKinConstPtr getManipulator(const std::string &manipulator_name) const = 0;

  /** @brief Plot the current scene */
  virtual void updateVisualization() const = 0;

  /**
   * @brief enablePlotting Endicate if data should be plotted/published
   * @param enable
   */
  virtual void enablePlotting(bool enable) = 0;

  /**
   * @brief plotTrajectory Plot a trajectory
   * @param traj
   */
  virtual void plotTrajectory(const std::string &name, const std::vector<std::string> &joint_names, const TrajArray &traj) = 0;

  /**
   * @brief plotCollisions Plot the collision results data
   * @param link_names List of link names for which to plot data
   * @param dist_results The collision results data
   */
  virtual void plotCollisions(const std::vector<std::string> &link_names, const DistanceResultVector &dist_results, double safe_dist) = 0;

  /**
   * @brief plotArrow Plot arrow defined by two points
   * @param pt1 Start position of the arrow
   * @param pt2 Final position of the arrow
   * @param rgba Color of the arrow
   * @param scale The size of the arrow (related to diameter)
   */
  virtual void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale) = 0;

  /**
   * @brief plotAxis Plat axis
   * @param axis The axis
   * @param scale The size of the axis
   */
  virtual void plotAxis(const Eigen::Affine3d &axis, double scale) = 0;

  /**
   * @brief This is called at the start of the plotting for each iteration
   *        to clear previous iteration graphics if neccessary.
   */
  virtual void plotClear() = 0;

  /** @brief plotWaitForInput Pause code and wait for enter key in terminal*/
  virtual void plotWaitForInput() = 0;

  /////////////////////////
  // Implemented Methods //
  /////////////////////////

  /**
   * @brief Disable collision between two collision objects
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   * @param reason The reason for disabling collison
   */
  virtual void addAllowedCollision(const std::string &obj1, const std::string &obj2, const std::string &reason)
  {
    allowed_collision_matrix_->insert(std::make_pair(getObjectPairKey(obj1, obj2), reason));
  }

  /**
   * @brief Remove disabled collision pair from allowed collision matrix
   * @param obj1 Collision object name
   * @param obj2 Collision object name
   */
  virtual void removeAllowedCollision(const std::string &obj1, const std::string &obj2)
  {
    allowed_collision_matrix_->erase(getObjectPairKey(obj1, obj2));
  }

  /** @brief Get the allowed collision matrix */
  virtual AllowedCollisionMatrixConstPtr getAllowedCollisions() const
  {
    return allowed_collision_matrix_;
  }

protected:
  AllowedCollisionMatrixPtr allowed_collision_matrix_; /**< The allowed collision matrix used during collision checking */


}; // class BasicColl

typedef boost::shared_ptr<BasicEnv> BasicEnvPtr;
typedef boost::shared_ptr<const BasicEnv> BasicEnvConstPtr;
} // namespace trajopt_scene

#endif // BASIC_COLL_H
