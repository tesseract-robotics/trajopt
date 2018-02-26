#ifndef BASIC_COLL_H
#define BASIC_COLL_H
/**
 * @file basic_coll.h
 * @brief Basic low-level collision and distance functions.
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
#include <trajopt/common.hpp>
#include <trajopt/basic_kin.h>

namespace trajopt
{

class TRAJOPT_API BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  struct DistanceResult
  {
    double distance;
    std::string link_names[2];
    Vector3d nearest_points[2];
    Vector3d normal;
    Vector3d cc_nearest_points[2];
    double cc_time;
    collision_detection::ContinouseCollisionType cc_type;
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
      normal.setZero();
      cc_nearest_points[0].setZero();
      cc_nearest_points[1].setZero();
      cc_time = -1;
      cc_type = collision_detection::CCType_None;
    }
  };

  struct DistanceRequest
  {
    double contact_distance;              /**< The maximum distance between two objects for which distance data should be calculated */
    std::vector<std::string> joint_names; /**< Vector of joint names (size must match number of joints in robot chain) */
    std::vector<std::string> link_names;  /**< Name of the links to calculate distance data for. */
    Eigen::VectorXd joint_angles1;        /**< Vector of joint angles (size must match number of joints in robot chain) */
    Eigen::VectorXd joint_angles2;        /**< Vector of joint angles (size must match number of joints in robot chain) */

    DistanceRequest() : contact_distance(0.0) {}
  };

  BasicEnv() {}

  /**
   * @brief calcDistances Should return distance information for all links in list req.link_names (Discrete Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcDistancesDiscrete(const DistanceRequest &req, std::vector<DistanceResult> &dists) const = 0;

  /**
   * @brief calcDistances Should return distance information for all links in list link_names (Continuous Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcDistancesContinuous(const DistanceRequest &req, std::vector<DistanceResult> &dists) const = 0;

  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Discrete Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsDiscrete(const DistanceRequest &req, std::vector<DistanceResult> &collisions) const = 0;

  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Continuous Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsContinuous(const DistanceRequest &req, std::vector<DistanceResult> &collisions) const = 0;

  /**
   * @brief continuousCollisionCheckTrajectory Should perform a contuous collision check over the trajectory
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param link_names Name of the links to calculate collision data for.
   * @param traj The joint values at each time step
   * @param collisions The return collision data.
   * @return True if collision was found, otherwise false.
   */
  virtual bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, std::vector<DistanceResult>& collisions) const = 0;

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
  virtual BasicKinPtr getManipulatorKin(const std::string &manipulator_name) const = 0;

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
  virtual void plotCollisions(const std::vector<std::string> &link_names, const std::vector<BasicEnv::DistanceResult> &dist_results, double safe_dist) = 0;

  virtual void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale) = 0;

//  virtual void plotAxis(const std::string &name, const Eigen::Affine3d &axis, double scale) const = 0;

  /**
   * @brief This is called at the start of the plotting for each iteration
   *        to clear previous iteration graphics if neccessary.
   */
  virtual void plotClear() = 0;

  /** @brief plotWaitForInput Pause code and wait for enter key in terminal*/
  virtual void plotWaitForInput() = 0;



}; // class BasicColl

typedef boost::shared_ptr<BasicEnv> BasicEnvPtr;
} // namespace trajopt

#endif // BASIC_COLL_H
