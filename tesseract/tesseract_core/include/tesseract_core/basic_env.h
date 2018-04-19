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
#ifndef TESSERACT_CORE_BASIC_ENV_H
#define TESSERACT_CORE_BASIC_ENV_H

#include <vector>
#include <string>
#include <tesseract_core/basic_types.h>
#include <tesseract_core/basic_kin.h>


namespace tesseract
{

class EnvBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Get a vector of joint names in the environment
   * @return A vector of joint names
   */
  virtual std::vector<std::string> getJointNames() const = 0;

  /**
   * @brief Get the current state of the environment
   *
   * Order should be the same as getJointNames()
   *
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues() const = 0;

  /**
   * @brief Get the current state of the manipulator
   *
   * Order should be the same as BasicKin.getJointNames()
   *
   * @return A vector of joint values
   */
  virtual Eigen::VectorXd getCurrentJointValues(const std::string &manipulator_name) const = 0;

  /**
   * @brief Get the root link name
   * @return String
   */
  virtual const std::string& getRootLinkName() const = 0;

  /**
   * @brief Get a vector of link names in the environment
   * @return A vector of link names
   */
  virtual std::vector<std::string> getLinkNames() const = 0;

  /**
   * @brief Get a vector of active link names in the environment
   * @return A vector of active link names
   */
  virtual std::vector<std::string> getActiveLinkNames() const = 0;

  /**
   * @brief Get all of the links transforms
   *
   * Order should be the same as getLinkNames()
   *
   * @return Get a vector of transforms for all links in the environment.
   */
  virtual vector_Affine3d getLinkTransforms() const = 0;

  /**
   * @brief Get the transform corresponding to the link.
   * @return Transform and is identity when no transform is available.
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

  /** @brief Get the allowed collision matrix */
  virtual AllowedCollisionMatrixConstPtr getAllowedCollisionMatrix() const = 0;

}; // class BasicEnvBase

typedef boost::shared_ptr<EnvBase> EnvBasePtr;
typedef boost::shared_ptr<const EnvBase> EnvBaseConstPtr;

class BasicEnv : public EnvBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief calcDistances Should return distance information for all links in list req.link_names (Discrete Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcDistancesDiscrete(const ContactRequest &req, ContactResultVector &dists) const = 0;

  /**
   * @brief calcDistances Should return distance information for all links in list link_names (Continuous Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcDistancesContinuous(const ContactRequest &req, ContactResultVector &dists) const = 0;

  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Discrete Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsDiscrete(const ContactRequest &req, ContactResultVector &collisions) const = 0;

  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Continuous Check)
   * @param req   The distance request information.
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsContinuous(const ContactRequest &req, ContactResultVector &collisions) const = 0;

  /**
   * @brief continuousCollisionCheckTrajectory Should perform a continuous collision check over the trajectory
   * and return every collision contact
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param link_names Name of the links to calculate collision data for.
   * @param traj The joint values at each time step
   * @param collisions The return collision data.
   * @return True if collision was found, otherwise false.
   */
  virtual bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, ContactResultVector& collisions) const = 0;

  /**
   * @brief continuousCollisionCheckTrajectory Should perform a continuous collision check over the trajectory
   * and stop on first collision.
   * @param joint_names JointNames corresponding to the values in traj (must be in same order)
   * @param link_names Name of the links to calculate collision data for.
   * @param traj The joint values at each time step
   * @param collision The return collision data.
   * @return True if collision was found, otherwise false.
   */
  virtual bool continuousCollisionCheckTrajectory(const std::vector<std::string> &joint_names, const std::vector<std::string> &link_names, const TrajArray& traj, ContactResult& collision) const = 0;

}; // class BasicEnv
typedef boost::shared_ptr<BasicEnv> BasicEnvPtr;
typedef boost::shared_ptr<const BasicEnv> BasicEnvConstPtr;

/**
 * @brief This should keep a single copy of environment and provide a way to update its state.
 *
 * When the calcDistance and calcColliiosn is requested it checks the current state and returns.
 */
class BasicEnvSingleton : public EnvBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Set the active contact request information
   * @param req ContactRequest information
   */
  virtual void setContactRequest(const ContactRequestBase &req) = 0;

  /**
   * @brief Get the active contact request information
   * @return Active contact request information
   */
  virtual const ContactRequestBase& getContactRequest() const = 0;

  /**
   * @brief calcDistances Should return distance information for all links in list req.link_names (Discrete Check)
   * @param dists A list of distance results.
   */
  virtual void calcDistancesDiscrete(ContactResultVector &dists) = 0;


  /**
   * @brief calcCollisions Should return collision information for all links in list link_names (Discrete Check)
   * @param dists A list of distance results.
   */
  virtual void calcCollisionsDiscrete(ContactResultVector &collisions) = 0;

};// class BasicEnvSingleton
typedef boost::shared_ptr<BasicEnvSingleton> BasicEnvSingletonPtr;
typedef boost::shared_ptr<const BasicEnvSingleton> BasicEnvSingletonConstPtr;

} //namespace tesseract

#endif // TESSERACT_CORE_BASIC_ENV_H
