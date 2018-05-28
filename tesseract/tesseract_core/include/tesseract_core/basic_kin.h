/**
 * @file basic_kin.h
 * @brief Basic low-level kinematics functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2013, Southwest Research Institute
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
#ifndef TESSERACT_CORE_BASIC_KIN_H
#define TESSERACT_CORE_BASIC_KIN_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <memory>
#include <tesseract_core/basic_types.h>

namespace tesseract
{
/**
 * @brief Basic low-level kinematics functions.
 *
 * All data should be returned or provided relative to the base link of the kinematic chain.
 *
 */
class BasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicKin() {}
  /**
   * @brief Calculates tool pose of robot chain
   * @param pose Transform of end-of-tip relative to root
   * @param change_base The transform from the base frame of the manipulator to the desired frame.
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(Eigen::Affine3d& pose,
                          const Eigen::Affine3d& change_base,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculates pose for a given link
   * @param pose Transform of link relative to root
   * @param change_base The transform from the base frame of the manipulator to the desired frame.
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param link_name Name of link to calculate pose
   * @param state The state of the environment
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(Eigen::Affine3d& pose,
                          const Eigen::Affine3d& change_base,
                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                          const std::string& link_name,
                          const EnvState& state) const = 0;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param jacobian Output jacobian
   * @param change_base The transform from the base frame of the manipulator to the desired frame.
   * @param joint_angles Input vector of joint angles
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Affine3d& change_base,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles) const = 0;

  /**
   * @brief Calculated jacobian at a link given joint angles
   * @param jacobian Output jacobian for a given link
   * @param change_base The transform from the base frame of the manipulator to the desired frame.
   * @param joint_angles Input vector of joint angles
   * @param link_name Name of link to calculate jacobian
   * @param state The state of the environment
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Affine3d& change_base,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                            const std::string& link_name,
                            const EnvState& state) const = 0;

  /**
   * @brief Calculated jacobian at a link given joint angles
   * @param jacobian Output jacobian for a given link
   * @param change_base The transform from the base frame of the manipulator to the desired frame.
   * @param joint_angles Input vector of joint angles
   * @param link_name Name of link to calculate jacobian
   * @param state The state of the environment
   * @param link_point Point in the link_name frame for which to calculate the jacobian about
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(Eigen::Ref<Eigen::MatrixXd> jacobian,
                            const Eigen::Affine3d& change_base,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                            const std::string& link_name,
                            const EnvState& state,
                            const Eigen::Ref<const Eigen::Vector3d>& link_point) const = 0;

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  virtual bool checkJoints(const Eigen::Ref<const Eigen::VectorXd>& vec) const = 0;

  /**
   * @brief Get list of joint names for robot
   * @param names Output vector of joint names, copied from joint_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  virtual const std::vector<std::string>& getJointNames() const = 0;

  /**
   * @brief Get list of all link names (with and without geometry) for robot
   * @param names Output vector of names, copied from link_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  virtual const std::vector<std::string>& getLinkNames() const = 0;

  /**
   * @brief Getter for joint_limits_
   * @return Matrix of joint limits
   */
  virtual const Eigen::MatrixX2d& getLimits() const = 0;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  virtual unsigned int numJoints() const = 0;

  /** @brief getter for the robot base link name */
  virtual const std::string& getBaseLinkName() const = 0;

  /** @brief getter for the robot tip link name */
  virtual const std::string& getTipLinkName() const = 0;

  /** @brief Name of the maniputlator */
  virtual const std::string& getName() const = 0;

  /**
   * @brief This is used to keep the internal data updated when links are attached in the environment
   * @param link_name Name of link being attached
   * @param parent_link_name The name of the link that link_name is attaching too.
   */
  virtual void addAttachedLink(const std::string& link_name, const std::string& parent_link_name) = 0;

  /**
   * @brief This is used to keep the interanl data updated when links are detached in the environment
   * @param link_name The name of the attached link
   */
  virtual void removeAttachedLink(const std::string& link_name) = 0;

  /** @brief Clear all attached links from manipulator */
  virtual void clearAttachedLinks() = 0;

  /**
   * @brief Solve equation Ax=b for x
   * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
   * @param A Input matrix (represents Jacobian)
   * @param b Input vector (represents desired pose)
   * @param x Output vector (represents joint values)
   * @return True if solver completes properly
   */
  static bool solvePInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                        const Eigen::Ref<const Eigen::VectorXd>& b,
                        Eigen::Ref<Eigen::VectorXd> x)
  {
    const double eps = 0.00001;  // TODO: Turn into class member var
    const double lambda = 0.01;  // TODO: Turn into class member var

    if ((A.rows() == 0) || (A.cols() == 0))
    {
      std::cerr << "Empty matrices not supported in solvePinv()";
      return false;
    }

    if (A.rows() != b.size())
    {
      std::cerr << "Matrix size mismatch: A(" << A.rows() << "," << A.cols() << "), b(" << b.size() << ")";
      return false;
    }

    // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
    // real)
    // in order to solve Ax=b -> x*=A+ b
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd& U = svd.matrixU();
    const Eigen::VectorXd& Sv = svd.singularValues();
    const Eigen::MatrixXd& V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    Eigen::VectorXd inv_Sv(nSv);
    for (size_t i = 0; i < nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
        inv_Sv(i) = 1 / Sv(i);
      else
        inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
    }
    x = V * inv_Sv.asDiagonal() * U.transpose() * b;
    return true;
  }

  /**
   * @brief Calculate Damped Pseudoinverse
   * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
   * @param A Input matrix (represents Jacobian)
   * @param P Output matrix (represents pseudoinverse of A)
   * @param eps Singular value threshold
   * @param lambda Damping factor
   * @return True if Pseudoinverse completes properly
   */
  static bool dampedPInv(const Eigen::Ref<const Eigen::MatrixXd>& A,
                         Eigen::Ref<Eigen::MatrixXd> P,
                         const double eps = 0.011,
                         const double lambda = 0.01)
  {
    if ((A.rows() == 0) || (A.cols() == 0))
    {
      std::cerr << "Empty matrices not supported in dampedPInv()";
      return false;
    }

    // Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are
    // real)
    // in order to solve Ax=b -> x*=A+ b
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd& U = svd.matrixU();
    const Eigen::VectorXd& Sv = svd.singularValues();
    const Eigen::MatrixXd& V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    Eigen::VectorXd inv_Sv(nSv);
    for (size_t i = 0; i < nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
        inv_Sv(i) = 1 / Sv(i);
      else
      {
        inv_Sv(i) = Sv(i) / (Sv(i) * Sv(i) + lambda * lambda);
      }
    }
    P = V * inv_Sv.asDiagonal() * U.transpose();
    return true;
  }
};  // class BasicKin

typedef std::shared_ptr<BasicKin> BasicKinPtr;
typedef std::shared_ptr<const BasicKin> BasicKinConstPtr;
}  // namespace tesseract

#endif  // TESSERACT_CORE_BASIC_KIN_H
