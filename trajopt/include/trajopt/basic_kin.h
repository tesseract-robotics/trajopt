/**
 * @file basic_kin.h
 * @brief Basic low-level kinematics functions.
 *
 * @author dsolomon
 * @date Sep 15, 2013
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
#ifndef BASIC_KIN_H
#define BASIC_KIN_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <trajopt/common.hpp>

namespace trajopt
{

/** @brief Basic low-level kinematics functions. */
class TRAJOPT_API BasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  BasicKin() : initialized_(false) {}

  /**
   * @brief Calculates tool pose of robot chain
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param pose Transform of end-of-tip relative to root
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose) const = 0;

  /**
   * @brief Calculates pose for a given link
   * @param joint_angles Vector of joint angles (size must match number of joints in robot chain)
   * @param pose Transform of link relative to root
   * @param link_name Name of link to calculate pose
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(const Eigen::VectorXd &joint_angles, Eigen::Affine3d &pose, const std::string &link_name) const = 0;

  /**
   * @brief Creates chain and calculates tool pose relative to root
   * New chain is not stored permanently, but subchain_fk_solver_ is updated
   * @param joint_angles Vector of joint angles (size must match number of joints in chain)
   * @param base Name of base link for new chain
   * @param tip Name of tip link for new chain
   * @param pose Transform of end-of-tip relative to base
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcFwdKin(const Eigen::VectorXd &joint_angles, const std::string &base, const std::string &tip, Eigen::Affine3d &pose) const = 0;

  /**
   * @brief Calculated jacobian of robot given joint angles
   * @param joint_angles Input vector of joint angles
   * @param jacobian Output jacobian
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian) const = 0;

  /**
   * @brief Calculated jacobian at a link given joint angles
   * @param joint_angles Input vector of joint angles
   * @param jacobian Output jacobian for a given link
   * @param link_name Name of link to calculate jacobian
   * @return True if calculation successful, False if anything is wrong (including uninitialized BasicKin)
   */
  virtual bool calcJacobian(const Eigen::VectorXd &joint_angles, Eigen::MatrixXd &jacobian, const std::string &link_name) const = 0;

  /**
   * @brief Checks if BasicKin is initialized (init() has been run: urdf model loaded, etc.)
   * @return True if init() has completed successfully
   */
  virtual bool checkInitialized() const { return initialized_; }

  /**
   * @brief Check for consistency in # and limits of joints
   * @param vec Vector of joint values
   * @return True if size of vec matches # of robot joints and all joints are within limits
   */
  virtual bool checkJoints(const Eigen::VectorXd &vec) const = 0;

  /**
   * @brief Get list of joint names for robot
   * @param names Output vector of joint names, copied from joint_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  virtual bool getJointNames(std::vector<std::string> &names) const = 0;

  /**
   * @brief Getter for joint_limits_
   * @return Matrix of joint limits
   */
  virtual Eigen::MatrixXd getLimits() const = 0;

  /**
   * @brief Get list of link names for robot
   * @param names Output vector of names, copied from link_list_ created in init()
   * @return True if BasicKin has been successfully initialized
   */
  virtual bool getLinkNames(std::vector<std::string> &names) const = 0;

  /**
   * @brief Initializes BasicKin
   * @return True if init() completes successfully
   */
  virtual bool init() = 0;

  /**
   * @brief Number of joints in robot
   * @return Number of joints in robot
   */
  virtual unsigned int numJoints() const = 0;

  /** @brief getter for the robot base link name */
  virtual std::string getRobotBaseLinkName() const = 0;

  /** @brief getter for the robot tip link name */
  virtual std::string getRobotTipLinkName() const = 0;

  /**
   * @brief Assigns values from another BasicKin to this
   * @param rhs Input BasicKin object to copy from
   * @return reference to this BasicKin object
   */
  virtual BasicKin& operator=(const BasicKin& rhs) = 0;

  /**
   * @brief Solve equation Ax=b for x
   * Use this SVD to compute A+ (pseudoinverse of A). Weighting still TBD.
   * @param A Input matrix (represents Jacobian)
   * @param b Input vector (represents desired pose)
   * @param x Output vector (represents joint values)
   * @return True if solver completes properly
   */
  static bool solvePInv(const Eigen::MatrixXd &A, const Eigen::VectorXd &b, Eigen::VectorXd &x)
  {
    const double eps = 0.00001;  // TODO: Turn into class member var
    const double lambda = 0.01;  // TODO: Turn into class member var

    if ( (A.rows() == 0) || (A.cols() == 0) )
    {
      std::cerr << "Empty matrices not supported in solvePinv()";
      return false;
    }

    if ( A.rows() != b.size() )
    {
      std::cerr << "Matrix size mismatch: A(" << A.rows() << "," << A.cols() << "), b(" << b.size() << ")";
      return false;
    }

    //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
    //in order to solve Ax=b -> x*=A+ b
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd &U = svd.matrixU();
    const Eigen::VectorXd &Sv = svd.singularValues();
    const Eigen::MatrixXd &V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    Eigen::VectorXd inv_Sv(nSv);
    for(size_t i=0; i<nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
        inv_Sv(i) = 1/Sv(i);
      else
        inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
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
  static bool dampedPInv(const Eigen::MatrixXd &A, Eigen::MatrixXd &P, const double eps = 0.011, const double lambda = 0.01)
  {
    if ( (A.rows() == 0) || (A.cols() == 0) )
    {
      std::cerr << "Empty matrices not supported in dampedPInv()";
      return false;
    }

    //Calculate A+ (pseudoinverse of A) = V S+ U*, where U* is Hermition of U (just transpose if all values of U are real)
    //in order to solve Ax=b -> x*=A+ b
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const Eigen::MatrixXd &U = svd.matrixU();
    const Eigen::VectorXd &Sv = svd.singularValues();
    const Eigen::MatrixXd &V = svd.matrixV();

    // calculate the reciprocal of Singular-Values
    // damp inverse with lambda so that inverse doesn't oscillate near solution
    size_t nSv = Sv.size();
    Eigen::VectorXd inv_Sv(nSv);
    for(size_t i=0; i<nSv; ++i)
    {
      if (fabs(Sv(i)) > eps)
        inv_Sv(i) = 1/Sv(i);
      else
      {
        inv_Sv(i) = Sv(i) / (Sv(i)*Sv(i) + lambda*lambda);
      }
    }
    P = V * inv_Sv.asDiagonal() * U.transpose();
    return true;
  }

private:
  bool initialized_; /**< Identifies if the object has been initialized */

}; // class BasicKin

typedef boost::shared_ptr<BasicKin> BasicKinPtr;
} // namespace trajopt

#endif // BASIC_KIN_H

