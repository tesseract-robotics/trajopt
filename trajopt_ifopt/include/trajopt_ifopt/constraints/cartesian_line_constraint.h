/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @author Colin Lewis
 * @date December 27, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_LINE_CONSTRAINT_H
#define TRAJOPT_IFOPT_CARTESIAN_LINE_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/constraint_set.h>

#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
/**
 * @brief Contains kinematic information for the cartesian position cost; include cart point .h & remove?
 */
struct CartLineInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartLineInfo>;
  using ConstPtr = std::shared_ptr<const CartLineInfo>;

  CartLineInfo() = default;
  CartLineInfo(
      tesseract_kinematics::JointGroup::ConstPtr manip,
      std::string source_frame,
      std::string target_frame,
      const Eigen::Isometry3d& target_frame_offset1,
      const Eigen::Isometry3d& target_frame_offset2,
      const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
      const Eigen::VectorXi& indices = Eigen::Matrix<int, 1, 6>(std::vector<int>({ 0, 1, 2, 3, 4, 5 }).data()));

  /** @brief The joint group */
  tesseract_kinematics::JointGroup::ConstPtr manip;

  /** @brief Link which should reach desired pos */
  std::string source_frame;

  /** @brief The target frame that should be reached by the source */
  std::string target_frame;

  /** @brief Static transform applied to the source_frame location */
  Eigen::Isometry3d source_frame_offset;

  /** @brief Static transform applied to the target_frame location defining the starting point of the line */
  Eigen::Isometry3d target_frame_offset1;

  /** @brief Static transform applied to the target_frame location defining the ending point of the line */
  Eigen::Isometry3d target_frame_offset2;

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices;
};

/**
 * @brief The CartLineConstraint class
 */
class CartLineConstraint : public ifopt::ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartLineConstraint>;
  using ConstPtr = std::shared_ptr<const CartLineConstraint>;

  CartLineConstraint(CartLineInfo info,
                     JointPosition::ConstPtr position_var,
                     const Eigen::VectorXd& coeffs,
                     const std::string& name = "CartLine");

  /**
   * @brief CalcValues Calculates the values associated with the constraint
   * @param joint_vals Input joint values for which FK is solved
   * @return Error of FK solution from target, size 6. The first 3 terms are associated with position and are currently
   * the only values honored for the linear model
   * */
  Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;
  /**
   * @brief Returns the values associated with the constraint. In this case it should be the
   * joint values placed along the line should be n_dof_ * n_vars_ long
   * @return
   */
  Eigen::VectorXd GetValues() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Returns the "bounds" of this constraint
   */
  std::vector<ifopt::Bounds> GetBounds() const override;

  void SetBounds(const std::vector<ifopt::Bounds>& bounds);

  /**
   * @brief Fills the jacobian block associated with the constraint
   * @param jac_block Block of the overall jacobian associated with these constraints
   */
  void CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals, Jacobian& jac_block) const;
  /**
   * @brief Fills the jacobian block associated with the given var_set.
   * @param var_set Name of the var_set to which the jac_block is associated
   * @param jac_block Block of the overall jacobian associated with these constraints and the var_set variable
   */
  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const override;

  /**
   * @brief Get the two poses defining the line
   * @return A std::pair of poses
   */
  std::pair<Eigen::Isometry3d, Eigen::Isometry3d> GetLine() const;

  /**
   * @brief Gets the kinematic info used to create this constraint
   * @return The kinematic info used to create this constraint
   */
  const CartLineInfo& GetInfo() const;

  /** @brief If true, numeric differentiation will be used. Default: true
   *
   * Note: While the logic for using the jacobian from KDL will be used if set to false, this has been buggy. Set this
   * to false at your own risk.
   */
  bool use_numeric_differentiation{ true };

  /**
   * @brief GetLinePoint Finds the nearest point on the line between Isometry a,b
   * to a test point
   * @param source_tf input location, orientation to compare to the line
   * note that only cartesian proximity is used to determine nearness;
   * LinePoint orientation is determined by a SLERP between Isometry a, b
   * @param target_tf1 The location of the start of the line
   * @param target_tf2 The location of the end of the line
   * @return The nearest point on the line to the source_tf
   */
  Eigen::Isometry3d GetLinePoint(const Eigen::Isometry3d& source_tf,
                                 const Eigen::Isometry3d& target_tf1,
                                 const Eigen::Isometry3d& target_tf2) const;

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief The constraint coefficients */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on the positions of each joint */
  std::vector<ifopt::Bounds> bounds_;

  /** @brief Pointers to the vars used by this constraint.
   *
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()
   */
  JointPosition::ConstPtr position_var_;

  /** @brief The cartesian line information used when calculating error */
  CartLineInfo info_;
};
};  // namespace trajopt_ifopt
#endif
