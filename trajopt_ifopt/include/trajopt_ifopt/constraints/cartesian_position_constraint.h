/**
 * @file cartesian_position_constraint.h
 * @brief The cartesian position constraint
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_POSITION_CONSTRAINT_H
#define TRAJOPT_IFOPT_CARTESIAN_POSITION_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/core/constraint_set.h>
#include <tesseract/common/eigen_types.h>
#include <tesseract/common/types.h>
#include <tesseract/kinematics/fwd.h>

namespace trajopt_ifopt
{
class Var;

/**
 * @brief Holds the pose of a source frame at a target frame, both on links of a joint group.
 *
 * The six rows are the pose of one frame in the coordinates of the other: the translation, then the rotation vector.
 * They are in the coordinates of the target, unless the target is the only frame the joints move.
 *
 * Both frames are evaluated in one state, or each in a state of its own. With two states, the source frame of one
 * state is held at the target frame of the other. To keep a state on an axis a of the pose another state reaches,
 * as for the approach to a pose reached with a free rotation, give both frames the same link and offset O, make the
 * approaching state the source and give the target the offset O Translation(d a). A target fixed for the
 * approaching state follows the nominal pose only. Either state may come first in the variable set, and the
 * rotation rows need the rotation between the two frames to stay below half a turn.
 */
class CartPosConstraint : public ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartPosConstraint>;
  using ConstPtr = std::shared_ptr<const CartPosConstraint>;

  CartPosConstraint(std::shared_ptr<const Var> position_var,
                    std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                    tesseract::common::LinkId source_frame,
                    tesseract::common::LinkId target_frame,
                    const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                    const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                    std::string name = "CartPos",
                    RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  CartPosConstraint(std::shared_ptr<const Var> position_var,
                    const Eigen::VectorXd& coeffs,
                    const std::vector<Bounds>& bounds,
                    std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                    tesseract::common::LinkId source_frame,
                    tesseract::common::LinkId target_frame,
                    const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                    const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                    std::string name = "CartPos",
                    RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  /**
   * @brief The source frame in one state and the target frame in another
   * @param source_var Joint values of the state of the source frame
   * @param target_var Joint values of the state of the target frame; @p source_var again gives the one-state form
   */
  CartPosConstraint(std::shared_ptr<const Var> source_var,
                    std::shared_ptr<const Var> target_var,
                    std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                    tesseract::common::LinkId source_frame,
                    tesseract::common::LinkId target_frame,
                    const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                    const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                    std::string name = "CartPos",
                    RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  /** @brief The two-state form with a coefficient and bounds for each of the six rows */
  CartPosConstraint(std::shared_ptr<const Var> source_var,
                    std::shared_ptr<const Var> target_var,
                    const Eigen::VectorXd& coeffs,
                    const std::vector<Bounds>& bounds,
                    std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                    tesseract::common::LinkId source_frame,
                    tesseract::common::LinkId target_frame,
                    const Eigen::Isometry3d& source_frame_offset = Eigen::Isometry3d::Identity(),
                    const Eigen::Isometry3d& target_frame_offset = Eigen::Isometry3d::Identity(),
                    std::string name = "CartPos",
                    RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  int update() override { return rows_; }

  /**
   * @brief CalcValues Calculates the values associated with the constraint
   * @param joint_vals Input joint values for which FK is solved
   * @return Error of FK solution from target, one entry per row of the constraint. Position rows come before
   * orientation rows.
   */
  Eigen::VectorXd calcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /**
   * @brief The values with the source frame at @p source_vals and the target frame at @p target_vals
   * @pre The constraint is on two states
   */
  Eigen::VectorXd calcValues(const Eigen::Ref<const Eigen::VectorXd>& source_vals,
                             const Eigen::Ref<const Eigen::VectorXd>& target_vals) const;

  /** @brief Returns the error of FK solution from target at the current values of the variables, as calcValues */
  Eigen::VectorXd getValues() const override;

  /** @copydoc Differentiable::getCoefficients */
  Eigen::VectorXd getCoefficients() const override;

  /**
   * @brief  Returns the "bounds" of this constraint. How these are enforced is up to the solver
   * @return Returns the "bounds" of this constraint
   */
  std::vector<Bounds> getBounds() const override;

  /**
   * @brief Fills the jacobian block associated with the constraint
   * @param jac_block Block of the overall jacobian associated with these constraints
   * @param joint_vals Joint values at which every state of the constraint is evaluated
   */
  void calcJacobianBlock(Jacobian& jac_block, const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const;

  /** @brief Get the jacobian */
  Jacobian getJacobian() const override;

  /**
   * @brief Set the target frame offset
   * @param target_frame_offset The pose of the target frame in the coordinates of the target link
   */
  void setTargetPose(const Eigen::Isometry3d& target_frame_offset);

  /**
   * @brief Returns the target frame offset
   * @return The pose of the target frame in the coordinates of the target link, not in world coordinates
   */
  Eigen::Isometry3d getTargetPose() const;

  /**
   * @brief Returns the current TCP pose in world frame given the input kinematic info and the current variable values
   * @return The current TCP pose given the input kinematic info and the current variable values
   */
  Eigen::Isometry3d getCurrentPose() const;

  /**
   * @brief If true, find the jacobian by perturbing each joint; if false, from the jacobian of the kinematics.
   * Default: true
   */
  bool use_numeric_differentiation{ true };

private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief The constraint coefficients */
  Eigen::VectorXd coeffs_;

  /** @brief Bounds on each row of the constraint */
  std::vector<Bounds> bounds_;

  /** @brief The joint values of the state of the source frame */
  std::shared_ptr<const Var> source_var_;

  /** @brief The joint values of the state of the target frame; source_var_ in the one-state form */
  std::shared_ptr<const Var> target_var_;

  /** @brief True if the two frames are in two states */
  bool two_states_{ false };

  /** @brief Policy for representing range bounds when building the constraint rows. */
  RangeBoundHandling range_bound_handling_{ RangeBoundHandling::kSplitToTwoInequalities };

  /** @brief The joint group */
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip_;

  /** @brief Link which should reach desired pos */
  tesseract::common::LinkId source_frame_;

  /** @brief The target frame that should be reached by the source */
  tesseract::common::LinkId target_frame_;

  /** @brief Static transform applied to the source_frame location */
  Eigen::Isometry3d source_frame_offset_;

  /** @brief Static transform applied to the target_frame location */
  Eigen::Isometry3d target_frame_offset_;

  /** @brief True if the rows are the pose of the target in the coordinates of the source: only the target moves */
  bool rows_in_source_{ false };

  /** @brief True if the joints of the group move both frames */
  bool base_moves_{ false };

  /** @brief True if both frames move in two states, so each state has a block of the jacobian */
  bool base_block_{ false };

  /**
   * @brief This is a vector of indices to be returned Default: {0, 1, 2, 3, 4, 5}
   *
   * If you only care about x, y and z error, this is {0, 1, 2}
   * If you only care about rotation error around x, y and z, this is {3, 4, 5}
   */
  Eigen::VectorXi indices_;

  /** @brief True if the indices name a rotation row */
  bool rotation_rows_{ false };

  /** @brief The poses of the links of the two frames */
  void calcLinkPoses(Eigen::Isometry3d& source_link_tf,
                     Eigen::Isometry3d& target_link_tf,
                     const Eigen::Ref<const Eigen::VectorXd>& source_vals,
                     const Eigen::Ref<const Eigen::VectorXd>& target_vals) const;

  /** @brief Fills the jacobian with the source frame at @p source_vals and the target frame at @p target_vals */
  void fillJacobianBlock(Jacobian& jac_block,
                         const Eigen::Ref<const Eigen::VectorXd>& source_vals,
                         const Eigen::Ref<const Eigen::VectorXd>& target_vals) const;

  /** @brief The kept rows of the six */
  Eigen::VectorXd keptRows(const Eigen::Ref<const Eigen::VectorXd>& all_rows) const;

  /** @brief Maps the angular velocity of the moving frame in base coordinates to the rate of the rotation rows */
  Eigen::Matrix3d calcRateMap(const Eigen::Isometry3d& base_tf, const Eigen::Isometry3d& moving_tf) const;

  /**
   * @brief The rates of the kept rows
   * @param twists Twists at the moving frame, in world coordinates: of the moving frame relative to the link of the
   * base frame in one state, or of the link of one frame alone with two states
   * @param base_tf The pose of the base frame
   * @param rate_map From calcRateMap
   */
  Eigen::MatrixXd toRowRates(Eigen::MatrixXd twists,
                             const Eigen::Isometry3d& base_tf,
                             const Eigen::Matrix3d& rate_map) const;

  static thread_local tesseract::common::LinkIdTransformMap transforms_cache_;  // NOLINT
};
}  // namespace trajopt_ifopt
#endif
