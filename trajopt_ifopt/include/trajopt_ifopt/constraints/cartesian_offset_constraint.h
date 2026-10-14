/**
 * @file cartesian_offset_constraint.h
 * @brief Holds a frame of one state at an offset along an axis of the pose it reaches in another state
 *
 * @author Roelof Oomen
 * @date October 13, 2026
 *
 * @copyright Copyright (c) 2026
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
#ifndef TRAJOPT_IFOPT_CARTESIAN_OFFSET_CONSTRAINT_H
#define TRAJOPT_IFOPT_CARTESIAN_OFFSET_CONSTRAINT_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <memory>
#include <string>
#include <vector>
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
 * @brief Ties the pose of a frame in one state to the pose the same frame reaches in another state.
 *
 * The frame is a link of the joint group with a constant offset O. With q_r the joint values of the reference state,
 * q_f those of the follower state, a unit axis a in frame coordinates and a signed distance d
 *
 *   T_ref = FK(q_r) O Translation(d a)
 *   T_fol = FK(q_f) O
 *
 * the six rows are the pose of T_fol in the coordinates of T_ref: the translation R_ref^T (p_fol - p_ref), then the
 * rotation vector of R_ref^T R_fol. All rows zero means the follower frame has the orientation of the reference frame
 * and sits at d a in its coordinates.
 *
 * Where a pose is reached with a free rotation, this keeps the approach to it on an axis of the pose actually
 * reached; a target fixed for the approach follows the nominal pose only.
 *
 * The translation rows are in reference frame coordinates, so the distance along the axis is a row of its own only
 * when a is a coordinate axis of the frame. The rotation rows need the rotation between the two frames to stay below
 * half a turn.
 *
 * Either state may come first in the variable set. The jacobian is exact, for both states.
 */
class CartOffsetConstraint : public ConstraintSet
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<CartOffsetConstraint>;
  using ConstPtr = std::shared_ptr<const CartOffsetConstraint>;

  /**
   * @param reference_var Joint values of the state whose frame the offset is taken from
   * @param follower_var Joint values of the state whose frame is held at the offset
   * @param manip The joint group of both states
   * @param frame Link that carries the frame; one the joints of the group move
   * @param frame_offset The frame in link coordinates
   * @param axis Direction of the offset in frame coordinates; normalized here
   * @param distance Signed length of the offset along @p axis; zero makes the two frames coincide
   * @param coeffs Weight of each of the six rows, finite and non-negative; zero drops the row
   * @param bounds Bounds of each of the six rows
   * @param name Name of the constraint set
   * @param range_bound_handling Keep a row with a range bound as one row or split it in two
   * @throws std::runtime_error if the two variables are one, a variable is not of the size of the group, the link is
   * not a moving link of the group, the axis is zero or not finite, the distance is not finite, the coefficients are
   * not six finite, non-negative values, or the bounds are not six
   */
  CartOffsetConstraint(std::shared_ptr<const Var> reference_var,
                       std::shared_ptr<const Var> follower_var,
                       std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                       tesseract::common::LinkId frame,
                       const Eigen::Isometry3d& frame_offset,
                       const Eigen::Vector3d& axis,
                       double distance,
                       const Eigen::VectorXd& coeffs,
                       const std::vector<Bounds>& bounds,
                       std::string name = "CartOffset",
                       RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  /** @brief Six rows with unit coefficients, each an equality */
  CartOffsetConstraint(std::shared_ptr<const Var> reference_var,
                       std::shared_ptr<const Var> follower_var,
                       std::shared_ptr<const tesseract::kinematics::JointGroup> manip,
                       tesseract::common::LinkId frame,
                       const Eigen::Isometry3d& frame_offset,
                       const Eigen::Vector3d& axis,
                       double distance,
                       std::string name = "CartOffset",
                       RangeBoundHandling range_bound_handling = RangeBoundHandling::kSplitToTwoInequalities);

  int update() override { return rows_; }

  /**
   * @brief The pose of the follower frame in the coordinates of the offset reference frame
   * @param reference_vals Joint values of the reference state
   * @param follower_vals Joint values of the follower state
   * @return The kept rows of: three translations [m], then the rotation vector [rad]
   */
  Eigen::VectorXd calcValues(const Eigen::Ref<const Eigen::VectorXd>& reference_vals,
                             const Eigen::Ref<const Eigen::VectorXd>& follower_vals) const;

  /** @copydoc Differentiable::getValues */
  Eigen::VectorXd getValues() const override;

  /** @copydoc Differentiable::getCoefficients */
  Eigen::VectorXd getCoefficients() const override;

  /** @brief The bounds of the kept rows */
  std::vector<Bounds> getBounds() const override;

  /** @brief The jacobian for both states; each row has an entry for every joint of each */
  Jacobian getJacobian() const override;

  /** @brief The unit direction of the offset in frame coordinates */
  const Eigen::Vector3d& getAxis() const { return axis_; }

  /** @brief The signed length of the offset */
  double getDistance() const { return distance_; }

private:
  std::shared_ptr<const Var> reference_var_;
  std::shared_ptr<const Var> follower_var_;
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip_;
  tesseract::common::LinkId frame_;
  Eigen::Isometry3d frame_offset_;
  Eigen::Vector3d axis_;
  double distance_;

  /** @brief The frame offset followed by the translation along the axis */
  Eigen::Isometry3d reference_offset_;

  /** @brief The number of joints of the group */
  Eigen::Index n_dof_{ 0 };

  /** @brief For each kept row, which of the six rows it is */
  Eigen::VectorXi indices_;
  Eigen::VectorXd coeffs_;
  std::vector<Bounds> bounds_;

  /** @brief True if a rotation row is kept */
  bool rotation_rows_{ false };

  /** @brief The poses of the link in the two states */
  void calcLinkPoses(Eigen::Isometry3d& reference_link_tf,
                     Eigen::Isometry3d& follower_link_tf,
                     const Eigen::Ref<const Eigen::VectorXd>& reference_vals,
                     const Eigen::Ref<const Eigen::VectorXd>& follower_vals) const;

  static thread_local tesseract::common::LinkIdTransformMap transforms_cache_;  // NOLINT
};
}  // namespace trajopt_ifopt
#endif
