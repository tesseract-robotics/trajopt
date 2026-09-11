/**
 * @file collision_utils.h
 * @brief Contains utility functions used by collision constraints
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date Nov 24, 2020
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
#ifndef TRAJOPT_COMMON_COLLISION_UTILS_H
#define TRAJOPT_COMMON_COLLISION_UTILS_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <array>
#include <tesseract/collision/types.h>
#include <tesseract/kinematics/fwd.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
struct GradientResults;

std::size_t getHash(const void* parent, const Eigen::Ref<const Eigen::VectorXd>& dof_vals);
std::size_t getHash(const void* parent,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

// If this works we will store the shape hash with the shape so it is not calculated everytime
std::size_t cantorHash(int shape_id, int subshape_id);

/**
 * @brief Remove any results that are invalid.
 * Invalid state are contacts that occur at fixed states or have distances outside the threshold.
 * @param contact_results Contact results vector to process.
 * @param margin The contact margin
 * @param margin The contact margin buffer
 * @param var0_fixed Indicates if the var0 is a fixed state
 * @param var1_fixed Indicates if the var1 is a fixed state
 */
void removeInvalidContactResults(tesseract::collision::ContactResultVector& contact_results,
                                 double margin,
                                 double margin_buffer,
                                 bool var0_fixed,
                                 bool var1_fixed);

/** @brief The part of a segment a contact was found in, as fractions of the segment from its start */
struct ContactInterval
{
  double start{ 0.0 };
  double end{ 1.0 };
};

/**
 * @brief How one link's gradient splits between the two ends of its contact interval
 * @details The link's contact point is modelled as the fixed blend, at the contact's fraction of the interval, of the
 * point the link carries at the interval's two end states. start_a and start_b are the weights the segment start's
 * gradient gives the jacobians at the interval's start and end states; end_a and end_b are those of the segment end's.
 * The start weights sum to 1 - cc_time and the end weights to cc_time when cc_time lies in the interval.
 */
struct IntervalWeights
{
  double start_a{ 0.0 };
  double start_b{ 0.0 };
  double end_a{ 0.0 };
  double end_b{ 0.0 };
};

/**
 * @brief The number of equal casts a continuous check splits a segment into
 * @return ceil(segment_length / longest_valid_segment_length) under LVS_CONTINUOUS when the segment is longer than the
 * longest valid segment length, otherwise 1; also 1 when the longest valid segment length is not positive
 */
long castCount(const tesseract::collision::CollisionCheckConfig& config, double segment_length);

/**
 * @brief The cast of a segment a contact at @p cc_time was found in
 * @param cc_time The contact time as a fraction of the segment
 * @param cast_count The number of equal casts the segment was checked with, or 0 for a check at interpolated states,
 * where a contact is a point in time
 * @return The cast holding @p cc_time, or [cc_time, cc_time] when @p cast_count is 0. A time on the boundary of two
 * casts may be placed in either; both give the same gradient, the contact then sitting at the state they share.
 */
ContactInterval contactInterval(double cc_time, long cast_count);

/**
 * @brief Split one link's gradient between the ends of its contact interval
 * @param cc_time The contact time as a fraction of the segment. A time outside @p interval is placed at its nearest
 * end.
 */
IntervalWeights intervalWeights(double cc_time, const ContactInterval& interval);

/**
 * @brief Extracts the gradient information based on the contact results
 * @param dofvals The joint values
 * @param contact_result The contact results to compute the gradient
 * @param data Data associated with the link pair the contact results associated with.
 * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
 * @return The gradient results
 */
void getGradient(GradientResults& results,
                 const Eigen::VectorXd& dofvals,
                 const tesseract::collision::ContactResult& contact_result,
                 double margin,
                 double margin_buffer,
                 const tesseract::kinematics::JointGroup& manip);

/**
 * @brief Extracts the gradient information based on the contact results
 * @param dofvals The joint values
 * @param contact_result The contact results to compute the gradient
 * @param data Data associated with the link pair the contact results associated with.
 * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
 * @return The gradient results
 */
void getGradient(GradientResults& results,
                 const Eigen::VectorXd& dofvals0,
                 const Eigen::VectorXd& dofvals1,
                 const tesseract::collision::ContactResult& contact_result,
                 double margin,
                 double margin_buffer,
                 const tesseract::kinematics::JointGroup& manip);

/**
 * @brief Print debug gradient information
 * @param res Contact Results
 * @param dist_grad_A Object A gradient
 * @param dist_grad_B Object B gradient
 * @param dof_vals The joint values
 * @param header If true, header is printed
 */
void debugPrintInfo(const tesseract::collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header = false);

}  // namespace trajopt_common
#endif  // TRAJOPT_COMMON_COLLISION_UTILS_H
