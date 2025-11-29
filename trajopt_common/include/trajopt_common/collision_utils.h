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
#include <tesseract_collision/core/types.h>
#include <tesseract_kinematics/core/fwd.h>
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
void removeInvalidContactResults(tesseract_collision::ContactResultVector& contact_results,
                                 double margin,
                                 double margin_buffer,
                                 bool var0_fixed,
                                 bool var1_fixed);

/**
 * @brief Extracts the gradient information based on the contact results
 * @param dofvals The joint values
 * @param contact_result The contact results to compute the gradient
 * @param data Data associated with the link pair the contact results associated with.
 * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
 * @return The gradient results
 */
GradientResults getGradient(const Eigen::VectorXd& dofvals,
                            const tesseract_collision::ContactResult& contact_result,
                            double margin,
                            double margin_buffer,
                            const tesseract_kinematics::JointGroup& manip);

/**
 * @brief Extracts the gradient information based on the contact results
 * @param dofvals The joint values
 * @param contact_result The contact results to compute the gradient
 * @param data Data associated with the link pair the contact results associated with.
 * @param isTimestep1 Indicates if this is the second timestep when computing gradient for continuous collision
 * @return The gradient results
 */
GradientResults getGradient(const Eigen::VectorXd& dofvals0,
                            const Eigen::VectorXd& dofvals1,
                            const tesseract_collision::ContactResult& contact_result,
                            double margin,
                            double margin_buffer,
                            const tesseract_kinematics::JointGroup& manip);

/**
 * @brief Print debug gradient information
 * @param res Contact Results
 * @param dist_grad_A Object A gradient
 * @param dist_grad_B Object B gradient
 * @param dof_vals The joint values
 * @param header If true, header is printed
 */
void debugPrintInfo(const tesseract_collision::ContactResult& res,
                    const Eigen::VectorXd& dist_grad_A,
                    const Eigen::VectorXd& dist_grad_B,
                    const Eigen::VectorXd& dof_vals,
                    bool header = false);

}  // namespace trajopt_common
#endif  // TRAJOPT_COMMON_COLLISION_UTILS_H
