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
#include <map>
#include <utility>
#include <vector>
#include <tesseract/collision/types.h>
#include <tesseract/kinematics/fwd.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_types.h>

namespace trajopt_common
{
std::size_t getHash(const void* parent, const Eigen::Ref<const Eigen::VectorXd>& dof_vals);
std::size_t getHash(const void* parent,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals0,
                    const Eigen::Ref<const Eigen::VectorXd>& dof_vals1);

/**
 * @brief Get the key that groups the contacts of a link pair by shape pair.
 * Contacts share a key exactly when they are on the same shape and subshape of each link, whichever link the
 * contact reports first.
 * @param contact The contact to get the key for
 * @return The shape pair key
 */
ShapePairKey getShapePairKey(const tesseract::collision::ContactResult& contact);

/**
 * @brief Group the contacts of a link pair by shape pair and append one gradient results set per group.
 * @param sets The sets to append to. The new sets are appended in shape pair key order.
 * @param link_pair The link pair the contacts belong to
 * @param contacts The contacts of the link pair
 * @param coeff The collision coeff of the link pair
 * @param is_continuous Indicate if the contacts are from a continuous contact checker
 * @param calc_gradient Called as calc_gradient(GradientResults&, const ContactResult&) to fill in the gradient
 * results of a contact
 */
template <typename CalcGradientFn>
void appendGradientResultsSets(std::vector<GradientResultsSet>& sets,
                               const tesseract::common::LinkIdPair& link_pair,
                               const tesseract::collision::ContactResultVector& contacts,
                               double coeff,
                               bool is_continuous,
                               const CalcGradientFn& calc_gradient)
{
  std::map<ShapePairKey, GradientResultsSet> shape_grs;
  for (const tesseract::collision::ContactResult& contact : contacts)
  {
    const ShapePairKey shape_key = getShapePairKey(contact);

    auto [it, inserted] = shape_grs.try_emplace(shape_key);
    GradientResultsSet& grs = it->second;

    if (inserted)
    {
      grs.key = link_pair;
      grs.shape_key = shape_key;
      grs.coeff = coeff;
      grs.is_continuous = is_continuous;
      grs.results.reserve(contacts.size());
    }

    GradientResults grad;
    calc_gradient(grad, contact);
    grs.add(std::move(grad));
  }

  for (auto& kv : shape_grs)
    sets.emplace_back(std::move(kv.second));
}

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
