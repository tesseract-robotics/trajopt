/**
 * @file discrete_combine_collision_data.h
 * @brief A container for function which combine collision data
 *
 * @author Levi Armstrong
 * @date June 4, 2021
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#ifndef TRAJOPT_IFOPT_DISCRETE_COMBINE_COLLISION_DATA_H
#define TRAJOPT_IFOPT_DISCRETE_COMBINE_COLLISION_DATA_H

#include <trajopt_ifopt/constraints/collision_types.h>

namespace trajopt_ifopt
{
/**
 * @brief The DiscreteCombineCollisionData struct
 * @warning Make sure that the values returned are not just the violation but the constraint values.
 * Remember the values are the constant in the quadratic function, so if you only return the
 * violation then if it is not violating the constraint this would be zero which means it
 * will always appear to be on the constraint boundary which will cause issue when solving.
 * If it is not voliating the constraint then return the max negative number.
 * If no contacts are found return the negative of the collision margin buffer. This is why
 * it is important to not set the collision margin buffer to zero.
 */
struct DiscreteCombineCollisionData
{
  CombineValuesPostFn combine_values;

  CombineJacobianPostFn combine_jacobian;

  DiscreteCombineCollisionData() = default;
  DiscreteCombineCollisionData(CombineCollisionDataMethod method);
};
}  // namespace trajopt_ifopt
#endif  // TRAJOPT_IFOPT_DISCRETE_COMBINE_COLLISION_DATA_H
