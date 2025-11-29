/**
 * @file eigen_types.h
 * @brief Contains eigen types for the trust region sqp solver
 *
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
#ifndef TRAJOPT_SQP_EIGEN_TYPES_H
#define TRAJOPT_SQP_EIGEN_TYPES_H

#include <Eigen/Eigen>

namespace trajopt_sqp
{
using SparseMatrix = Eigen::SparseMatrix<double, Eigen::RowMajor>;
using SparseVector = Eigen::SparseVector<double, Eigen::RowMajor>;
}  // namespace trajopt_sqp

#endif  // TRAJOPT_SQP_EIGEN_TYPES_H
