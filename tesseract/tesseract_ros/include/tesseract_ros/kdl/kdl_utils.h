/**
 * @file kdl_utils.h
 * @brief Tesseract ROS KDL utility functions.
 *
 * @author Levi Armstrong
 * @date May 27, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#ifndef TESSERACT_ROS_KDL_UTILS_H
#define TESSERACT_ROS_KDL_UTILS_H
#include <kdl/frames.hpp>
#include <Eigen/Eigen>

namespace tesseract
{
namespace tesseract_ros
{
/**
 * @brief Convert KDL::Frame to Eigen::Affine3d
 * @param frame Input KDL Frame
 * @param transform Output Eigen transform (Affine3d)
 */
inline void KDLToEigen(const KDL::Frame& frame, Eigen::Affine3d& transform)
{
  transform.setIdentity();

  // translation
  for (size_t i = 0; i < 3; ++i)
    transform(i, 3) = frame.p[i];

  // rotation matrix
  for (size_t i = 0; i < 9; ++i)
    transform(i / 3, i % 3) = frame.M.data[i];
}

/**
 * @brief Convert Eigen::Affine3d to KDL::Frame
 * @param transform Input Eigen transform (Affine3d)
 * @param frame Output KDL Frame
 */
inline void EigenToKDL(const Eigen::Affine3d& transform, KDL::Frame& frame)
{
  frame.Identity();

  for (unsigned int i = 0; i < 3; ++i)
    frame.p[i] = transform(i, 3);

  for (unsigned int i = 0; i < 9; ++i)
    frame.M.data[i] = transform(i / 3, i % 3);
}

/**
 * @brief Convert KDL::Jacobian to Eigen::Matrix
 * @param jacobian Input KDL Jacobian
 * @param matrix Output Eigen MatrixXd
 */
inline void KDLToEigen(const KDL::Jacobian& jacobian, Eigen::Ref<Eigen::MatrixXd> matrix)
{
  for (size_t i = 0; i < jacobian.rows(); ++i)
    for (size_t j = 0; j < jacobian.columns(); ++j)
      matrix(i, j) = jacobian(i, j);
}

/**
 * @brief Convert Eigen::Vector to KDL::JntArray
 * @param vec Input Eigen vector
 * @param joints Output KDL joint array
 */
inline void EigenToKDL(const Eigen::Ref<const Eigen::VectorXd>& vec, KDL::JntArray& joints) { joints.data = vec; }
}
}
#endif  // TESSERACT_ROS_KDL_UTILS_H
