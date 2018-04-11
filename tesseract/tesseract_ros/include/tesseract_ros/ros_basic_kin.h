/**
 * @file ros_basic_kin.h
 * @brief Basic low-level kinematics functions.
 *
 * @author Levi Armstrong
 * @date April 15, 2018
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
#ifndef ROS_BASIC_KIN_H
#define ROS_BASIC_KIN_H

#include <tesseract_core/basic_kin.h>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <boost/shared_ptr.hpp>

namespace tesseract
{
namespace tesseract_ros
{

/**
 * @brief Basic low-level kinematics functions.
 *
 * All data should be returned or provided relative to the base link of the kinematic chain.
 *
 */
class ROSBasicKin : public BasicKin
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ROSBasicKin() {}


}; // class BasicKin

typedef boost::shared_ptr<ROSBasicKin> ROSBasicKinPtr;
typedef boost::shared_ptr<const ROSBasicKin> ROSBasicKinConstPtr;
} // namespace tesseract_ros
} // namespace trajopt

#endif // BASIC_KIN_H

