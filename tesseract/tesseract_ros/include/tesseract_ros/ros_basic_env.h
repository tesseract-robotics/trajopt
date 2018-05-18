/**
 * @file ros_basic_env.h
 * @brief Tesseract ROS Basic low-level environment with collision and distance functions.
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
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
#ifndef TESSERACT_ROS_BASIC_ENV_H
#define TESSERACT_ROS_BASIC_ENV_H

#include <tesseract_core/basic_env.h>
#include <urdf/model.h>
#include <srdfdom/model.h>

namespace tesseract
{

namespace tesseract_ros
{

class ROSBasicEnv : public BasicEnv
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual bool init(urdf::ModelInterfaceConstSharedPtr urdf_model) = 0;
  virtual bool init(urdf::ModelInterfaceConstSharedPtr urdf_model, srdf::ModelConstSharedPtr srdf_model) = 0;

  virtual bool checkInitialized() const = 0;

  virtual urdf::ModelInterfaceConstSharedPtr getURDF() const = 0;

  virtual srdf::ModelConstSharedPtr getSRDF() const = 0;

  virtual void loadContactCheckerPlugin(const std::string& plugin) = 0;

}; // class ROSBasicEnv
typedef std::shared_ptr<ROSBasicEnv> ROSBasicEnvPtr;
typedef std::shared_ptr<const ROSBasicEnv> ROSBasicEnvConstPtr;

} //namespace tesseract_ros
} //namespace tesseract

#endif // TESSERACT_ROS_BASIC_ENV_H
