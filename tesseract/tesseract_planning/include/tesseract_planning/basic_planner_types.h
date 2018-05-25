/**
 * @file basic_planner_types.h
 * @brief Basic low-level planner types.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
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
#ifndef TESSERACT_PLANNING_BASIC_PLANNER_TYPES_H
#define TESSERACT_PLANNING_BASIC_PLANNER_TYPES_H

#include <tesseract_ros/ros_basic_env.h>

namespace tesseract
{

namespace tesseract_planning
{

/** @brief Negative status code should be related to errors and positive should be used for status of covergenace */
typedef std::unordered_map<int, std::string> StatusCodeMap;

struct PlannerRequest
{
  std::string name;                       /**< @brief The name of the planner to use */
  tesseract_ros::ROSBasicEnvConstPtr env; /**< @brief The environment to use for planning */
  EnvStateConstPtr start_state;           /**< @brief The start state to use for planning */
  std::string config;                     /**< @brief The configuration to use (json file) */
  std::string config_format;              /**< @brief The file extension used to parse config */
};

struct PlannerResponse
{
  std::vector<std::string> joint_names; /**< @brief The joint names */
  TrajArray trajectory;                 /**< @brief The generated trajectory */
  int status_code;                      /**< @brief Negative status code should be related to errors and positive should be used for status of covergenace */
  std::string status_description;       /**< @brief Provide a brief description about what the error code means */
};

}
}

#endif // TESSERACT_PLANNING_BASIC_PLANNER_TYPES_H
