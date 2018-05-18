/**
 * @file trajopt_planner.h
 * @brief Tesseract ROS Trajopt planner
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
#ifndef TESSERACT_ROS_TRAJOPT_PLANNER_H
#define TESSERACT_ROS_TRAJOPT_PLANNER_H

#include <tesseract_ros_planning/basic_planner.h>

namespace tesseract
{

namespace tesseract_ros
{

class TrajoptPlanner : public BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  TrajoptPlanner()
  {
    name_ = "TRAJOPT";

    // Error Status Codes
    status_code_map_[-1] = "Invalid config data format";
    status_code_map_[-2] = "Failed to parse config data";
    status_code_map_[-3] = "";

    // Converge Status Codes

  }

  bool solve(PlannerResponse& res) override;

  bool terminate() override;

  void clear() override;

private:

};
}
}
#endif // TESSERACT_ROS_TRAJOPT_PLANNER_H
