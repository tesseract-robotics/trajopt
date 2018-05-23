/**
 * @file basic_planner.h
 * @brief Basic low-level planner class.
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
#ifndef TESSERACT_PLANNING_BASIC_PLANNER_H
#define TESSERACT_PLANNING_BASIC_PLANNER_H
#include <tesseract_planning/basic_planner_types.h>
#include <unordered_map>

namespace tesseract
{

namespace tesseract_planning
{

class BasicPlanner
{
public:
  /** @brief Construct a basic planner */
  BasicPlanner() {}

  virtual ~BasicPlanner();

  /** @brief Get the name of this planner */
  const std::string& getName() const { return name_; }

  /** @brief Get the planner request */
  const PlannerRequest& getRequest() const { return request_; }

  /** @brief Set the planner request for this context */
  void setRequest(const PlannerRequest& request) { request_ = request; }

  const StatusCodeMap& getAvailableStatusCodes() const { return status_code_map_; }

  /** @brief Solve the planner request problem */
  virtual bool solve(PlannerResponse& res) = 0;

  /**
   * @brief If solve() is running, terminate the computation. Return false if termination not possible. No-op if
   * solve() is not running (returns true).
   */
  virtual bool terminate() = 0;

  /** @brief Clear the data structures used by the planner */
  virtual void clear() = 0;

protected:

  std::string name_;              /**< @brief The name of this planner */
  PlannerRequest request_;        /**< @brief The planner request information */
  StatusCodeMap status_code_map_; /**< @brief A map of error codes to description */


};
}
}
#endif // TESSERACT_ROS_BASIC_PLANNER_H
