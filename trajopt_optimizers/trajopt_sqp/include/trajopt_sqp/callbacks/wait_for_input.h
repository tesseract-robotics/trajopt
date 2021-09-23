/**
 * @file wait_for_input.h
 * @brief A callback that calls the plotter waitForInput function to pause the optimization for debugging
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
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
#ifndef TRAJOPT_SQP_INCLUDE_WAIT_FOR_INPUT_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_WAIT_FOR_INPUT_CALLBACK_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <tesseract_visualization/visualization.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
/**
 * @brief Calls the waitForInput function in the plotter. Use to pause the optimization while displaying intermediate
 * results
 */
class WaitForInputCallback : public trajopt_sqp::SQPCallback
{
public:
  using Ptr = std::shared_ptr<WaitForInputCallback>;
  using ConstPtr = std::shared_ptr<const WaitForInputCallback>;

  WaitForInputCallback(tesseract_visualization::Visualization::Ptr plotter);

  bool execute(const QPProblem& problem, const trajopt_sqp::SQPResults& sqp_results) override;

protected:
  tesseract_visualization::Visualization::Ptr plotter_;
};
}  // namespace trajopt_sqp

#endif
