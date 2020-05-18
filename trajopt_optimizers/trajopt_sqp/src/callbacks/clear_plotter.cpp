/**
 * @file clear_plotter.h
 * @brief A callback to clear the plotter passed in. Add this callback before plotting callbacks to clear the plotter
 * before plotting new results
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
#include <trajopt_sqp/callbacks/clear_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

ClearPlotterCallback::ClearPlotterCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

bool ClearPlotterCallback::execute(const ifopt::Problem& /*nlp*/, const trajopt_sqp::SQPResults&)
{
  plotter_->clear();
  return true;
}
