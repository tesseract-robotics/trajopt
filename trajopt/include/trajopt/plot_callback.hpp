#pragma once

#include <functional>
#include <memory>

#include <tesseract_visualization/fwd.h>
#include <tesseract_state_solver/fwd.h>

namespace sco
{
class OptProb;
struct OptResults;
}  // namespace sco

namespace trajopt
{
class TrajOptProb;

/**
Returns a callback function suitable for an Optimizer.
This callback will plot the trajectory (with translucent copies of the robot) as
well as all of the Cost and Constraint functions with plot methods
*/
std::function<void(sco::OptProb*, sco::OptResults&)>
PlotCallback(const std::shared_ptr<tesseract_visualization::Visualization>& plotter);

/**
 * @brief Returns a callback suitable for an optimizer but does not require the problem
 * @param plotter
 * @param state_solver
 * @param joint_name
 * @return
 */
std::function<void(sco::OptProb*, sco::OptResults&)>
PlotProbCallback(const std::shared_ptr<tesseract_visualization::Visualization>& plotter,
                 const tesseract_scene_graph::StateSolver& state_solver,
                 const std::vector<std::string>& joint_names);

}  // namespace trajopt
