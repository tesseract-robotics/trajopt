#pragma once
#include <trajopt/common.hpp>
#include <trajopt_sco/optimizers.hpp>

namespace trajopt
{
class TrajOptProb;

/**
Returns a callback function suitable for an Optimizer.
This callback will plot the trajectory (with translucent copies of the robot) as
well as all of the Cost and Constraint functions with plot methods
*/
sco::Optimizer::Callback PlotCallback(TrajOptProb& prob, const tesseract_visualization::Visualization::Ptr& plotter);
/**
 * @brief Returns a callback suitable for an optimizer but does not require the problem
 * @param plotter
 * @param joint_names
 * @return
 */
sco::Optimizer::Callback PlotProbCallback(const tesseract_visualization::Visualization::Ptr& plotter,
                                          const tesseract_environment::StateSolver::Ptr& state_solver,
                                          const std::vector<std::string>& joint_names);

}  // namespace trajopt
