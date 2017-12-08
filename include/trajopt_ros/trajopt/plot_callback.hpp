#pragma once
#include "sco/optimizers.hpp"
#include "trajopt/common.hpp"
namespace trajopt {
class TrajOptProb;

/**
Returns a callback function suitable for an Optimizer.
This callback will plot the trajectory (with translucent copies of the robot) as well as all of the Cost and Constraint functions with plot methods
*/
Optimizer::Callback TRAJOPT_API PlotCallback(TrajOptProb& prob);

}
