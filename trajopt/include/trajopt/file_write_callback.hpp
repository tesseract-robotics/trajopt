#pragma once
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>

namespace trajopt
{
sco::Optimizer::Callback WriteCallback(std::shared_ptr<std::ofstream> file, const TrajOptProbPtr& prob);
}
