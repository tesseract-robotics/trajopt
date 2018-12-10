#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <functional>
#include <set>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

namespace trajopt
{
void PlotCosts(const tesseract::BasicPlottingPtr plotter,
               const std::vector<std::string>& joint_names,
               const std::vector<sco::CostPtr>& costs,
               const std::vector<sco::ConstraintPtr>& cnts,
               const VarArray& vars,
               const sco::OptResults& results)
{
  plotter->clear();

  for (const sco::CostPtr& cost : costs)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cost.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  for (const sco::ConstraintPtr& cnt : cnts)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cnt.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  plotter->plotTrajectory(joint_names, getTraj(results.x, vars));
  plotter->waitForInput();
}

sco::Optimizer::Callback PlotCallback(TrajOptProb& prob, const tesseract::BasicPlottingPtr plotter)
{
  std::vector<sco::ConstraintPtr> cnts = prob.getConstraints();
  return std::bind(&PlotCosts,
                   plotter,
                   prob.GetKin()->getJointNames(),
                   std::ref(prob.getCosts()),
                   cnts,
                   std::ref(prob.GetVars()),
                   std::placeholders::_2);
}
}
