#include <functional>
#include <set>
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

using namespace util;
using namespace std;
namespace trajopt
{
void PlotCosts(const tesseract::BasicPlottingPtr plotter,
               const std::vector<std::string>& joint_names,
               vector<CostPtr>& costs,
               vector<ConstraintPtr>& cnts,
               const VarArray& vars,
               const DblVec& x)
{
  plotter->clear();

  for (CostPtr& cost : costs)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cost.get()))
    {
      plt->Plot(plotter, x);
    }
  }

  for (ConstraintPtr& cnt : cnts)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cnt.get()))
    {
      plt->Plot(plotter, x);
    }
  }

  plotter->plotTrajectory(joint_names, getTraj(x, vars));
  plotter->waitForInput();
}

Optimizer::Callback PlotCallback(TrajOptProb& prob, const tesseract::BasicPlottingPtr plotter)
{
  vector<ConstraintPtr> cnts = prob.getConstraints();
  return std::bind(&PlotCosts,
                   plotter,
                   prob.GetKin()->getJointNames(),
                   std::ref(prob.getCosts()),
                   cnts,
                   std::ref(prob.GetVars()),
                   std::placeholders::_2);
}
}
