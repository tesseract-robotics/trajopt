#include <trajopt/common.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <tesseract_core/basic_kin.h>
#include <tesseract_core/basic_env.h>
#include <trajopt/plot_callback.hpp>
#include <boost/foreach.hpp>
#include <boost/bind.hpp>
#include <set>

using namespace util;
using namespace std;
namespace trajopt
{

void PlotCosts(const tesseract::BasicPlottingPtr plotter, const std::vector<std::string>& joint_names, vector<CostPtr>& costs, vector<ConstraintPtr>& cnts, const VarArray& vars, const DblVec& x)
{
  plotter->clear();

  BOOST_FOREACH(CostPtr& cost, costs)
  {
    if (Plotter* plt = dynamic_cast<Plotter*>(cost.get()))
    {
      plt->Plot(plotter, x);
    }
  }
  BOOST_FOREACH(ConstraintPtr& cnt, cnts)
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
  return boost::bind(&PlotCosts,
                      plotter,
                      prob.GetKin()->getJointNames(),
                      boost::ref(prob.getCosts()),
                      cnts,
                      boost::ref(prob.GetVars()),
                      _2);
}

}
