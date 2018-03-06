#include <trajopt/common.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt/basic_kin.h>
#include <trajopt/basic_env.h>
#include <trajopt/plot_callback.hpp>
#include <boost/foreach.hpp>
#include <set>

using namespace util;
using namespace std;
namespace trajopt {

void PlotCosts(BasicKinPtr manip, BasicEnvPtr env, vector<CostPtr>& costs, vector<ConstraintPtr>& cnts, const VarArray& vars, const DblVec& x)
{
  env->plotClear();

  BOOST_FOREACH(CostPtr& cost, costs) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cost.get())) {
      plotter->Plot(x);
    }
  }
  BOOST_FOREACH(ConstraintPtr& cnt, cnts) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cnt.get())) {
      plotter->Plot(x);
    }
  }
  TrajArray traj = getTraj(x, vars);
  std::vector<std::string> joint_names;
  manip->getJointNames(joint_names);

  env->plotTrajectory(manip->getName(), joint_names, traj);
  env->plotWaitForInput();
}

Optimizer::Callback PlotCallback(TrajOptProb& prob) {

  vector<ConstraintPtr> cnts = prob.getConstraints();
  return boost::bind(&PlotCosts,
                      prob.GetKin(),
                      prob.GetEnv(),
                      boost::ref(prob.getCosts()),
                      cnts,
                      boost::ref(prob.GetVars()),
                      _2);
}

}
