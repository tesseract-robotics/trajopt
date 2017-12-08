#include "trajopt/plot_callback.hpp"
#include "trajopt/common.hpp"
#include "osgviewer/osgviewer.hpp"
#include "utils/eigen_conversions.hpp"
#include <boost/foreach.hpp>
#include "trajopt/problem_description.hpp"
#include <set>
using namespace OpenRAVE;
using namespace util;
using namespace std;
namespace trajopt {



void PlotTraj(OSGViewer& viewer, Configuration& rad, const TrajArray& x, vector<GraphHandlePtr>& handles) {
  vector<KinBodyPtr> bodies = rad.GetBodies();
  for (int i=0; i < x.rows(); ++i) {
    rad.SetDOFValues(toDblVec(x.row(i)));
    BOOST_FOREACH(const KinBodyPtr& body, bodies) {
      handles.push_back(viewer.PlotKinBody(body));
      SetTransparency(handles.back(), .35);
    }
  }
}

void PlotCosts(OSGViewer& viewer, vector<CostPtr>& costs, vector<ConstraintPtr>& cnts, Configuration& rad, const VarArray& vars, const DblVec& x) {
  vector<GraphHandlePtr> handles;
  handles.clear();
  BOOST_FOREACH(CostPtr& cost, costs) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cost.get())) {
      plotter->Plot(x, *rad.GetEnv(), handles);
    }
  }
  BOOST_FOREACH(ConstraintPtr& cnt, cnts) {
    if (Plotter* plotter = dynamic_cast<Plotter*>(cnt.get())) {
      plotter->Plot(x, *rad.GetEnv(), handles);
    }
  }
  TrajArray traj = getTraj(x, vars);
  PlotTraj(viewer, rad, traj, handles);
  viewer.Idle();
  rad.SetDOFValues(toDblVec(traj.row(traj.rows()-1)));
}



Optimizer::Callback PlotCallback(TrajOptProb& prob) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(prob.GetEnv());
  vector<ConstraintPtr> cnts = prob.getConstraints();
  return boost::bind(&PlotCosts, boost::ref(*viewer),
                      boost::ref(prob.getCosts()),
                      cnts,
                      boost::ref(*prob.GetRAD()),
                      boost::ref(prob.GetVars()),
                      _2);
}

}
