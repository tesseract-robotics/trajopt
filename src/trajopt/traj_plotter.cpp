#include <trajopt_ros/trajopt/traj_plotter.hpp>
#include <trajopt_ros/utils/eigen_conversions.hpp>
#include <boost/foreach.hpp>
#include <trajopt_ros/osgviewer/osgviewer.hpp>
#include <iostream>
using namespace OpenRAVE;
using namespace std;

namespace trajopt {

TrajPlotter::TrajPlotter(OR::EnvironmentBasePtr env, ConfigurationPtr config, const VarArray& trajvars)
  : m_env(env), m_config(config), m_trajvars(trajvars), m_decimation(1)
{
}

void TrajPlotter::Add(const vector<CostPtr>& costs) {
  BOOST_FOREACH(const CostPtr& cost, costs) {
    if (PlotterPtr plotter = boost::dynamic_pointer_cast<Plotter>(cost)) {
      m_plotters.push_back(plotter);
    }
  }
}
void TrajPlotter::Add(const vector<ConstraintPtr>& constraints) {
  BOOST_FOREACH(const ConstraintPtr& cnt, constraints) {
    if (PlotterPtr plotter = boost::dynamic_pointer_cast<Plotter>(cnt)) {
      m_plotters.push_back(plotter);
    }
  }
}
void TrajPlotter::Add(const vector<PlotterPtr>& plotters) {
  BOOST_FOREACH(const PlotterPtr& plotter, plotters) {
    m_plotters.push_back(plotter);
  }
}
void TrajPlotter::AddLink(KinBody::LinkPtr link) {
  m_links.insert(link);
}

void TrajPlotter::Add(PlotterPtr plotter) {
  m_plotters.push_back(plotter);
}

void TrajPlotter::OptimizerCallback(OptProb*, DblVec& x) {
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(m_env);
  vector<GraphHandlePtr> handles;

  MatrixXd traj = getTraj(x,m_trajvars);
  vector<KinBodyPtr> bodies = m_config->GetBodies();
  vector<Eigen::MatrixXf> linktrajs(m_links.size(), Eigen::MatrixXf(traj.rows(),3));
  for (int i=0; i < traj.rows(); ++i) {
    m_config->SetDOFValues(toDblVec(traj.row(i)));
    if (i % m_decimation == 0) {
      BOOST_FOREACH(const KinBodyPtr& body, bodies) {
        handles.push_back(viewer->PlotKinBody(body));
        SetTransparency(handles.back(), .35);
      }
    }
    int iLink=0;
    BOOST_FOREACH(const KinBody::LinkPtr& link, m_links) {
      OR::Vector p = link->GetTransform().trans;
      linktrajs[iLink].row(i) = Eigen::Vector3f(p.x, p.y, p.z);
      ++iLink;
    }
  }

  BOOST_FOREACH(PlotterPtr& plotter, m_plotters) {
    plotter->Plot(x, *m_env, handles);
  }

  BOOST_FOREACH(Eigen::MatrixXf& linktraj, linktrajs) {
    handles.push_back(m_env->drawlinestrip(linktraj.data(), linktraj.rows(), linktraj.cols()*sizeof(float), 2, RaveVectorf(1,0,0,1)));
  }


  viewer->Idle();


}

}
