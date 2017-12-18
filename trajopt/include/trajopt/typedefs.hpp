#pragma once
#include <vector>
#include <map>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>

#include <trajopt_sco/modeling.hpp>
#include <trajopt_utils/basic_array.hpp>
#include <trajopt_utils/macros.h>

#include <moveit/planning_scene/planning_scene.h>
#include <trajopt/basic_kin.h>

namespace trajopt {

using std::vector;
using std::map;
using namespace sco;
using namespace util;
using planning_scene::PlanningScenePtr;
using planning_scene::PlanningSceneConstPtr;

typedef BasicArray<Var> VarArray;
typedef BasicArray<AffExpr> AffArray;
typedef BasicArray<Cnt> CntArray;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;

typedef vector<double> DblVec;
typedef vector<int> IntVec;

using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

/**
Interface for objects that know how to plot themselves given solution vector x
*/
class Plotter {
public:
  virtual void Plot(const DblVec& x) = 0;
  virtual ~Plotter() {}
};
typedef boost::shared_ptr<Plotter> PlotterPtr;

}
