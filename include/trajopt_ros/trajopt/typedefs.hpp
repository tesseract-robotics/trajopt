#pragma once
#include <vector>
#include <map>
#include <Eigen/Core>
#include <boost/shared_ptr.hpp>
#include <openrave/openrave.h>

#include <trajopt_ros/sco/modeling.hpp>
#include <trajopt_ros/utils/basic_array.hpp>
#include <trajopt_ros/utils/macros.h>

namespace trajopt {


namespace OR = OpenRAVE;
using OR::KinBody;
using OR::RobotBase;
using std::vector;
using std::map;
using namespace sco;
using namespace util;

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
  virtual void Plot(const DblVec& x, OR::EnvironmentBase&, std::vector<OR::GraphHandlePtr>& handles) = 0;
  virtual ~Plotter() {}
};
typedef boost::shared_ptr<Plotter> PlotterPtr;

}
