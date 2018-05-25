#pragma once
#include <Eigen/Core>
#include <map>
#include <vector>

#include <trajopt_sco/modeling.hpp>
#include <trajopt_utils/basic_array.hpp>
#include <trajopt_utils/macros.h>

#include <tesseract_core/basic_plotting.h>

namespace trajopt
{
using std::vector;
using std::map;
using namespace sco;
using namespace util;

typedef BasicArray<Var> VarArray;
typedef BasicArray<AffExpr> AffArray;
typedef BasicArray<Cnt> CntArray;

typedef vector<double> DblVec;
typedef vector<int> IntVec;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Matrix3d;

/** @brief Interface for objects that know how to plot themselves given solution
 * vector x */
class Plotter
{
public:
  virtual void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) = 0;
  virtual ~Plotter() {}
};
typedef std::shared_ptr<Plotter> PlotterPtr;
}
