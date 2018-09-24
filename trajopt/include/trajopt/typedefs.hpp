#pragma once
#include <Eigen/Core>
#include <map>
#include <vector>

#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/basic_array.hpp>
#include <trajopt_utils/macros.h>

#include <tesseract_core/basic_plotting.h>

namespace trajopt
{
using std::map;
using std::vector;
using namespace sco;
using namespace util;

typedef BasicArray<Var> VarArray;
typedef BasicArray<AffExpr> AffArray;
typedef BasicArray<Cnt> CntArray;

typedef vector<double> DblVec;
typedef vector<int> IntVec;

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;

/** @brief Interface for objects that know how to plot themselves given solution
 * vector x */
class Plotter
{
public:
  virtual void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x) = 0;
  virtual ~Plotter() {}
};
typedef std::shared_ptr<Plotter> PlotterPtr;

/**  @brief Adds plotting to the CostFromErrFunc class in trajopt_sco
 *
 * */
template <typename Err_Func_Type>
class TrajoptCostFromErrFunc : public CostFromErrFunc, public Plotter
{
public:
  Err_Func_Type err_func_;
  /// supply error function, obtain derivative numerically
  TrajoptCostFromErrFunc(VectorOfVectorPtr f,
                         const VarVector& vars,
                         const VectorXd& coeffs,
                         PenaltyType pen_type,
                         const string& name)
  {
    sco::CostFromErrFunc(f, vars, coeffs, pen_type, name);
  }

  /// supply error function and gradient
  TrajoptCostFromErrFunc(VectorOfVectorPtr f,
                         MatrixOfVectorPtr dfdx,
                         const VarVector& vars,
                         const VectorXd& coeffs,
                         PenaltyType pen_type,
                         const string& name)
  {
    sco::CostFromErrFunc(f, dfdx, vars, coeffs, pen_type, name);
  }

  void Plot(const tesseract::BasicPlottingPtr plotter, const DblVec& x)
  {
//    auto err_func_ptr_ = &err_func_;
    // If error function has a inherited from Plotter, call its Plot function
//    if (Plotter* plt = dynamic_cast<Plotter*>(&err_func_))
//    {
      err_func_.Plot(plotter, x);
//    }
  }
};


}  // namespace trajopt
