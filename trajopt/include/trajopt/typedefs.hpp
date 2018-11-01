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
  virtual void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) = 0;
  virtual ~Plotter() {}
};
typedef std::shared_ptr<Plotter> PlotterPtr;

/**  @brief Adds plotting to the VectorOfVector class in trajopt_sco */
class TrajOptVectorOfVector : public VectorOfVector
{
public:
  virtual void Plot(const tesseract::BasicPlottingPtr& plotter, const VectorXd& dof_vals) = 0;
};

/**  @brief Adds plotting to the CostFromErrFunc class in trajopt_sco */
class TrajOptCostFromErrFunc : public CostFromErrFunc, public Plotter
{
public:
  /// supply error function, obtain derivative numerically
  TrajOptCostFromErrFunc(VectorOfVectorPtr f,
                         const VarVector& vars,
                         const VectorXd& coeffs,
                         PenaltyType pen_type,
                         const std::string& name)
    : CostFromErrFunc(f, vars, coeffs, pen_type, name)
  {
  }

  /// supply error function and gradient
  TrajOptCostFromErrFunc(VectorOfVectorPtr f,
                         MatrixOfVectorPtr dfdx,
                         const VarVector& vars,
                         const VectorXd& coeffs,
                         PenaltyType pen_type,
                         const string& name)
    : CostFromErrFunc(f, dfdx, vars, coeffs, pen_type, name)
  {
  }

  void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) override
  {
    // If error function has a inherited from TrajOptVectorOfVector, call its Plot function
    if (TrajOptVectorOfVector* plt = dynamic_cast<TrajOptVectorOfVector*>(f_.get()))
    {
      VectorXd dof_vals = getVec(x, vars_);
      plt->Plot(plotter, dof_vals);
    }
  }
};

/**  @brief Adds plotting to the ConstraintFromFunc class in trajopt_sco */
class TrajOptConstraintFromErrFunc : public ConstraintFromErrFunc, public Plotter
{
public:
  /// supply error function, obtain derivative numerically
  TrajOptConstraintFromErrFunc(VectorOfVectorPtr f,
                            const VarVector& vars,
                            const VectorXd& coeffs,
                            ConstraintType type,
                            const std::string& name)
    : ConstraintFromErrFunc(f, vars, coeffs, type, name)
  {
  }

  /// supply error function and gradient
  TrajOptConstraintFromErrFunc(VectorOfVectorPtr f,
                            MatrixOfVectorPtr dfdx,
                            const VarVector& vars,
                            const VectorXd& coeffs,
                            ConstraintType type,
                            const std::string& name)
    : ConstraintFromErrFunc(f, dfdx, vars, coeffs, type, name)
  {
  }

  void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) override
  {
    // If error function has a inherited from TrajOptVectorOfVector, call its Plot function
    if (TrajOptVectorOfVector* plt = dynamic_cast<TrajOptVectorOfVector*>(f_.get()))
    {
      VectorXd dof_vals = getVec(x, vars_);
      plt->Plot(plotter, dof_vals);
    }
  }
};

}
