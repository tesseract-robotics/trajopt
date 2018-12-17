#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <map>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/basic_array.hpp>
#include <trajopt_utils/macros.h>

#include <tesseract_core/basic_plotting.h>

namespace trajopt
{
typedef util::BasicArray<sco::Var> VarArray;
typedef util::BasicArray<sco::AffExpr> AffArray;
typedef util::BasicArray<sco::Cnt> CntArray;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> DblMatrix;
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> TrajArray;
typedef sco::DblVec DblVec;
typedef sco::IntVec IntVec;

/** @brief Interface for objects that know how to plot themselves given solution
 * vector x */
class Plotter
{
public:
  virtual void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) = 0;
  virtual ~Plotter() = default;
};
typedef std::shared_ptr<Plotter> PlotterPtr;

/**  @brief Adds plotting to the VectorOfVector class in trajopt_sco */
class TrajOptVectorOfVector : public sco::VectorOfVector
{
public:
  virtual void Plot(const tesseract::BasicPlottingPtr& plotter, const Eigen::VectorXd& dof_vals) = 0;
};

/**  @brief Adds plotting to the CostFromErrFunc class in trajopt_sco */
class TrajOptCostFromErrFunc : public sco::CostFromErrFunc, public Plotter
{
public:
  /// supply error function, obtain derivative numerically
  TrajOptCostFromErrFunc(sco::VectorOfVectorPtr f,
                         const sco::VarVector& vars,
                         const Eigen::VectorXd& coeffs,
                         sco::PenaltyType pen_type,
                         const std::string& name)
    : CostFromErrFunc(f, vars, coeffs, pen_type, name)
  {
  }

  /// supply error function and gradient
  TrajOptCostFromErrFunc(sco::VectorOfVectorPtr f,
                         sco::MatrixOfVectorPtr dfdx,
                         const sco::VarVector& vars,
                         const Eigen::VectorXd& coeffs,
                         sco::PenaltyType pen_type,
                         const std::string& name)
    : CostFromErrFunc(f, dfdx, vars, coeffs, pen_type, name)
  {
  }

  void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) override
  {
    // If error function has a inherited from TrajOptVectorOfVector, call its Plot function
    if (TrajOptVectorOfVector* plt = dynamic_cast<TrajOptVectorOfVector*>(f_.get()))
    {
      Eigen::VectorXd dof_vals = sco::getVec(x, vars_);
      plt->Plot(plotter, dof_vals);
    }
  }
};

/**  @brief Adds plotting to the ConstraintFromFunc class in trajopt_sco */
class TrajOptConstraintFromErrFunc : public sco::ConstraintFromErrFunc, public Plotter
{
public:
  /// supply error function, obtain derivative numerically
  TrajOptConstraintFromErrFunc(sco::VectorOfVectorPtr f,
                               const sco::VarVector& vars,
                               const Eigen::VectorXd& coeffs,
                               sco::ConstraintType type,
                               const std::string& name)
    : ConstraintFromErrFunc(f, vars, coeffs, type, name)
  {
  }

  /// supply error function and gradient
  TrajOptConstraintFromErrFunc(sco::VectorOfVectorPtr f,
                               sco::MatrixOfVectorPtr dfdx,
                               const sco::VarVector& vars,
                               const Eigen::VectorXd& coeffs,
                               sco::ConstraintType type,
                               const std::string& name)
    : ConstraintFromErrFunc(f, dfdx, vars, coeffs, type, name)
  {
  }

  void Plot(const tesseract::BasicPlottingPtr& plotter, const DblVec& x) override
  {
    // If error function has a inherited from TrajOptVectorOfVector, call its Plot function
    if (TrajOptVectorOfVector* plt = dynamic_cast<TrajOptVectorOfVector*>(f_.get()))
    {
      Eigen::VectorXd dof_vals = sco::getVec(x, vars_);
      plt->Plot(plotter, dof_vals);
    }
  }
};
}
