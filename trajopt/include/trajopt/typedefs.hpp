#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Core>
#include <map>
#include <vector>

#include <tesseract_visualization/visualization.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/basic_array.hpp>

namespace trajopt
{
using VarArray = util::BasicArray<sco::Var>;
using AffArray = util::BasicArray<sco::AffExpr>;
using CntArray = util::BasicArray<sco::Cnt>;
using TrajArray = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using DblVec = sco::DblVec;
using IntVec = sco::IntVec;

template <typename T>
using AlignedVector = std::vector<T, Eigen::aligned_allocator<T>>;

template <typename Key, typename Value>
using AlignedUnorderedMap = std::unordered_map<Key,
                                               Value,
                                               std::hash<Key>,
                                               std::equal_to<Key>,
                                               Eigen::aligned_allocator<std::pair<const Key, Value>>>;

/** @brief Interface for objects that know how to plot themselves given solution
 * vector x */
class Plotter
{
public:
  using Ptr = std::shared_ptr<Plotter>;

  Plotter() = default;
  virtual ~Plotter() = default;
  Plotter(const Plotter&) = default;
  Plotter& operator=(const Plotter&) = default;
  Plotter(Plotter&&) = default;
  Plotter& operator=(Plotter&&) = default;

  virtual void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) = 0;
};

/**  @brief Adds plotting to the VectorOfVector class in trajopt_sco */
class TrajOptVectorOfVector : public sco::VectorOfVector
{
public:
  virtual void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const Eigen::VectorXd& dof_vals) = 0;
};

/**  @brief Adds plotting to the CostFromErrFunc class in trajopt_sco */
class TrajOptCostFromErrFunc : public sco::CostFromErrFunc, public Plotter
{
public:
  /// supply error function, obtain derivative numerically
  TrajOptCostFromErrFunc(sco::VectorOfVector::Ptr f,
                         sco::VarVector vars,
                         const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                         sco::PenaltyType pen_type,
                         const std::string& name)
    : CostFromErrFunc(std::move(f), std::move(vars), coeffs, pen_type, name)
  {
  }

  /// supply error function and gradient
  TrajOptCostFromErrFunc(sco::VectorOfVector::Ptr f,
                         sco::MatrixOfVector::Ptr dfdx,
                         sco::VarVector vars,
                         const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                         sco::PenaltyType pen_type,
                         const std::string& name)
    : CostFromErrFunc(std::move(f), std::move(dfdx), std::move(vars), coeffs, pen_type, name)
  {
  }

  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override
  {
    // If error function has a inherited from TrajOptVectorOfVector, call its Plot function
    if (auto* plt = dynamic_cast<TrajOptVectorOfVector*>(f_.get()))
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
  TrajOptConstraintFromErrFunc(sco::VectorOfVector::Ptr f,
                               sco::VarVector vars,
                               const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                               sco::ConstraintType type,
                               const std::string& name)
    : ConstraintFromErrFunc(std::move(f), std::move(vars), coeffs, type, name)
  {
  }

  /// supply error function and gradient
  TrajOptConstraintFromErrFunc(sco::VectorOfVector::Ptr f,
                               sco::MatrixOfVector::Ptr dfdx,
                               sco::VarVector vars,
                               const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                               sco::ConstraintType type,
                               const std::string& name)
    : ConstraintFromErrFunc(std::move(f), std::move(dfdx), std::move(vars), coeffs, type, name)
  {
  }

  void Plot(const tesseract_visualization::Visualization::Ptr& plotter, const DblVec& x) override
  {
    // If error function has a inherited from TrajOptVectorOfVector, call its Plot function
    if (auto* plt = dynamic_cast<TrajOptVectorOfVector*>(f_.get()))
    {
      Eigen::VectorXd dof_vals = sco::getVec(x, vars_);
      plt->Plot(plotter, dof_vals);
    }
  }
};
}  // namespace trajopt
