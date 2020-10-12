#pragma once
#include <trajopt_sco/modeling.hpp>
#include <trajopt_sco/num_diff.hpp>
#include <trajopt_sco/sco_common.hpp>

/**
@file modeling_utils.hpp
@brief Build problem from user-defined functions
Utilities for creating Cost and Constraint objects from functions
using numerical derivatives or user-defined analytic derivatives.
 */

namespace sco
{
enum PenaltyType
{
  SQUARED,
  ABS,
  HINGE
};

/**
x is the big solution vector of the whole problem. vars are variables that
index into the vector x
this function extracts (from x) the values of the variables in vars
 */
Eigen::VectorXd getVec(const DblVec& x, const VarVector& vars);
/**
Same idea as above, but different output type
 */
DblVec getDblVec(const DblVec& x, const VarVector& vars);

AffExpr affFromValGrad(double y, const Eigen::VectorXd& x, const Eigen::VectorXd& dydx, const VarVector& vars);

class CostFromFunc : public Cost
{
public:
  using Ptr = std::shared_ptr<CostFromFunc>;

  /// supply function, obtain derivative and hessian numerically
  CostFromFunc(ScalarOfVector::Ptr f, VarVector vars, const std::string& name, bool full_hessian = false);
  double value(const DblVec& x) override;
  ConvexObjective::Ptr convex(const DblVec& x, Model* model) override;
  VarVector getVars() override { return vars_; }

protected:
  ScalarOfVector::Ptr f_;
  VarVector vars_;
  bool full_hessian_;
  double epsilon_;
};

class CostFromErrFunc : public Cost
{
public:
  /// supply error function, obtain derivative numerically
  CostFromErrFunc(VectorOfVector::Ptr f,
                  VarVector vars,
                  const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                  PenaltyType pen_type,
                  const std::string& name);
  /// supply error function and gradient
  CostFromErrFunc(VectorOfVector::Ptr f,
                  MatrixOfVector::Ptr dfdx,
                  VarVector vars,
                  const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                  PenaltyType pen_type,
                  const std::string& name);
  double value(const DblVec& x) override;
  ConvexObjective::Ptr convex(const DblVec& x, Model* model) override;
  VarVector getVars() override { return vars_; }

protected:
  VectorOfVector::Ptr f_;
  MatrixOfVector::Ptr dfdx_;
  VarVector vars_;
  Eigen::VectorXd coeffs_;
  PenaltyType pen_type_;
  double epsilon_;
};

class ConstraintFromErrFunc : public Constraint
{
public:
  using Ptr = std::shared_ptr<ConstraintFromErrFunc>;

  /// supply error function, obtain derivative numerically
  ConstraintFromErrFunc(VectorOfVector::Ptr f,
                        VarVector vars,
                        const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                        ConstraintType type,
                        const std::string& name);
  /// supply error function and gradient
  ConstraintFromErrFunc(VectorOfVector::Ptr f,
                        MatrixOfVector::Ptr dfdx,
                        VarVector vars,
                        const Eigen::Ref<const Eigen::VectorXd>& coeffs,
                        ConstraintType type,
                        const std::string& name);
  DblVec value(const DblVec& x) override;
  ConvexConstraints::Ptr convex(const DblVec& x, Model* model) override;
  ConstraintType type() override { return type_; }
  VarVector getVars() override { return vars_; }

protected:
  VectorOfVector::Ptr f_;
  MatrixOfVector::Ptr dfdx_;
  VarVector vars_;
  Eigen::VectorXd coeffs_;
  ConstraintType type_;
  double epsilon_;
  Eigen::VectorXd scaling_;
};

std::string AffExprToString(const AffExpr& aff);
}  // namespace sco
