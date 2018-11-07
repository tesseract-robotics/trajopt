#pragma once

/**
Simple quadratic costs on trajectory
*/

#include <trajopt/common.hpp>
#include <trajopt_sco/modeling.hpp>
#include <trajopt_utils/macros.h>

namespace trajopt
{
class TRAJOPT_API JointPosCost : public sco::Cost
{
public:
  JointPosCost(const VarVector& vars, const VectorXd& vals, const VectorXd& coeffs);
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  /** @brief Evaluate cost given the vector of values */
  virtual double value(const vector<double>&);

private:
  /** @brief The variables being optimized. Used to properly index the vector being optimized */
  VarVector vars_;
  /** @brief The target values. Cost is applied to difference between current value and this one */
  VectorXd vals_;
  /** @brief The coefficients used to weight the cost */
  VectorXd coeffs_;
  /** @brief Stores the cost as an expression */
  QuadExpr expr_;
};

class TRAJOPT_API JointVelCost : public sco::Cost
{
public:
  JointVelCost(const VarArray& traj, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec&);

private:
  VarArray vars_;
  Eigen::VectorXd coeffs_;
  sco::QuadExpr expr_;
};

class TRAJOPT_API JointAccCost : public sco::Cost
{
public:
  JointAccCost(const VarArray& traj, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec&);

private:
  VarArray vars_;
  Eigen::VectorXd coeffs_;
  sco::QuadExpr expr_;
};

class TRAJOPT_API JointJerkCost : public sco::Cost
{
public:
  JointJerkCost(const VarArray& traj, const Eigen::VectorXd& coeffs);
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  virtual double value(const DblVec&);

private:
  VarArray vars_;
  Eigen::VectorXd coeffs_;
  sco::QuadExpr expr_;
};
}
