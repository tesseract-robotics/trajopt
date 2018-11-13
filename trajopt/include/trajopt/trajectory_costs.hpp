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
  /** @brief Forms error in QuadExpr - independent of penalty type */
  JointVelEqCost(const VarArray& traj, const VectorXd& coeffs, const VectorXd& targs, int& first_step, int& last_step);
  /** @brief Convexifies cost expression - In this case, it is already quadratic so there's nothing to do */
  virtual sco::ConvexObjectivePtr convex(const DblVec& x, sco::Model* model);
  /** @brief Numerically evaluate cost given the vector of values */
  virtual double value(const DblVec&);

private:
  /** @brief The variables being optimized. Used to properly index the vector being optimized */
  VarArray vars_;
  /** @brief The coefficients used to weight the cost */
  Eigen::VectorXd coeffs_;
  /** @brief Stores the cost as an expression */
  sco::QuadExpr expr_;
  /** @brief Vector of velocity targets */
  VectorXd targs_;
  /** @brief First time step to which the term is applied */
  int first_step_;
  /** @brief Last time step to which the term is applied */
  int last_step_;

  // TODO: Add time steps
  // TODO: Add getVars
};

/**
 * @brief The JointVelIneqCost class
 * Assumes that the target is ...
 */
class TRAJOPT_API JointVelIneqCost : public Cost
{
public:
  /** @brief Forms error in QuadExpr - independent of penalty type */
  JointVelIneqCost(const VarArray& traj,
                   const VectorXd& coeffs,
                   const VectorXd& targs,
                   const VectorXd& upper_limits,
                   const VectorXd& lower_limits,
                   int& first_step,
                   int& last_step);
  /** @brief Convexifies cost expression - In this case, it is already quadratic so there's nothing to do */
  virtual ConvexObjectivePtr convex(const vector<double>& x, Model* model);
  /** @brief Numerically evaluate cost given the vector of values */
  virtual double value(const vector<double>&);

private:
  /** @brief The variables being optimized. Used to properly index the vector being optimized */
  VarArray vars_;
  /** @brief The coefficients used to weight the cost */
  VectorXd coeffs_;
  /** @brief Vector of velocity targets */
  VectorXd upper_tols_;
  /** @brief Vector of velocity targets */
  VectorXd lower_tols_;
  /** @brief Vector of velocity targets */
  VectorXd targs_;
  /** @brief First time step to which the term is applied */
  int first_step_;
  /** @brief Last time step to which the term is applied */
  int last_step_;
  /** @brief Stores the cost as an expression */
  AffExpr expr_;
  /** @brief Stores the cost as an expression */
  AffExpr expr_neg_;

  // TODO: Add time steps
  // TODO: Add getVars
};

class TRAJOPT_API JointVelEqConstraint : public EqConstraint
{
public:
  /** @brief Forms error in QuadExpr - independent of penalty type */
  JointVelEqConstraint(const VarArray& traj,
                       const VectorXd& coeffs,
                       const VectorXd& targs,
                       int& first_step,
                       int& last_step);
  /** @brief Convexifies cost expression - In this case, it is already quadratic so there's nothing to do */
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  /** @brief Numerically evaluate cost given the vector of values */
  virtual vector<double> value(const vector<double>&);
  /** Calculate constraint violations (positive part for inequality constraint,
   * absolute value for inequality constraint)*/
  vector<double> violations(const vector<double>& x);
  /** Sum of violations */
  double violation(const vector<double>& x);

  //  VarVector getVars() { return VarVector(); }
  // TODO: Figure out why we are using vararray instead of varvector, and should I convert between them?
  // From looking at pos, it appears that array contains all of the vectors for all the joints (maybe timesteps too?)

private:
  /** @brief The variables being optimized. Used to properly index the vector being optimized */
  VarArray vars_;
  /** @brief The coefficients used to weight the cost */
  VectorXd coeffs_;
  /** @brief Stores the cost as an expression */
  AffExpr expr_;
  /** @brief Vector of velocity targets */
  VectorXd targs_;
  /** @brief First time step to which the term is applied */
  int first_step_;
  /** @brief Last time step to which the term is applied */
  int last_step_;

  // TODO: Add time steps
};

class TRAJOPT_API JointVelIneqConstraint : public IneqConstraint
{
public:
  /** @brief Forms error in QuadExpr - independent of penalty type */
  JointVelIneqConstraint(const VarArray& traj,
                         const VectorXd& coeffs,
                         const VectorXd& targs,
                         const VectorXd& upper_limits,
                         const VectorXd& lower_limits,
                         int& first_step,
                         int& last_step);
  /** @brief Convexifies cost expression - In this case, it is already quadratic so there's nothing to do */
  virtual ConvexConstraintsPtr convex(const vector<double>& x, Model* model);
  /** @brief Numerically evaluate cost given the vector of values */
  virtual vector<double> value(const vector<double>&);

private:
  /** @brief The variables being optimized. Used to properly index the vector being optimized */
  VarArray vars_;
  /** @brief The coefficients used to weight the cost */
  VectorXd coeffs_;
  /** @brief Vector of velocity targets */
  VectorXd upper_tols_;
  /** @brief Vector of velocity targets */
  VectorXd lower_tols_;
  /** @brief Vector of velocity targets */
  VectorXd targs_;
  /** @brief First time step to which the term is applied */
  int first_step_;
  /** @brief Last time step to which the term is applied */
  int last_step_;
  /** @brief Stores the cost as an expression */
  AffExpr expr_;
  /** @brief Stores the cost as an expression */
  AffExpr expr_neg_;

  // TODO: Add time steps
  // TODO: Add getVars
};

class TRAJOPT_API JointAccCost : public Cost
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
}  // namespace trajopt
