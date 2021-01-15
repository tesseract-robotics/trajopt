#pragma once

/*
 * Model a non-convex optimization problem by defining Cost and Constraint
 * objects
 * which know how to generate a convex approximation
 *
 *
 */
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <vector>
#include <memory>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_interface.hpp>

namespace sco
{
/**
Stores convex terms in a objective
For non-quadratic terms like hinge(x) and abs(x), it needs to add auxilliary
variables and linear constraints to the model
Note: When this object is deleted, the constraints and variables it added to the
model are removed
 */
class ConvexObjective
{
public:
  using Ptr = std::shared_ptr<ConvexObjective>;

  ConvexObjective(Model* model) : model_(model) {}
  virtual ~ConvexObjective();
  ConvexObjective(const ConvexObjective&) = delete;
  ConvexObjective& operator=(const ConvexObjective&) = delete;
  ConvexObjective(ConvexObjective&&) = default;
  ConvexObjective& operator=(ConvexObjective&&) = default;

  void addAffExpr(const AffExpr&);
  void addQuadExpr(const QuadExpr&);
  void addHinge(const AffExpr&, double coeff);
  void addAbs(const AffExpr&, double coeff);
  void addHinges(const AffExprVector&);
  void addL1Norm(const AffExprVector&);
  void addL2Norm(const AffExprVector&);
  void addMax(const AffExprVector&);

  bool inModel() { return model_ != nullptr; }
  void addConstraintsToModel();
  void removeFromModel();
  double value(const DblVec& x);

  Model* model_;
  /// The objective Function
  QuadExpr quad_;
  VarVector vars_;
  /// EQ Constraints
  AffExprVector eqs_;
  // INEQ Constraints
  AffExprVector ineqs_;
  CntVector cnts_;
};

/**
Stores convex inequality constraints and affine equality constraints.
Actually only affine inequality constraints are currently implemented.
*/
class ConvexConstraints
{
public:
  using Ptr = std::shared_ptr<ConvexConstraints>;

  ConvexConstraints(Model* model) : model_(model) {}
  virtual ~ConvexConstraints();

  /** Expression that should == 0 */
  void addEqCnt(const AffExpr&);
  /** Expression that should <= 0 */
  void addIneqCnt(const AffExpr&);
  void setModel(Model* model)
  {
    assert(!inModel());
    model_ = model;
  }
  bool inModel() { return model_ != nullptr; }
  void addConstraintsToModel();
  void removeFromModel();

  DblVec violations(const DblVec& x);
  double violation(const DblVec& x);

  AffExprVector eqs_;
  AffExprVector ineqs_;

private:
  Model* model_{ nullptr };
  CntVector cnts_;
  ConvexConstraints() = default;
  ConvexConstraints(const ConvexConstraints&) = default;
  ConvexConstraints& operator=(const ConvexConstraints&) = default;
  ConvexConstraints(ConvexConstraints&&) = default;
  ConvexConstraints& operator=(ConvexConstraints&&) = default;
};

/**
Non-convex cost function, which knows how to calculate its convex approximation
(convexify() method)
*/
class Cost
{
public:
  using Ptr = std::shared_ptr<Cost>;

  /** Evaluate at solution vector x*/
  virtual double value(const DblVec&) = 0;
  /** Convexify at solution vector x*/
  virtual ConvexObjective::Ptr convex(const DblVec& x, Model* model) = 0;
  /** Get problem variables associated with this cost */
  virtual VarVector getVars() = 0;
  std::string name() { return name_; }
  void setName(const std::string& name) { name_ = name; }

  Cost() = default;
  Cost(std::string name) : name_(std::move(name)) {}
  virtual ~Cost() = default;
  Cost(const Cost&) = default;
  Cost& operator=(const Cost&) = default;
  Cost(Cost&&) = default;
  Cost& operator=(Cost&&) = default;

protected:
  std::string name_{ "unnamed" };
};

/**
Non-convex vector-valued constraint function, which knows how to calculate its
convex approximation
*/
class Constraint
{
public:
  using Ptr = std::shared_ptr<Constraint>;

  /** inequality vs equality */
  virtual ConstraintType type() = 0;
  /** Evaluate at solution vector x*/
  virtual DblVec value(const DblVec& x) = 0;
  /** Convexify at solution vector x*/
  virtual ConvexConstraints::Ptr convex(const DblVec& x, Model* model) = 0;
  /** Calculate constraint violations (positive part for inequality constraint,
   * absolute value for inequality constraint)*/
  DblVec violations(const DblVec& x);
  /** Sum of violations */
  double violation(const DblVec& x);
  /** Get problem variables associated with this constraint */
  virtual VarVector getVars() = 0;
  std::string name() { return name_; }
  void setName(const std::string& name) { name_ = name; }

  Constraint() = default;
  Constraint(std::string name) : name_(std::move(name)) {}
  virtual ~Constraint() = default;
  Constraint(const Constraint&) = default;
  Constraint& operator=(const Constraint&) = default;
  Constraint(Constraint&&) = default;
  Constraint& operator=(Constraint&&) = default;

protected:
  std::string name_{ "unnamed" };
};

class EqConstraint : public Constraint
{
public:
  using Ptr = std::shared_ptr<EqConstraint>;

  ConstraintType type() override { return EQ; }
  EqConstraint() = default;
  EqConstraint(std::string name) : Constraint(std::move(name)) {}
};

class IneqConstraint : public Constraint
{
public:
  using Ptr = std::shared_ptr<IneqConstraint>;

  ConstraintType type() override { return INEQ; }
  IneqConstraint() = default;
  IneqConstraint(std::string name) : Constraint(std::move(name)) {}
};

/**
Non-convex optimization problem
*/
class OptProb
{
public:
  using Ptr = std::shared_ptr<OptProb>;

  OptProb(ModelType convex_solver = ModelType::AUTO_SOLVER);
  virtual ~OptProb() = default;
  OptProb(const OptProb&) = delete;
  OptProb& operator=(const OptProb&) = delete;
  OptProb(OptProb&&) = default;
  OptProb& operator=(OptProb&&) = default;

  /** create variables with bounds [-INFINITY, INFINITY]  */
  VarVector createVariables(const std::vector<std::string>& names);
  /** create variables with bounds [lb[i], ub[i] */
  VarVector createVariables(const std::vector<std::string>& names, const DblVec& lb, const DblVec& ub);
  /** set the lower bounds of all the variables */
  void setLowerBounds(const DblVec& lb);
  /** set the upper bounds of all the variables */
  void setUpperBounds(const DblVec& ub);
  /** set lower bounds of some of the variables */
  void setLowerBounds(const DblVec& lb, const VarVector& vars);
  /** set upper bounds of some of the variables */
  void setUpperBounds(const DblVec& ub, const VarVector& vars);
  /** Note: in the current implementation, this function just adds the
   * constraint to the
   * model. So if you're not careful, you might end up with an infeasible
   * problem. */
  void addLinearConstraint(const AffExpr&, ConstraintType type);
  /** Add nonlinear cost function */
  void addCost(Cost::Ptr);
  /** Add nonlinear constraint function */
  void addConstraint(Constraint::Ptr);
  void addEqConstraint(Constraint::Ptr);
  void addIneqConstraint(Constraint::Ptr);
  /** Find closest point to solution vector x that satisfies linear inequality
   * constraints */
  DblVec getCentralFeasiblePoint(const DblVec& x);
  /**
   * @brief Pushes the input vector within joint limits by some amount in a repeatable fashion
   * @param x Input vector. If it is already within joint limits, it will not be changed
   * @param delta (Default = 1e-3) Amount that it is pushed within the joint limits. Pushing it exactly onto a limit can
   * cause numerical problems
   * @return The input vector x with values modified to be within the joint limits
   */
  DblVec getClosestFeasiblePoint(const DblVec& x, const double& delta = 1e-3);
  /** This one does it by solving the QP with only variable limits */
  DblVec getClosestFeasiblePointQP(const DblVec& x);

  std::vector<Constraint::Ptr> getConstraints() const;
  const std::vector<Cost::Ptr>& getCosts() { return costs_; }
  const std::vector<Constraint::Ptr>& getIneqConstraints() { return ineqcnts_; }
  const std::vector<Constraint::Ptr>& getEqConstraints() { return eqcnts_; }
  const DblVec& getLowerBounds() { return lower_bounds_; }
  const DblVec& getUpperBounds() { return upper_bounds_; }
  Model::Ptr getModel() { return model_; }
  const VarVector& getVars() { return vars_; }
  int getNumCosts() { return static_cast<int>(costs_.size()); }
  int getNumConstraints() { return static_cast<int>(eqcnts_.size() + ineqcnts_.size()); }
  int getNumVars() { return static_cast<int>(vars_.size()); }

protected:
  Model::Ptr model_;
  VarVector vars_;
  DblVec lower_bounds_;
  DblVec upper_bounds_;
  std::vector<Cost::Ptr> costs_;
  std::vector<Constraint::Ptr> eqcnts_;
  std::vector<Constraint::Ptr> ineqcnts_;
};

template <typename VecType>
inline void setVec(DblVec& x, const VarVector& vars, const VecType& vals)
{
  assert(vars.size() == vals.size());
  for (size_t i = 0; i < vars.size(); ++i)
  {
    x[static_cast<size_t>(vars[i].var_rep->index)] = vals[i];
  }
}
template <typename OutVecType>
inline OutVecType getVec1(const DblVec& x, const VarVector& vars)
{
  OutVecType out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
    out[i] = x[vars[i].var_rep->index];
  return out;
}
}  // namespace sco
