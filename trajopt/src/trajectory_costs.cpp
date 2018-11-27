#include <Eigen/Core>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

namespace
{
/** @brief Returns the difference between each row of a matrixXd and the row before */
static Eigen::MatrixXd diffAxis0(const Eigen::MatrixXd& in)
{
  return in.middleRows(1, in.rows() - 1) - in.middleRows(0, in.rows() - 1);
}
}  // namespace

namespace trajopt
{
//////////// Joint cost functions /////////////////

//////////////////// Position /////////////////////
JointPosCost::JointPosCost(const VarVector& vars, const VectorXd& vals, const VectorXd& coeffs)
  : Cost("JointPos"), vars_(vars), vals_(vals), coeffs_(coeffs)
{
  for (std::size_t i = 0; i < vars.size(); ++i)
  {
    if (coeffs[i] > 0)
    {
      sco::AffExpr diff = sco::exprSub(sco::AffExpr(vars[i]), sco::AffExpr(vals[i]));
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(diff), coeffs[i]));
    }
  }
}
double JointPosCost::value(const DblVec& xvec)
{
  Eigen::VectorXd dofs = sco::getVec(xvec, vars_);
  return ((dofs - vals_).array().square() * coeffs_.array()).sum();
}
sco::ConvexObjectivePtr JointPosCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

//////////////////// Velocity /////////////////////
JointVelEqCost::JointVelEqCost(const VarArray& vars,
                               const VectorXd& coeffs,
                               const VectorXd& targs,
                               int& first_step,
                               int& last_step)
  : Cost("JointVel"), vars_(vars), coeffs_(coeffs), targs_(targs), first_step_(first_step), last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      sco::AffExpr vel;
      sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
      sco::exprInc(vel, sco::exprMult(vars(i + 1, j), 1));
      exprDec(vel, targs_[j]);
      // expr_ = coeff * vel^2
      exprInc(expr_, exprMult(exprSquare(vel), coeffs_[j]));
    }
  }
}
double JointVelEqCost::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = (getTraj(xvec, vars_));
  // Takes diff b/n the subsequent rows and subtract each row by the targs_ vector
  MatrixXd diff =
      (diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()))).rowwise() - targs_.transpose();
  // Element-wise square it, multiply it by a diagonal matrix of coefficients, and sums output
  return (diff.array().square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointVelEqCost::convex(const vector<double>& /*x*/, Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointVelIneqCost::JointVelIneqCost(const VarArray& vars,
                                   const VectorXd& coeffs,
                                   const VectorXd& targs,
                                   const VectorXd& upper_tols,
                                   const VectorXd& lower_tols,
                                   int& first_step,
                                   int& last_step)
  : Cost("JointVel")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targs_(targs)
  , first_step_(first_step)
  , last_step_(last_step)
{
  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      AffExpr expr;
      exprInc(vel, exprMult(vars(i, j), 1));
      exprInc(vel, exprMult(vars(i + 1, j), -1));

      exprDec(vel, targs_[j]);        // offset to center about 0
      exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      exprDec(expr, vel);             // expr = upper_tol_- (vel - targs_)
      expr = exprMult(expr, -1);             // expr = - (upper_tol_- (vel - targs_))
      expr = exprMult(expr, coeffs[j]);      // expr = - (upper_tol_- (vel - targs_)) * coeffs_
      expr_vec_.push_back(expr);
    }
  }

  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      AffExpr expr_neg;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);             // offset to center about 0
      exprInc(expr_neg, lower_tols_[j]);   // expr_ = lower_tol_
      exprDec(expr_neg, vel);              // expr = lower_tol_- (vel - targs_)
      expr_neg = exprMult(expr_neg, coeffs[j]);       // expr = (lower_tol_- (vel - targs_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

double JointVelIneqCost::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  MatrixXd vel = diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()));
  // Subtract targets to center about 0 and then subtract from tolerance
  MatrixXd diff0 = (vel.rowwise() - targs_.transpose());
  MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  return diff1.cwiseMax(0).sum() + diff2.cwiseMax(0).sum();
}

ConvexObjectivePtr JointVelIneqCost::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (AffExpr expr : expr_vec_)
  {
    out->addHinge(expr, 1);
  }
  return out;
}

JointVelEqConstraint::JointVelEqConstraint(const VarArray& vars,
                                           const VectorXd& coeffs,
                                           const VectorXd& targs,
                                           int& first_step,
                                           int& last_step)
  : EqConstraint("JointVel")
  , vars_(vars)
  , coeffs_(coeffs)
  , targs_(targs)
  , first_step_(first_step)
  , last_step_(last_step)
{
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));
      exprDec(vel, targs_[j]);
      // expr_ = coeff * vel - Not squared b/c QuadExpr cnt not yet supported (TODO)
      expr_vec_.push_back(exprMult(vel, coeffs_[j]));
    }
  }
}

vector<double> JointVelEqConstraint::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targs_ vector
  MatrixXd diff =
      (diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()))).rowwise() - targs_.transpose();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and converts to vector
  return toDblVec((diff.array().square()).matrix() * coeffs_.asDiagonal());
}
ConvexConstraintsPtr JointVelEqConstraint::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  for (AffExpr expr : expr_vec_)
  {
  out->addEqCnt(expr);
  }
  return out;
}

JointVelIneqConstraint::JointVelIneqConstraint(const VarArray& vars,
                                               const VectorXd& coeffs,
                                               const VectorXd& targs,
                                               const VectorXd& upper_tols,
                                               const VectorXd& lower_tols,
                                               int& first_step,
                                               int& last_step)
  : IneqConstraint("JointVel")
  , vars_(vars)
  , coeffs_(coeffs)
  , upper_tols_(upper_tols)
  , lower_tols_(lower_tols)
  , targs_(targs)
  , first_step_(first_step)
  , last_step_(last_step)
{
  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      AffExpr expr;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);         // offset to center about 0
      exprInc(expr, upper_tols_[j]);  // expr_ = upper_tol
      exprDec(expr, vel);             // expr = upper_tol_- (vel - targs_)
      expr = exprMult(expr, -1);             // expr = - (upper_tol_- (vel - targs_))
      expr = exprMult(expr, coeffs[j]);      // expr = - (upper_tol_- (vel - targs_)) * coeffs_
      expr_vec_.push_back(expr);
    }
  }

  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = first_step_; i <= last_step_ - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      AffExpr expr_neg;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);             // offset to center about 0
      exprInc(expr_neg, lower_tols_[j]);   // expr_ = lower_tol_
      exprDec(expr_neg, vel);              // expr = lower_tol_- (vel - targs_)
      expr_neg = exprMult(expr_neg, coeffs[j]);       // expr = (lower_tol_- (vel - targs_)) * coeffs_
      expr_vec_.push_back(expr_neg);
    }
  }
}

vector<double> JointVelIneqConstraint::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  MatrixXd vel = diffAxis0(traj.block(first_step_, 0, last_step_ - first_step_ + 1, traj.cols()));
  // Subtract targets to center about 0 and then subtract from tolerance
  MatrixXd diff0 = (vel.rowwise() - targs_.transpose());
  MatrixXd diff1 = (diff0.rowwise() - upper_tols_.transpose()) * coeffs_.asDiagonal();
  MatrixXd diff2 = ((diff0 * -1).rowwise() + lower_tols_.transpose()) * coeffs_.asDiagonal();
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to
  // vector
  MatrixXd out(diff1.rows(), diff1.cols() + diff2.cols());
  out << diff1, diff2;
  return toDblVec(out);
}

ConvexConstraintsPtr JointVelIneqConstraint::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  for (AffExpr expr : expr_vec_)
  {
    out->addIneqCnt(expr);
  }
  return out;
}

//////////////////// Acceleration /////////////////////
JointAccCost::JointAccCost(const VarArray& vars, const VectorXd& coeffs)
  : Cost("JointAcc"), vars_(vars), coeffs_(coeffs)
{
  for (int i = 0; i < vars.rows() - 2; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr acc;
      sco::exprInc(acc, sco::exprMult(vars(i, j), 1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 1, j), -2.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 2, j), 1.0));
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(acc), coeffs_[j]));
    }
  }
}
double JointAccCost::value(const DblVec& xvec)
{
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(diffAxis0(traj)).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointAccCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

//////////////////// Jerk /////////////////////
JointJerkCost::JointJerkCost(const VarArray& vars, const VectorXd& coeffs)
  : Cost("JointJerk"), vars_(vars), coeffs_(coeffs)
{
  for (int i = 0; i < vars.rows() - 4; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr acc;
      sco::exprInc(acc, sco::exprMult(vars(i, j), -1.0 / 2.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 1, j), 1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 2, j), 0.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 3, j), -1.0));
      sco::exprInc(acc, sco::exprMult(vars(i + 4, j), 1.0 / 2.0));
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(acc), coeffs_[j]));
    }
  }
}
double JointJerkCost::value(const DblVec& xvec)
{
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(diffAxis0(diffAxis0(traj))).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointJerkCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}
}  // namespace trajopt
