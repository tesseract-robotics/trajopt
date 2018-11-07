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
JointVelEqCost::JointVelEqCost(const VarArray& vars, const VectorXd& coeffs, const VectorXd& targs)
  : Cost("JointVel"), vars_(vars), coeffs_(coeffs), targs_(targs)
{
  for (int i = 0; i < vars.rows() - 1; ++i)
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
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targs_ vector
  ArrayXd diff = diffAxis0(traj).array().rowwise() - targs_.transpose().array();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and sums output
  return (diff.square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointVelEqCost::convex(const vector<double>& /*x*/, Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  //  out->addHinge();
  //  out->addAbs();
  // etc
  return out;
}

JointVelIneqCost::JointVelIneqCost(const VarArray& vars,
                                   const VectorXd& coeffs,
                                   const VectorXd& targs,
                                   const VectorXd& upper_tols,
                                   const VectorXd& lower_tols)
  : Cost("JointVel"), vars_(vars), coeffs_(coeffs), targs_(targs), upper_tols_(upper_tols), lower_tols_(lower_tols)
{
  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);        // offset to center about 0
      exprInc(expr_, upper_tols_[j]);  // expr_ = upper_tol
      exprDec(expr_, vel);           // expr = upper_tol_- (vel - targs_)
      exprMult(expr_, -1);            // expr = - (upper_tol_- (vel - targs_))
      exprMult(expr_, coeffs[j]);     // expr = - (upper_tol_- (vel - targs_)) * coeffs_
    }
  }

  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);            // offset to center about 0
      exprInc(expr_neg_, lower_tols_[j]);  // expr_ = lower_tol_
      exprDec(expr_neg_, vel);           // expr = lower_tol_- (vel - targs_)
      exprMult(expr_neg_, coeffs[j]);     // expr = (lower_tol_- (vel - targs_)) * coeffs_
    }
  }
}

double JointVelIneqCost::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  VectorXd vel = diffAxis0(traj);
  // Subtract targets to center about 0 and then subtract from tolerance
  ArrayXd diff1 = -1 * upper_tols_.transpose().array() - (vel.array().rowwise() - targs_.transpose().array());
  ArrayXd diff2 = lower_tols_.transpose().array() - (vel.array().rowwise() - targs_.transpose().array());
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, and sums outputs of the two
  return (diff1.matrix().cwiseMax(0) * coeffs_.asDiagonal()).sum() +
         (diff2.matrix().cwiseMax(0) * coeffs_.asDiagonal()).sum();
}

ConvexObjectivePtr JointVelIneqCost::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  out->addHinge(expr_, 1);
  out->addHinge(expr_neg_, 1);
  return out;
}



JointVelEqConstraint::JointVelEqConstraint(const VarArray& vars, const VectorXd& coeffs, const VectorXd& targs)
  : EqConstraint("JointVel"), vars_(vars), coeffs_(coeffs), targs_(targs)
{
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));
      exprDec(vel, targs_[j]);
      // expr_ = coeff * vel^2
      exprInc(expr_, exprMult(vel, coeffs_[j]));
    }
  }
}
vector<double> JointVelEqConstraint::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows and subtract each row by the targs_ vector
  ArrayXd diff = diffAxis0(traj).array().rowwise() - targs_.transpose().array();
  // Squares it, multiplies it by a diagonal matrix of coefficients, and converts to vector
  return toDblVec((diff.square().matrix() * coeffs_.asDiagonal()).matrix());
}
ConvexConstraintsPtr JointVelEqConstraint::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  out->addEqCnt(expr_);

  return out;
}

JointVelIneqConstraint::JointVelIneqConstraint(const VarArray& vars,
                                   const VectorXd& coeffs,
                                   const VectorXd& targs,
                                   const VectorXd& upper_tols,
                                   const VectorXd& lower_tols)
  : IneqConstraint("JointVel"), vars_(vars), coeffs_(coeffs), targs_(targs), upper_tols_(upper_tols), lower_tols_(lower_tols)
{
  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);        // offset to center about 0
      exprInc(expr_, upper_tols_[j]);  // expr_ = upper_tol
      exprDec(expr_, vel);           // expr = upper_tol_- (vel - targs_)
      exprMult(expr_, -1);            // expr = - (upper_tol_- (vel - targs_))
      exprMult(expr_, coeffs[j]);     // expr = - (upper_tol_- (vel - targs_)) * coeffs_
    }
  }

  // Form upper limit expr = - (upper_tol-(vel-targ))
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      // vel = (x2 - x1) - targ
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));

      exprDec(vel, targs_[j]);            // offset to center about 0
      exprInc(expr_neg_, lower_tols_[j]);  // expr_ = lower_tol_
      exprDec(expr_neg_, vel);           // expr = lower_tol_- (vel - targs_)
      exprMult(expr_neg_, coeffs[j]);     // expr = (lower_tol_- (vel - targs_)) * coeffs_
    }
  }
}

vector<double> JointVelIneqConstraint::value(const vector<double>& xvec)
{
  // Convert vector from optimization to trajectory
  MatrixXd traj = getTraj(xvec, vars_);
  // Takes diff b/n the subsequent rows to get velocity
  VectorXd vel = diffAxis0(traj);
  // Subtract targets to center about 0 and then subtract from tolerance
  ArrayXd diff1 = -1 * upper_tols_.transpose().array() - (vel.array().rowwise() - targs_.transpose().array());
  ArrayXd diff2 = lower_tols_.transpose().array() - (vel.array().rowwise() - targs_.transpose().array());
  // Applies hinge, multiplies it by a diagonal matrix of coefficients, sums each corresponding value, and converts to vector
//  auto diff3 = (diff1.matrix().cwiseMax(0) * coeffs_.asDiagonal()) + (diff2.matrix().cwiseMax(0) * coeffs_.asDiagonal());  // Should be and ArrayXd
  return toDblVec(((diff1.matrix().cwiseMax(0) * coeffs_.asDiagonal()) + (diff2.matrix().cwiseMax(0) * coeffs_.asDiagonal())).matrix());
  //TODO: Constraints need to return a vector of doubles. One for each joint/timestep. If one joint is really big and one really small, we don't want them to cancel and not trigger cnt
}

ConvexConstraintsPtr JointVelIneqConstraint::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexConstraintsPtr out(new ConvexConstraints(model));
  // Add hinge cost. Set the coefficient to 1 here since we include it in the AffExpr already
  // This is necessary since we want a seperate coefficient per joint
  out->addIneqCnt(expr_);
  out->addIneqCnt(expr_neg_);
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
