#include <Eigen/Core>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>

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
//////////// Quadratic cost functions /////////////////

JointPosCost::JointPosCost(const sco::VarVector& vars, const Eigen::VectorXd& vals, const Eigen::VectorXd& coeffs)
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

JointVelCost::JointVelCost(const VarArray& vars, const Eigen::VectorXd& coeffs)
  : Cost("JointVel"), vars_(vars), coeffs_(coeffs)
{
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      sco::AffExpr vel;
      sco::exprInc(vel, sco::exprMult(vars(i, j), -1));
      sco::exprInc(vel, sco::exprMult(vars(i + 1, j), 1));
      sco::exprInc(expr_, sco::exprMult(sco::exprSquare(vel), coeffs_[j]));
    }
  }
}
double JointVelCost::value(const DblVec& xvec)
{
  Eigen::MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(traj).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
sco::ConvexObjectivePtr JointVelCost::convex(const DblVec& /*x*/, sco::Model* model)
{
  sco::ConvexObjectivePtr out(new sco::ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointAccCost::JointAccCost(const VarArray& vars, const Eigen::VectorXd& coeffs)
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

JointJerkCost::JointJerkCost(const VarArray& vars, const Eigen::VectorXd& coeffs)
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
}
