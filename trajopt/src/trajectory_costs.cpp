#include <Eigen/Core>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>

using namespace std;
using namespace sco;
using namespace Eigen;

namespace
{
static MatrixXd diffAxis0(const MatrixXd& in)
{
  return in.middleRows(1, in.rows() - 1) - in.middleRows(0, in.rows() - 1);
}
}

namespace trajopt
{
//////////// Quadratic cost functions /////////////////

JointPosCost::JointPosCost(const VarVector& vars, const VectorXd& vals, const VectorXd& coeffs)
  : Cost("JointPos"), vars_(vars), vals_(vals), coeffs_(coeffs)
{
  for (std::size_t i = 0; i < vars.size(); ++i)
  {
    if (coeffs[i] > 0)
    {
      AffExpr diff = exprSub(AffExpr(vars[i]), AffExpr(vals[i]));
      exprInc(expr_, exprMult(exprSquare(diff), coeffs[i]));
    }
  }
}
double JointPosCost::value(const vector<double>& xvec)
{
  VectorXd dofs = getVec(xvec, vars_);
  return ((dofs - vals_).array().square() * coeffs_.array()).sum();
}
ConvexObjectivePtr JointPosCost::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointVelCost::JointVelCost(const VarArray& vars, const VectorXd& coeffs)
  : Cost("JointVel"), vars_(vars), coeffs_(coeffs)
{
  for (int i = 0; i < vars.rows() - 1; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      AffExpr vel;
      exprInc(vel, exprMult(vars(i, j), -1));
      exprInc(vel, exprMult(vars(i + 1, j), 1));
      exprInc(expr_, exprMult(exprSquare(vel), coeffs_[j]));
    }
  }
}
double JointVelCost::value(const vector<double>& xvec)
{
  MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(traj).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointVelCost::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointAccCost::JointAccCost(const VarArray& vars, const VectorXd& coeffs)
  : Cost("JointAcc"), vars_(vars), coeffs_(coeffs)
{
  for (int i = 0; i < vars.rows() - 2; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      AffExpr acc;
      exprInc(acc, exprMult(vars(i, j), 1.0));
      exprInc(acc, exprMult(vars(i + 1, j), -2.0));
      exprInc(acc, exprMult(vars(i + 2, j), 1.0));
      exprInc(expr_, exprMult(exprSquare(acc), coeffs_[j]));
    }
  }
}
double JointAccCost::value(const vector<double>& xvec)
{
  MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(diffAxis0(traj)).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointAccCost::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}

JointJerkCost::JointJerkCost(const VarArray& vars, const VectorXd& coeffs)
  : Cost("JointJerk"), vars_(vars), coeffs_(coeffs)
{
  for (int i = 0; i < vars.rows() - 4; ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      AffExpr acc;
      exprInc(acc, exprMult(vars(i, j), -1.0 / 2.0));
      exprInc(acc, exprMult(vars(i + 1, j), 1.0));
      exprInc(acc, exprMult(vars(i + 2, j), 0.0));
      exprInc(acc, exprMult(vars(i + 3, j), -1.0));
      exprInc(acc, exprMult(vars(i + 4, j), 1.0 / 2.0));
      exprInc(expr_, exprMult(exprSquare(acc), coeffs_[j]));
    }
  }
}
double JointJerkCost::value(const vector<double>& xvec)
{
  MatrixXd traj = getTraj(xvec, vars_);
  return (diffAxis0(diffAxis0(diffAxis0(traj))).array().square().matrix() * coeffs_.asDiagonal()).sum();
}
ConvexObjectivePtr JointJerkCost::convex(const vector<double>& /*x*/, Model* model)
{
  ConvexObjectivePtr out(new ConvexObjective(model));
  out->addQuadExpr(expr_);
  return out;
}
}
