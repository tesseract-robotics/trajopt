#include <Eigen/Core>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_sco/modeling_utils.hpp>

#include <ros/ros.h>

using namespace std;
using namespace sco;
using namespace Eigen;

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

}
