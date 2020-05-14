#ifndef TRAJOPT_IFOPT_NUMERIC_DIFFERENTIATION_H
#define TRAJOPT_IFOPT_NUMERIC_DIFFERENTIATION_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Eigen>
#include <ifopt/cost_term.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt
{
using ErrorCalculator = std::function<Eigen::VectorXd(const Eigen::Ref<Eigen::VectorXd>&)>;
using Jacobian = Eigen::SparseMatrix<double, Eigen::RowMajor>;

/**
 * @brief Calculates the jacobian of the given error calculator using forward numeric differentiation
 * @param f Input error calculator, Eigen::VectorXd(const Eigen::Ref<Eigen::VectorXd>&)
 * @param x Point about which f is calculated
 * @param epsilon Amount x is perturbed
 * @return The resulting jacobian. If f(x) = y, jac.size = [y.size(), x.size()]
 */
inline Jacobian calcForwardNumJac(const ErrorCalculator& f, const Eigen::Ref<Eigen::VectorXd>& x, double epsilon)
{
  Eigen::VectorXd y = f(x);
  Eigen::MatrixXd out(y.size(), x.size());
  Eigen::VectorXd xpert = x;
  for (int i = 0; i < x.size(); ++i)
  {
    xpert(i) = x(i) + epsilon;
    Eigen::VectorXd ypert = f(xpert);
    out.col(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out.sparseView();
}
}  // namespace trajopt

#endif
