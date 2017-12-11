#include <trajopt_sco/expr_vec_ops.hpp>
namespace sco {

AffExpr varDot(const VectorXd& x, const VarVector& v) {

  AffExpr out;
  out.constant = 0;
  out.vars = v;
  out.coeffs = vector<double>(x.data(), x.data()+x.size());
  return out;
}

}
