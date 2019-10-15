#include <trajopt_sco/num_diff.hpp>

namespace sco
{
ScalarOfVector::Ptr ScalarOfVector::construct(const func& f)
{
  struct F : public ScalarOfVector
  {
    func f;
    F(const func& _f) : f(_f) {}
    double operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  ScalarOfVector* sov = new F(f);  // to avoid erroneous clang warning
  return ScalarOfVector::Ptr(sov);
}

VectorOfVector::Ptr VectorOfVector::construct(const func& f)
{
  struct F : public VectorOfVector
  {
    func f;
    F(const func& _f) : f(_f) {}
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  VectorOfVector* vov = new F(f);  // to avoid erroneous clang warning
  return VectorOfVector::Ptr(vov);
}

MatrixOfVector::Ptr MatrixOfVector::construct(const func& f)
{
  struct F : public MatrixOfVector
  {
    func f;
    F(const func& _f) : f(_f) {}
    Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  MatrixOfVector* mov = new F(f);  // to avoid erroneous clang warning
  return MatrixOfVector::Ptr(mov);
}

Eigen::VectorXd calcForwardNumGrad(const ScalarOfVector& f, const Eigen::VectorXd& x, double epsilon)
{
  Eigen::VectorXd out(x.size());
  Eigen::VectorXd xpert = x;
  double y = f(x);
  for (int i = 0; i < x.size(); ++i)
  {
    xpert(i) = x(i) + epsilon;
    double ypert = f(xpert);
    out(i) = (ypert - y) / epsilon;
    xpert(i) = x(i);
  }
  return out;
}
Eigen::MatrixXd calcForwardNumJac(const VectorOfVector& f, const Eigen::VectorXd& x, double epsilon)
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
  return out;
}

void calcGradAndDiagHess(const ScalarOfVector& f,
                         const Eigen::VectorXd& x,
                         double epsilon,
                         double& y,
                         Eigen::VectorXd& grad,
                         Eigen::VectorXd& hess)
{
  y = f(x);
  grad.resize(x.size());
  hess.resize(x.size());
  Eigen::VectorXd xpert = x;
  for (int i = 0; i < x.size(); ++i)
  {
    xpert(i) = x(i) + epsilon / 2;
    double yplus = f(xpert);
    xpert(i) = x(i) - epsilon / 2;
    double yminus = f(xpert);
    grad(i) = (yplus - yminus) / epsilon;
    hess(i) = (yplus + yminus - 2 * y) / (epsilon * epsilon / 4);
    xpert(i) = x(i);
  }
}

void calcGradHess(ScalarOfVector::Ptr f,
                  const Eigen::VectorXd& x,
                  double epsilon,
                  double& y,
                  Eigen::VectorXd& grad,
                  Eigen::MatrixXd& hess)
{
  y = f->call(x);
  VectorOfVector::Ptr grad_func = forwardNumGrad(f, epsilon);
  grad = grad_func->call(x);
  hess = calcForwardNumJac(*grad_func, x, epsilon);
  hess = (hess + hess.transpose()) / 2;
}

struct ForwardNumGrad : public VectorOfVector
{
  ScalarOfVector::Ptr f_;
  double epsilon_;
  ForwardNumGrad(ScalarOfVector::Ptr f, double epsilon) : f_(f), epsilon_(epsilon) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& x) const override { return calcForwardNumGrad(*f_, x, epsilon_); }
};

struct ForwardNumJac : public MatrixOfVector
{
  VectorOfVector::Ptr f_;
  double epsilon_;
  ForwardNumJac(VectorOfVector::Ptr f, double epsilon) : f_(f), epsilon_(epsilon) {}
  Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const override { return calcForwardNumJac(*f_, x, epsilon_); }
};

VectorOfVector::Ptr forwardNumGrad(ScalarOfVector::Ptr f, double epsilon)
{
  return VectorOfVector::Ptr(new ForwardNumGrad(f, epsilon));
}
MatrixOfVector::Ptr forwardNumJac(VectorOfVector::Ptr f, double epsilon)
{
  return MatrixOfVector::Ptr(new ForwardNumJac(f, epsilon));
}
}  // namespace sco
