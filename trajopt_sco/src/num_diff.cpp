#include <trajopt_sco/num_diff.hpp>

namespace sco
{
ScalarOfVector::Ptr ScalarOfVector::construct(func f)
{
  struct F : public ScalarOfVector
  {
    func f;
    F(func _f) : f(std::move(_f)) {}
    double operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  auto sov = std::make_shared<F>(std::move(f));
  return sov;
}

VectorOfVector::Ptr VectorOfVector::construct(func f)
{
  struct F : public VectorOfVector
  {
    func f;
    F(func _f) : f(std::move(_f)) {}
    Eigen::VectorXd operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  auto vov = std::make_shared<F>(std::move(f));
  return vov;
}

MatrixOfVector::Ptr MatrixOfVector::construct(func f)
{
  struct F : public MatrixOfVector
  {
    func f;
    F(func _f) : f(std::move(_f)) {}
    Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const override { return f(x); }
  };
  auto mov = std::make_shared<F>(std::move(f));
  return mov;
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

void calcGradHess(const ScalarOfVector::Ptr& f,
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
  ForwardNumGrad(ScalarOfVector::Ptr f, double epsilon) : f_(std::move(f)), epsilon_(epsilon) {}
  Eigen::VectorXd operator()(const Eigen::VectorXd& x) const override { return calcForwardNumGrad(*f_, x, epsilon_); }
};

struct ForwardNumJac : public MatrixOfVector
{
  VectorOfVector::Ptr f_;
  double epsilon_;
  ForwardNumJac(VectorOfVector::Ptr f, double epsilon) : f_(std::move(f)), epsilon_(epsilon) {}
  Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const override { return calcForwardNumJac(*f_, x, epsilon_); }
};

VectorOfVector::Ptr forwardNumGrad(ScalarOfVector::Ptr f, double epsilon)
{
  return std::make_shared<ForwardNumGrad>(std::move(f), epsilon);
}
MatrixOfVector::Ptr forwardNumJac(VectorOfVector::Ptr f, double epsilon)
{
  return std::make_shared<ForwardNumJac>(std::move(f), epsilon);
}
}  // namespace sco
