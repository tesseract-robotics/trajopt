#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <functional>
#include <memory>
TRAJOPT_IGNORE_WARNINGS_POP

/*
 * Numerical derivatives
 */

namespace sco
{
class ScalarOfVector
{
public:
  using Ptr = std::shared_ptr<ScalarOfVector>;

  virtual double operator()(const Eigen::VectorXd& x) const = 0;
  double call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~ScalarOfVector() {}
  using func = std::function<double(const Eigen::VectorXd&)>;
  static ScalarOfVector::Ptr construct(const func&);
};
class VectorOfVector
{
public:
  using Ptr = std::shared_ptr<VectorOfVector>;

  virtual Eigen::VectorXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::VectorXd call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~VectorOfVector() {}
  using func = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
  static VectorOfVector::Ptr construct(const func&);
};
class MatrixOfVector
{
public:
  using Ptr = std::shared_ptr<MatrixOfVector>;

  virtual Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::MatrixXd call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~MatrixOfVector() {}
  using func = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
  static MatrixOfVector::Ptr construct(const func&);
};

Eigen::VectorXd calcForwardNumGrad(const ScalarOfVector& f, const Eigen::VectorXd& x, double epsilon);
Eigen::MatrixXd calcForwardNumJac(const VectorOfVector& f, const Eigen::VectorXd& x, double epsilon);
void calcGradAndDiagHess(const ScalarOfVector& f,
                         const Eigen::VectorXd& x,
                         double epsilon,
                         double& y,
                         Eigen::VectorXd& grad,
                         Eigen::VectorXd& hess);
void calcGradHess(ScalarOfVector::Ptr f,
                  const Eigen::VectorXd& x,
                  double epsilon,
                  double& y,
                  Eigen::VectorXd& grad,
                  Eigen::MatrixXd& hess);
VectorOfVector::Ptr forwardNumGrad(ScalarOfVector::Ptr f, double epsilon);
MatrixOfVector::Ptr forwardNumJac(VectorOfVector::Ptr f, double epsilon);
}  // namespace sco
