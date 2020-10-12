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

  ScalarOfVector() = default;
  virtual ~ScalarOfVector() = default;
  ScalarOfVector(const ScalarOfVector&) = default;
  ScalarOfVector& operator=(const ScalarOfVector&) = default;
  ScalarOfVector(ScalarOfVector&&) = default;
  ScalarOfVector& operator=(ScalarOfVector&&) = default;

  virtual double operator()(const Eigen::VectorXd& x) const = 0;
  double call(const Eigen::VectorXd& x) const { return operator()(x); }

  using func = std::function<double(const Eigen::VectorXd&)>;
  static ScalarOfVector::Ptr construct(func f);
};

class VectorOfVector
{
public:
  using Ptr = std::shared_ptr<VectorOfVector>;

  VectorOfVector() = default;
  virtual ~VectorOfVector() = default;
  VectorOfVector(const VectorOfVector&) = default;
  VectorOfVector& operator=(const VectorOfVector&) = default;
  VectorOfVector(VectorOfVector&&) = default;
  VectorOfVector& operator=(VectorOfVector&&) = default;

  virtual Eigen::VectorXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::VectorXd call(const Eigen::VectorXd& x) const { return operator()(x); }

  using func = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
  static VectorOfVector::Ptr construct(func f);
};

class MatrixOfVector
{
public:
  using Ptr = std::shared_ptr<MatrixOfVector>;

  MatrixOfVector() = default;
  virtual ~MatrixOfVector() = default;
  MatrixOfVector(const MatrixOfVector&) = default;
  MatrixOfVector& operator=(const MatrixOfVector&) = default;
  MatrixOfVector(MatrixOfVector&&) = default;
  MatrixOfVector& operator=(MatrixOfVector&&) = default;

  virtual Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::MatrixXd call(const Eigen::VectorXd& x) const { return operator()(x); }

  using func = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;
  static MatrixOfVector::Ptr construct(func f);
};

Eigen::VectorXd calcForwardNumGrad(const ScalarOfVector& f, const Eigen::VectorXd& x, double epsilon);
Eigen::MatrixXd calcForwardNumJac(const VectorOfVector& f, const Eigen::VectorXd& x, double epsilon);
void calcGradAndDiagHess(const ScalarOfVector& f,
                         const Eigen::VectorXd& x,
                         double epsilon,
                         double& y,
                         Eigen::VectorXd& grad,
                         Eigen::VectorXd& hess);
void calcGradHess(const ScalarOfVector::Ptr& f,
                  const Eigen::VectorXd& x,
                  double epsilon,
                  double& y,
                  Eigen::VectorXd& grad,
                  Eigen::MatrixXd& hess);
VectorOfVector::Ptr forwardNumGrad(ScalarOfVector::Ptr f, double epsilon);
MatrixOfVector::Ptr forwardNumJac(VectorOfVector::Ptr f, double epsilon);
}  // namespace sco
