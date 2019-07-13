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
  using func = std::function<double(Eigen::VectorXd)>;
  static ScalarOfVector::Ptr construct(const func&);
  //  typedef VectorXd (*c_func)(const VectorXd&);
  //  static ScalarOfVectorPtr construct(const c_func&);
};
class VectorOfVector
{
public:

  using Ptr = std::shared_ptr<VectorOfVector>;

  virtual Eigen::VectorXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::VectorXd call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~VectorOfVector() {}
  using func = std::function<Eigen::VectorXd(Eigen::VectorXd)>;
  static VectorOfVector::Ptr construct(const func&);
  //  typedef VectorXd (*c_func)(const VectorXd&);
  //  static VectorOfVectorPtr construct(const c_func&);
};
class MatrixOfVector
{
public:

  using Ptr = std::shared_ptr<MatrixOfVector>;

  virtual Eigen::MatrixXd operator()(const Eigen::VectorXd& x) const = 0;
  Eigen::MatrixXd call(const Eigen::VectorXd& x) const { return operator()(x); }
  virtual ~MatrixOfVector() {}
  using func = std::function<Eigen::MatrixXd(Eigen::VectorXd)>;
  static MatrixOfVector::Ptr construct(const func&);
  //  typedef VectorMatrixXd (*c_func)(const VectorXd&);
  //  static MatrixOfVectorPtr construct(const c_func&);
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
}
