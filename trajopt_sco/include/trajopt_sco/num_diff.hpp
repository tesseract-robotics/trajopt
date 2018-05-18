#pragma once
#include <functional>
#include <Eigen/Dense>
#include <memory>
/*
 * Numerical derivatives
 */

namespace sco {

using Eigen::VectorXd;
using Eigen::MatrixXd;
class ScalarOfVector;
class VectorOfVector;
class MatrixOfVector;
typedef std::shared_ptr<ScalarOfVector> ScalarOfVectorPtr;
typedef std::shared_ptr<VectorOfVector> VectorOfVectorPtr;
typedef std::shared_ptr<MatrixOfVector> MatrixOfVectorPtr;

class ScalarOfVector {
public:
  virtual double operator()(const VectorXd& x) const = 0;
  double call(const VectorXd& x) const {return operator()(x);}
  virtual ~ScalarOfVector() {}

  typedef std::function<double(VectorXd)> func;
  static ScalarOfVectorPtr construct(const func&);
  //  typedef VectorXd (*c_func)(const VectorXd&);
  //  static ScalarOfVectorPtr construct(const c_func&);

};
class VectorOfVector {
public:
  virtual VectorXd operator()(const VectorXd& x) const = 0;
  VectorXd call(const VectorXd& x) const {return operator()(x);}
  virtual ~VectorOfVector() {}

  typedef std::function<VectorXd(VectorXd)> func;
  static VectorOfVectorPtr construct(const func&);
  //  typedef VectorXd (*c_func)(const VectorXd&);
  //  static VectorOfVectorPtr construct(const c_func&);

};
class MatrixOfVector {
public:
  virtual MatrixXd operator()(const VectorXd& x) const = 0;
  MatrixXd call(const VectorXd& x) const {return operator()(x);}
  virtual ~MatrixOfVector() {}

  typedef std::function<MatrixXd(VectorXd)> func;
  static MatrixOfVectorPtr construct(const func&);
  //  typedef VectorMatrixXd (*c_func)(const VectorXd&);
  //  static MatrixOfVectorPtr construct(const c_func&);
};


VectorXd calcForwardNumGrad(const ScalarOfVector& f, const VectorXd& x, double epsilon);
MatrixXd calcForwardNumJac(const VectorOfVector& f, const VectorXd& x, double epsilon);
void calcGradAndDiagHess(const ScalarOfVector& f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, VectorXd& hess);
void calcGradHess(ScalarOfVectorPtr f, const VectorXd& x, double epsilon,
    double& y, VectorXd& grad, MatrixXd& hess);
VectorOfVectorPtr forwardNumGrad(ScalarOfVectorPtr f, double epsilon);
MatrixOfVectorPtr forwardNumJac(VectorOfVectorPtr f, double epsilon);



}
