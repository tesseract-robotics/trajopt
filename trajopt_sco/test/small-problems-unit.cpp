#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <cmath>
#include <boost/format.hpp>
#include <iostream>
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <sstream>

using namespace util;
using namespace std;
using namespace sco;
using namespace Eigen;



void setupProblem(OptProbPtr& probptr, size_t nvars) {
  probptr.reset(new OptProb());
  vector<string> var_names;
  for (size_t i=0; i < nvars; ++i) {
    var_names.push_back( (boost::format("x_%i")%i).str() );
  }
  probptr->createVariables(var_names);
}


void expectAllNear(const DblVec& x, const DblVec& y, double abstol) {
  EXPECT_EQ(x.size(), y.size());
  stringstream ss;
  LOG_INFO("checking %s ?= %s", CSTR(x), CSTR(y));
  for (size_t i=0; i < x.size(); ++i) EXPECT_NEAR(x[i], y[i], abstol);
}

double f_QuadraticSeparable(const VectorXd& x) {
  return x(0)*x(0) + sq(x(1) - 1) + sq(x(2)-2);
}
TEST(SQP, QuadraticSeparable)  {
  // if the problem is exactly a QP, it should be solved in one iteration
  OptProbPtr prob;
  setupProblem(prob, 3);
  prob->addCost(CostPtr(new CostFromFunc(ScalarOfVector::construct(&f_QuadraticSeparable), prob->getVars(), "f")));
  BasicTrustRegionSQP solver(prob);
  solver.trust_box_size_ = 100;
  vector<double> x = list_of(3)(4)(5);
  solver.initialize(x);
  OptStatus status = solver.optimize();
  ASSERT_EQ(status, OPT_CONVERGED);
  expectAllNear(solver.x(), list_of(0)(1)(2), 1e-3);
  // todo: checks on number of iterations and function evaluates
}
double f_QuadraticNonseparable(const VectorXd& x) {
  return sq(x(0) - x(1) + 3*x(2)) + sq(x(0)-1) + sq(x(2) - 2);
}
TEST(SQP, QuadraticNonseparable)  {
  OptProbPtr prob;
  setupProblem(prob, 3);
  prob->addCost(CostPtr(new CostFromFunc(ScalarOfVector::construct(&f_QuadraticNonseparable), prob->getVars(), "f",  true)));
  BasicTrustRegionSQP solver(prob);
  solver.trust_box_size_ = 100;
  solver.min_trust_box_size_ = 1e-5;
  solver.min_approx_improve_ = 1e-6;
  vector<double> x = list_of(3)(4)(5);
  solver.initialize(x);
  OptStatus status = solver.optimize();
  ASSERT_EQ(status, OPT_CONVERGED);
  expectAllNear(solver.x(), list_of(1)(7)(2), .01);
  // todo: checks on number of iterations and function evaluates
}


void testProblem(ScalarOfVectorPtr f, VectorOfVectorPtr g, ConstraintType cnt_type,
  const DblVec& init, const DblVec& sol) {
    OptProbPtr prob;
    size_t n = init.size();
    assert (sol.size() == n);
    setupProblem(prob, n);
    prob->addCost(CostPtr(new CostFromFunc(f, prob->getVars(), "f", true)));
    prob->addConstraint(ConstraintPtr(new ConstraintFromFunc(g, prob->getVars(), VectorXd(), cnt_type,"g")));
    BasicTrustRegionSQP solver(prob);
    solver.max_iter_ = 1000;
    solver.min_trust_box_size_ = 1e-5;
    solver.min_approx_improve_ = 1e-10;
    solver.merit_error_coeff_ = 1;
    
    solver.initialize(init);
    OptStatus status = solver.optimize();
    EXPECT_EQ(status, OPT_CONVERGED);
    expectAllNear(solver.x(), sol, .01);
}
// http://www.ai7.uni-bayreuth.de/test_problem_coll.pdf

double f_TP1(const VectorXd& x) {
  return 1*sq(x(1)-sq(x(0))) + sq(1-x(0));
}
VectorXd g_TP1(const VectorXd& x) {
  VectorXd out(1);
  out(0) = -1.5 - x(1);
  return out;
}
double f_TP2(const VectorXd& x) {
  return 100*sq(x(1)-sq(x(0))) + sq(1-x(0));
}
VectorXd g_TP2(const VectorXd& x) {
  VectorXd out(1);
  out(0) = -1.5 - x(1);
  return out;
}
double f_TP3(const VectorXd& x) {
  return (x(1) + 1e-5 * sq(x(1)-x(0)));
}
VectorXd g_TP3(const VectorXd& x) {
  VectorXd out(1);
  out(0) = 0 - x(1);
  return out;
}
double f_TP6(const VectorXd& x) {
  return sq(1-x(0));
}
VectorXd g_TP6(const VectorXd& x) {
  VectorXd out(1);
  out(0) = 10*(x(1)-sq(x(0)));
  return out;
}
double f_TP7(const VectorXd& x) {
  return log(1+sq(x(0))) - x(1);
}
VectorXd g_TP7(const VectorXd& x) {
  VectorXd out(1);
  out(0) = sq(1+sq(x(0))) + sq(x(1)) - 4;
  return out;
}

TEST(SQP, TP1) {
  testProblem(ScalarOfVector::construct(&f_TP1), VectorOfVector::construct(&g_TP1), INEQ, list_of(-2)(1), list_of(1)(1));
}
TEST(SQP, TP3) {
  testProblem(ScalarOfVector::construct(&f_TP3), VectorOfVector::construct(&g_TP3), INEQ, list_of(10)(1), list_of(0)(0));
}
TEST(SQP, TP6) {
  testProblem(ScalarOfVector::construct(&f_TP6), VectorOfVector::construct(&g_TP6), EQ, list_of(10)(1), list_of(1)(1));
}
TEST(SQP, TP7) {
  testProblem(ScalarOfVector::construct(&f_TP7), VectorOfVector::construct(&g_TP7), EQ, list_of(2)(2), list_of(0.)(sqrtf(3.)));
}
