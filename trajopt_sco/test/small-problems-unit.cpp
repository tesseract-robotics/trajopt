#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Dense>
#include <boost/format.hpp>
#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <sstream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/modeling_utils.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_sco/sco_common.hpp>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace util;
using namespace std;
using namespace sco;
using namespace Eigen;

class SQP : public testing::TestWithParam<ModelType>
{
protected:
  SQP() = default;
};

void setupProblem(OptProb::Ptr& probptr, size_t nvars, ModelType convex_solver)
{
  probptr = std::make_shared<OptProb>(convex_solver);
  vector<string> var_names;
  for (size_t i = 0; i < nvars; ++i)
  {
    var_names.push_back((boost::format("x_%i") % i).str());
  }
  probptr->createVariables(var_names);
}

void expectAllNear(const DblVec& x, const DblVec& y, double abstol)
{
  EXPECT_EQ(x.size(), y.size());
  stringstream ss;
  LOG_INFO("checking %s ?= %s", CSTR(x), CSTR(y));
  for (size_t i = 0; i < x.size(); ++i)
    EXPECT_NEAR(x[i], y[i], abstol);
}

double f_QuadraticSeparable(const VectorXd& x) { return x(0) * x(0) + sq(x(1) - 1) + sq(x(2) - 2); }
TEST_P(SQP, QuadraticSeparable)  // NOLINT
{
  // if the problem is exactly a QP, it should be solved in one iteration
  OptProb::Ptr prob;
  setupProblem(prob, 3, GetParam());
  prob->addCost(std::make_shared<CostFromFunc>(ScalarOfVector::construct(&f_QuadraticSeparable), prob->getVars(), "f"));
  BasicTrustRegionSQP solver(prob);
  BasicTrustRegionSQPParameters& params = solver.getParameters();
  params.trust_box_size = 100;
  DblVec x = { 3, 4, 5 };
  solver.initialize(x);
  OptStatus status = solver.optimize();
  ASSERT_EQ(status, OPT_CONVERGED);
  expectAllNear(solver.x(), { 0, 1, 2 }, 1e-3);
  // todo: checks on number of iterations and function evaluates
}
double f_QuadraticNonseparable(const VectorXd& x) { return sq(x(0) - x(1) + 3 * x(2)) + sq(x(0) - 1) + sq(x(2) - 2); }
TEST_P(SQP, QuadraticNonseparable)  // NOLINT
{
  OptProb::Ptr prob;
  setupProblem(prob, 3, GetParam());
  prob->addCost(
      Cost::Ptr(new CostFromFunc(ScalarOfVector::construct(&f_QuadraticNonseparable), prob->getVars(), "f", true)));
  BasicTrustRegionSQP solver(prob);
  BasicTrustRegionSQPParameters& params = solver.getParameters();
  params.trust_box_size = 100;
  params.min_trust_box_size = 1e-5;
  params.min_approx_improve = 1e-6;
  DblVec x = { 3, 4, 5 };
  solver.initialize(x);
  OptStatus status = solver.optimize();
  ASSERT_EQ(status, OPT_CONVERGED);
  expectAllNear(solver.x(), { 1, 7, 2 }, .01);
  // todo: checks on number of iterations and function evaluates
}

void testProblem(ScalarOfVector::Ptr f,
                 VectorOfVector::Ptr g,
                 ConstraintType cnt_type,
                 const DblVec& init,
                 const DblVec& sol,
                 ModelType convex_solver)
{
  OptProb::Ptr prob;
  size_t n = init.size();
  assert(sol.size() == n);
  setupProblem(prob, n, convex_solver);
  prob->addCost(std::make_shared<CostFromFunc>(std::move(f), prob->getVars(), "f", true));
  prob->addConstraint(
      std::make_shared<ConstraintFromErrFunc>(std::move(g), prob->getVars(), VectorXd(), cnt_type, "g"));
  BasicTrustRegionSQP solver(prob);
  BasicTrustRegionSQPParameters& params = solver.getParameters();
  params.max_iter = 1000;
  params.min_trust_box_size = 1e-5;
  params.min_approx_improve = 1e-10;
  params.initial_merit_error_coeff = 1;

  solver.initialize(init);
  OptStatus status = solver.optimize();
  EXPECT_EQ(status, OPT_CONVERGED);
  expectAllNear(solver.x(), sol, .01);
}
// http://www.ai7.uni-bayreuth.de/test_problem_coll.pdf

double f_TP1(const VectorXd& x) { return 1 * sq(x(1) - sq(x(0))) + sq(1 - x(0)); }
VectorXd g_TP1(const VectorXd& x)
{
  VectorXd out(1);
  out(0) = -1.5 - x(1);
  return out;
}
double f_TP2(const VectorXd& x) { return 100 * sq(x(1) - sq(x(0))) + sq(1 - x(0)); }
VectorXd g_TP2(const VectorXd& x)
{
  VectorXd out(1);
  out(0) = -1.5 - x(1);
  return out;
}
double f_TP3(const VectorXd& x) { return (x(1) + 1e-5 * sq(x(1) - x(0))); }
VectorXd g_TP3(const VectorXd& x)
{
  VectorXd out(1);
  out(0) = 0 - x(1);
  return out;
}
double f_TP6(const VectorXd& x) { return sq(1 - x(0)); }
VectorXd g_TP6(const VectorXd& x)
{
  VectorXd out(1);
  out(0) = 10 * (x(1) - sq(x(0)));
  return out;
}
double f_TP7(const VectorXd& x) { return log(1 + sq(x(0))) - x(1); }
VectorXd g_TP7(const VectorXd& x)
{
  VectorXd out(1);
  out(0) = sq(1 + sq(x(0))) + sq(x(1)) - 4;
  return out;
}

TEST_P(SQP, TP1)  // NOLINT
{
  testProblem(
      ScalarOfVector::construct(&f_TP1), VectorOfVector::construct(&g_TP1), INEQ, { -2, 1 }, { 1, 1 }, GetParam());
}
TEST_P(SQP, TP3)  // NOLINT
{
  testProblem(
      ScalarOfVector::construct(&f_TP3), VectorOfVector::construct(&g_TP3), INEQ, { 10, 1 }, { 0, 0 }, GetParam());
}
TEST_P(SQP, TP6)  // NOLINT
{
  testProblem(
      ScalarOfVector::construct(&f_TP6), VectorOfVector::construct(&g_TP6), EQ, { 10, 1 }, { 1, 1 }, GetParam());
}
TEST_P(SQP, TP7)  // NOLINT
{
  testProblem(ScalarOfVector::construct(&f_TP7),
              VectorOfVector::construct(&g_TP7),
              EQ,
              { 2, 2 },
              { 0., sqrtf(3.) },
              GetParam());
}

auto getAvailableSolvers = []() {
  std::vector<ModelType> solvers = availableSolvers();
  auto it = std::find(solvers.begin(), solvers.end(), ModelType::OSQP);
  if (it != solvers.end())
  {
    solvers.erase(it);
  };
  return solvers;
};

INSTANTIATE_TEST_CASE_P(AllSolvers, SQP, testing::ValuesIn(getAvailableSolvers()));
