#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

const bool DEBUG = false;

class JointPositionOptimization : public testing::TestWithParam<const char*>
{
public:
  // 1) Create the problem
  ifopt::Problem nlp_;

  void SetUp() override
  {
    if (DEBUG)
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    else
      console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // 1) Create the problem
    ifopt::Problem nlp;

    // 2) Add Variables
    std::vector<trajopt::JointPosition::ConstPtr> vars;
    for (int ind = 0; ind < 2; ind++)
    {
      auto pos = Eigen::VectorXd::Zero(7);
      std::vector<std::string> joint_names(7, "name");
      auto var = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }

    // 3) Add constraints
    Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
    std::vector<trajopt::JointPosition::ConstPtr> start;
    start.push_back(vars.front());

    auto start_constraint = std::make_shared<trajopt::JointPosConstraint>(start_pos, start, "StartPosition");
    nlp_.AddConstraintSet(start_constraint);

    Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7);
    std::vector<trajopt::JointPosition::ConstPtr> end;
    end.push_back(vars.back());
    auto end_constraint = std::make_shared<trajopt::JointPosConstraint>(end_pos, end, "EndPosition");
    nlp_.AddConstraintSet(end_constraint);

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Jacobian: \n" << nlp_.GetJacobianOfConstraints() << std::endl;
    }
  }
};

/**
 * @brief Applies a joint position constraint and solves the problem with IPOPT
 */
TEST_F(JointPositionOptimization, joint_position_optimization_ipopt)  // NOLINT
{
  ifopt::Problem nlp_ipopt(nlp_);
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  //  ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  // solve
  solver.Solve(nlp_ipopt);
  Eigen::VectorXd x = nlp_ipopt.GetOptVariables()->GetValues();
  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 1.0, 1e-5);
  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_ipopt.PrintCurrent();
  }
}
/**
 * @brief Applies a joint position constraint and solves the problem with trajopt_sqp
 */
TEST_F(JointPositionOptimization, joint_position_optimization_trajopt_sqp)  // NOLINT
{
  ifopt::Problem nlp_trajopt_sqp(nlp_);
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // solve
  solver.verbose = DEBUG;
  solver.Solve(nlp_trajopt_sqp);
  Eigen::VectorXd x = nlp_trajopt_sqp.GetOptVariables()->GetValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-5);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 1.0, 1e-5);

  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_trajopt_sqp.PrintCurrent();
  }
}

////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
