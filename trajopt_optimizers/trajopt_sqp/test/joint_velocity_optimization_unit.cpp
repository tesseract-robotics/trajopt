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
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>

const bool DEBUG = false;

class VelocityConstraintOptimization : public testing::TestWithParam<const char*>
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

    // 2) Add Variables
    std::vector<trajopt::JointPosition::ConstPtr> vars;
    std::vector<std::string> joint_names(7, "name");
    {
      auto pos = Eigen::VectorXd::Zero(7);
      auto var = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_0");
      auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
      var->SetBounds(bounds);
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }
    for (int ind = 1; ind < 3; ind++)
    {
      auto pos = Eigen::VectorXd::Ones(7) * 10;
      auto var = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
      auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
      var->SetBounds(bounds);
      vars.push_back(var);
      nlp_.AddVariableSet(var);
    }

    // 3) Add constraints
    Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(7);
    std::vector<trajopt::JointPosition::ConstPtr> start;
    start.push_back(vars.front());
    auto start_constraint = std::make_shared<trajopt::JointPosConstraint>(start_pos, start, "StartPosition");
    nlp_.AddConstraintSet(start_constraint);

    Eigen::VectorXd end_pos = Eigen::VectorXd::Ones(7) * 10;
    std::vector<trajopt::JointPosition::ConstPtr> end;
    end.push_back(vars.back());
    auto end_constraint = std::make_shared<trajopt::JointPosConstraint>(end_pos, end, "EndPosition");
    nlp_.AddConstraintSet(end_constraint);

    // 4) Add costs
    Eigen::VectorXd vel_target = Eigen::VectorXd::Zero(7);
    auto vel_constraint = std::make_shared<trajopt::JointVelConstraint>(vel_target, vars, "jv");

    // Must link the variables to the constraint since that happens in AddConstraintSet
    vel_constraint->LinkWithVariables(nlp_.GetOptVariables());
    Eigen::VectorXd weights = Eigen::VectorXd::Constant(vel_constraint->GetRows(), 0.01);
    auto vel_cost = std::make_shared<trajopt::SquaredCost>(vel_constraint, weights);
    nlp_.AddCostSet(vel_cost);

    if (DEBUG)
    {
      nlp_.PrintCurrent();
      std::cout << "Constraint Jacobian: \n" << nlp_.GetJacobianOfConstraints().toDense() << std::endl;
      std::cout << "Cost Jacobian: \n" << nlp_.GetJacobianOfCosts() << std::endl;
    }
  }
};

/** @brief Joint position constraints with a squared velocity cost in between. Optimized using ipopt */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_ipopt)  // NOLINT
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
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-5);
  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_ipopt.PrintCurrent();
  }
}

/** @brief Joint position constraints with a squared velocity cost in between. Optimized using trajopt_sqp */
TEST_F(VelocityConstraintOptimization, velocity_constraint_optimization_trajopt_sqp)  // NOLINT
{
  ifopt::Problem nlp_trajopt_sqp(nlp_);
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(DEBUG);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);

  // 6) solve
  solver.verbose = DEBUG;
  solver.Solve(nlp_trajopt_sqp);
  Eigen::VectorXd x = nlp_trajopt_sqp.GetOptVariables()->GetValues();

  for (Eigen::Index i = 0; i < 7; i++)
    EXPECT_NEAR(x[i], 0.0, 1e-3);
  for (Eigen::Index i = 7; i < 14; i++)
    EXPECT_NEAR(x[i], 5.0, 1e-1);
  for (Eigen::Index i = 14; i < 21; i++)
    EXPECT_NEAR(x[i], 10.0, 1e-3);
  if (DEBUG)
  {
    std::cout << x.transpose() << std::endl;
    nlp_trajopt_sqp.PrintCurrent();
  }
}
