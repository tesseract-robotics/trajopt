#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
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

int main(int /*argc*/, char** /*argv*/)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
  // 1) Create the problem
  ifopt::Problem nlp;

  // 2) Add Variables
  std::vector<trajopt::JointPosition::Ptr> vars;
  {
    auto pos = Eigen::VectorXd::Zero(7);
    auto var = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_0");
    auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
    var->SetBounds(bounds);
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  for (int ind = 1; ind < 3; ind++)
  {
    auto pos = Eigen::VectorXd::Ones(7) * 10;
    auto var = std::make_shared<trajopt::JointPosition>(pos, "Joint_Position_" + std::to_string(ind));
    auto bounds = std::vector<ifopt::Bounds>(7, ifopt::NoBound);
    var->SetBounds(bounds);
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // 3) Add constraints
  Eigen::VectorXd start_pos(7);
  start_pos << 0, 0, 0, 0, 0, 0, 0;
  std::vector<trajopt::JointPosition::Ptr> start;
  start.push_back(vars.front());
  auto start_constraint = std::make_shared<trajopt::JointPosConstraint>(start_pos, start, "StartPosition");
  nlp.AddConstraintSet(start_constraint);

  Eigen::VectorXd end_pos(7);
  end_pos << 10, 10, 10, 10, 10, 10, 10;
  std::vector<trajopt::JointPosition::Ptr> end;
  end.push_back(vars.back());
  auto end_constraint = std::make_shared<trajopt::JointPosConstraint>(end_pos, end, "EndPosition");
  nlp.AddConstraintSet(end_constraint);

  // 4) Add costs
  Eigen::VectorXd vel_target(7);
  vel_target << 0, 0, 0, 0, 0, 0, 0;
  auto vel_constraint = std::make_shared<trajopt::JointVelConstraint>(vel_target, vars, "JointVelocity");

  // Must link the variables to the constraint since that happens in AddConstraintSet
  vel_constraint->LinkWithVariables(nlp.GetOptVariables());
  auto vel_cost = std::make_shared<trajopt::SquaredCost>(vel_constraint);
  nlp.AddCostSet(vel_cost);

  nlp.PrintCurrent();
  std::cout << "Constraint Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;
  std::cout << "Cost Jacobian: \n" << nlp.GetJacobianOfCosts() << std::endl;

  // 5) Set up the solver
  auto qp_solver = std::make_shared<trajopt::OSQPEigenSolver>();
  trajopt::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(false);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);

  // 6) solve
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << "Results: " << x.transpose() << std::endl;

  nlp.PrintCurrent();
}
