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

int main(int /*argc*/, char** /*argv*/)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // 1) Create the problem
  ifopt::Problem nlp;

  // 2) Add Variables
  std::vector<trajopt::JointPosition::Ptr> vars;
  for (int ind = 0; ind < 9; ind++)
  {
    auto pos = Eigen::VectorXd::Zero(7);
    std::vector<std::string> joint_names(7, "name");
    auto var = std::make_shared<trajopt::JointPosition>(pos, joint_names, "Joint_Position_" + std::to_string(ind));
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
  end_pos << 1, 1, 1, 1, 1, 1, 1;
  std::vector<trajopt::JointPosition::Ptr> end;
  end.push_back(vars.back());
  auto end_constraint = std::make_shared<trajopt::JointPosConstraint>(end_pos, end, "EndPosition");
  nlp.AddConstraintSet(end_constraint);

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  // 5) choose solver and options
  ifopt::IpoptSolver solver;
  solver.SetOption("derivative_test", "first-order");
  solver.SetOption("linear_solver", "mumps");
  //  ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  solver.SetOption("jacobian_approximation", "exact");
  solver.SetOption("print_level", 5);

  // 6) solve
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << std::endl;

  nlp.PrintCurrent();
}
