#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <iostream>

#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>

#include <tesseract/tesseract.h>
#include <tesseract_scene_graph/resource_locator.h>
#include <boost/filesystem/path.hpp>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt_ifopt/callbacks/joint_state_plotter.h>
#include <trajopt_ifopt/callbacks/cartesian_error_plotter.h>

inline std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://trajopt") == 0)
  {
    mod_url.erase(0, strlen("package://trajopt"));
    size_t pos = mod_url.find('/');
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = std::string(TRAJOPT_DIR);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url;
  }

  return mod_url;
}

int main(int /*argc*/, char** /*argv*/)
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);

  // 1)  Load Robot
  boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
  boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");
  tesseract_scene_graph::ResourceLocator::Ptr locator =
      std::make_shared<tesseract_scene_graph::SimpleResourceLocator>(locateResource);
  auto tesseract = std::make_shared<tesseract::Tesseract>();
  tesseract->init(urdf_file, srdf_file, locator);

  // Extract necessary kinematic information
  auto forward_kinematics = tesseract->getFwdKinematicsManager()->getFwdKinematicSolver("right_arm");
  tesseract_environment::AdjacencyMap::Ptr adjacency_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      tesseract->getEnvironment()->getSceneGraph(),
      forward_kinematics->getActiveLinkNames(),
      tesseract->getEnvironment()->getCurrentState()->link_transforms);
  auto kinematic_info = std::make_shared<trajopt::CartPosKinematicInfo>(
      forward_kinematics, adjacency_map, Eigen::Isometry3d::Identity(), forward_kinematics->getTipLinkName());

  // 2) Create the problem
  ifopt::Problem nlp;

  // Get target position
  Eigen::VectorXd start_pos(forward_kinematics->numJoints());
  std::cout << "Joint Limits:\n" << forward_kinematics->getLimits().transpose() << std::endl;
  start_pos << 0.0, 0, 0, -1.0, 0, -1, -0.00;

  auto target_pose = Eigen::Isometry3d::Identity();
  auto joint_target = start_pos;
  forward_kinematics->calcFwdKin(target_pose, joint_target);
  auto world_to_base =
      tesseract->getEnvironment()->getCurrentState()->link_transforms.at(forward_kinematics->getBaseLinkName());
  target_pose = world_to_base * target_pose;
  std::cout << "target_pose:\n" << target_pose.matrix() << std::endl;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::Ptr> vars;
  for (int ind = 0; ind < 1; ind++)
  {
    auto zero = Eigen::VectorXd::Zero(7);
    auto var = std::make_shared<trajopt::JointPosition>(
        zero, forward_kinematics->getJointNames(), "Joint_Position_" + std::to_string(ind));
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // 4) Add constraints
  forward_kinematics->calcFwdKin(target_pose, joint_target);
  for (const auto& var : vars)
  {
    auto cnt = std::make_shared<trajopt::CartPosConstraint>(target_pose, kinematic_info, var);
    nlp.AddConstraintSet(cnt);
    //    cnt->LinkWithVariables(nlp.GetOptVariables());
    //    auto cost = std::make_shared<trajopt::SquaredCost>(cnt);
    //    nlp.AddCostSet(cost);
  }

  nlp.PrintCurrent();
  std::cout << "Constraint Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  // 5) Choose solver and options
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
  qp_solver->solver_.settings()->setMaxIteraction(8192);
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  //  ifopt::IpoptSolver solver;
  //  solver.SetOption("derivative_test", "first-order");
  //  solver.SetOption("linear_solver", "mumps");
  //  //  ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  //  solver.SetOption("jacobian_approximation", "exact");
  //  solver.SetOption("print_level", 5);

  // 6) solve
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << "Optimized Variables:" << x.transpose() << std::endl;

  nlp.PrintCurrent();
}
