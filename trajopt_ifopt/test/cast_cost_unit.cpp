#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract/tesseract.h>

#include <tesseract_environment/core/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
#include <ifopt/problem.h>
#include <ifopt/ipopt_solver.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_test_utils.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_ifopt/constraints/collision_constraint.h>

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

class CastTest : public testing::TestWithParam<const char*>
{
public:
  Tesseract::Ptr tesseract = std::make_shared<Tesseract>(); /**< Tesseract */

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/boxbot.srdf");
    auto tmp = TRAJOPT_DIR;
    std::cout << tmp;

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(tesseract->init(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

TEST_F(CastTest, boxes)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("CastTest, boxes");

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  tesseract->getEnvironment()->setState(ipos);

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = tesseract->getEnvironment()->getStateSolver();
  DiscreteContactManager::Ptr manager = tesseract->getEnvironment()->getDiscreteContactManager();
  auto forward_kinematics = tesseract->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  AdjacencyMap::Ptr adjacency_map =
      std::make_shared<AdjacencyMap>(tesseract->getEnvironment()->getSceneGraph(),
                                     forward_kinematics->getActiveLinkNames(),
                                     tesseract->getEnvironment()->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setContactDistanceThreshold(0);

  collisions.clear();

  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
  {
    Eigen::VectorXd pos(2);
    pos << -1.9, 0;
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 0, 0;
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_1");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }
  {
    Eigen::VectorXd pos(2);
    pos << 1.9, 3.8;
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_2");
    vars.push_back(var);
    nlp.AddVariableSet(var);
  }

  // Step 3: Setup collision
  auto env = tesseract->getEnvironment();
  auto kin = tesseract->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  double margin_coeff = 20;
  double margin = 0.3;
  trajopt::SafetyMarginData::ConstPtr margin_data = std::make_shared<trajopt::SafetyMarginData>(margin, margin_coeff);
  double safety_margin_buffer = 0.05;
  sco::VarVector var_vector;  // unused

  trajopt::SingleTimestepCollisionEvaluator::Ptr collision_evaluator =
      std::make_shared<trajopt::SingleTimestepCollisionEvaluator>(
          kin,
          env,
          adj_map,
          Eigen::Isometry3d::Identity(),
          margin_data,
          tesseract_collision::ContactTestType::CLOSEST,
          var_vector,
          trajopt::CollisionExpressionEvaluatorType::SINGLE_TIME_STEP,
          safety_margin_buffer);

  // 4) Add constraints
  for (const auto& var : vars)
  {
    auto cnt = std::make_shared<trajopt::CollisionConstraintIfopt>(collision_evaluator, var);
    nlp.AddConstraintSet(cnt);
  }

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  // 5) choose solver and options
  ifopt::IpoptSolver ipopt;
  ipopt.SetOption("derivative_test", "first-order");
  ipopt.SetOption("linear_solver", "mumps");
  //   ipopt.SetOption("jacobian_approximation", "finite-difference-values");
  ipopt.SetOption("jacobian_approximation", "exact");
  ipopt.SetOption("print_level", 5);

  // 6) solve
  ipopt.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
  std::cout << x.transpose() << std::endl;

  TrajArray inputs(3, 2);
  inputs << -1.9, 0, 0, 0.0, 1.9, 3.8;
  Eigen::Map<TrajArray> results(x.data(), 3, 2);

  bool found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), inputs);

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
  found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results);

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
