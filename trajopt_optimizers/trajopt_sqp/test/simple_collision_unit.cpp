<<<<<<< HEAD
/**
 * @file simple_collision_unit.cpp
 * @brief A simple collision unit test which matches exactly a unit test in trajopt
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
=======
>>>>>>> fixup
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>
#include <tesseract_environment/core/utils.h>
#include <tesseract_visualization/visualization.h>
#include <tesseract_scene_graph/utils.h>
#include <ifopt/problem.h>
#include <ifopt/constraint_set.h>
TRAJOPT_IGNORE_WARNINGS_POP

<<<<<<< HEAD
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>
#include "test_suite_utils.hpp"

using namespace trajopt_ifopt;
=======
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>
#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/constraints/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>


using namespace trajopt;
using namespace std;
using namespace util;
>>>>>>> fixup
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;

<<<<<<< HEAD
class SimpleCollisionConstraintIfopt : public ifopt::ConstraintSet
{
public:
  SimpleCollisionConstraintIfopt(DiscreteCollisionEvaluator::Ptr collision_evaluator,
                                 JointPosition::ConstPtr position_var,
                                 const std::string& name = "SimpleCollisionConstraint")
    : ifopt::ConstraintSet(3, name)
    , position_var_(std::move(position_var))
    , collision_evaluator_(std::move(collision_evaluator))
  {
    // Set n_dof_ for convenience
    n_dof_ = position_var_->GetRows();
    assert(n_dof_ > 0);

    bounds_ = std::vector<ifopt::Bounds>(3, ifopt::BoundSmallerZero);
  }

  Eigen::VectorXd GetValues() const final
  {
    // Get current joint values
    Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

    return CalcValues(joint_vals);
  }

  // Set the limits on the constraint values
  std::vector<ifopt::Bounds> GetBounds() const final { return bounds_; }

  void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const final
  {
    // Only modify the jacobian if this constraint uses var_set
    if (var_set == position_var_->GetName())
    {
      // Get current joint values
      VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

      CalcJacobianBlock(joint_vals, jac_block);
    }
  }

  Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
  {
    Eigen::VectorXd err = Eigen::VectorXd::Zero(3);

    // Check the collisions
    CollisionCacheData::ConstPtr cdata = collision_evaluator_->CalcCollisions(joint_vals);

    if (cdata->contact_results_map.empty())
      return err;

    Eigen::Index i{ 0 };
    for (const auto& pair : cdata->contact_results_map)
    {
      for (const auto& dist_result : pair.second)
      {
        double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
            dist_result.link_names[0], dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
            dist_result.link_names[0], dist_result.link_names[1]);
        err[i++] += std::max<double>(((dist - dist_result.distance) * coeff), 0.);
      }
    }

    return err;
  }

  void SetBounds(const std::vector<ifopt::Bounds>& bounds)
  {
    assert(bounds.size() == 3);
    bounds_ = bounds;
  }

  void CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals, Jacobian& jac_block) const
  {
    // Reserve enough room in the sparse matrix
    jac_block.reserve(n_dof_ * 3);

    // Calculate collisions
    CollisionCacheData::ConstPtr cdata = collision_evaluator_->CalcCollisions(joint_vals);

    // Get gradients for all contacts
    /** @todo Use the cdata gradient results */
    std::vector<trajopt_ifopt::GradientResults> grad_results;
    for (const auto& pair : cdata->contact_results_map)
    {
      for (const auto& dist_result : pair.second)
      {
        trajopt_ifopt::GradientResults result = collision_evaluator_->GetGradient(joint_vals, dist_result);
        grad_results.push_back(result);
      }
    }

    for (std::size_t i = 0; i < grad_results.size(); ++i)
    {
      if (grad_results[i].gradients[0].has_gradient)
      {
        // This does work but could be faster
        for (int j = 0; j < n_dof_; j++)
        {
          // Collision is 1 x n_dof
          jac_block.coeffRef(static_cast<Eigen::Index>(i), j) = -1.0 * grad_results[i].gradients[0].gradient[j];
        }
      }
      else if (grad_results[i].gradients[1].has_gradient)
      {
        // This does work but could be faster
        for (int j = 0; j < n_dof_; j++)
        {
          // Collision is 1 x n_dof
          jac_block.coeffRef(static_cast<Eigen::Index>(i), j) = -1.0 * grad_results[i].gradients[1].gradient[j];
        }
      }
    }
  }

  DiscreteCollisionEvaluator::Ptr GetCollisionEvaluator() const { return collision_evaluator_; }

=======
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

class SimpleCollisionConstraintIfopt : public ifopt::ConstraintSet
{
public:

SimpleCollisionConstraintIfopt(DiscreteCollisionEvaluator::Ptr collision_evaluator,
                               JointPosition::ConstPtr position_var,
                               const std::string& name = "SimpleCollisionConstraint")
  : ifopt::ConstraintSet(3, name)
  , position_var_(std::move(position_var))
  , collision_evaluator_(std::move(collision_evaluator))
{
  // Set n_dof_ for convenience
  n_dof_ = position_var_->GetRows();
  assert(n_dof_ > 0);

  bounds_ = std::vector<ifopt::Bounds>(3, ifopt::BoundSmallerZero);
}

Eigen::VectorXd GetValues() const
{
  // Get current joint values
  Eigen::VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

  return CalcValues(joint_vals);
}

// Set the limits on the constraint values
std::vector<ifopt::Bounds> GetBounds() const { return bounds_; }

void FillJacobianBlock(std::string var_set, Jacobian& jac_block) const
{
  // Only modify the jacobian if this constraint uses var_set
  if (var_set == position_var_->GetName())
  {
    // Get current joint values
    VectorXd joint_vals = this->GetVariables()->GetComponent(position_var_->GetName())->GetValues();

    CalcJacobianBlock(joint_vals, jac_block);
  }
}

Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
{
  Eigen::VectorXd err = Eigen::VectorXd::Zero(3);

  // Check the collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals, dist_results);

  if (dist_results.empty())
    return err;

  for (std::size_t i = 0; i < dist_results.size(); ++i)
  {
    tesseract_collision::ContactResult& dist_result = dist_results[i];
    double dist = collision_evaluator_->GetCollisionConfig().collision_margin_data.getPairCollisionMargin(
        dist_result.link_names[0], dist_result.link_names[1]);
    double coeff = collision_evaluator_->GetCollisionConfig().collision_coeff_data.getPairCollisionCoeff(
        dist_result.link_names[0], dist_result.link_names[1]);
    err[static_cast<Eigen::Index>(i)] += std::max<double>(((dist - dist_result.distance) * coeff), 0.);
  }

  return err;
}

void SetBounds(const std::vector<ifopt::Bounds>& bounds)
{
  assert(bounds.size() == 3);
  bounds_ = bounds;
}

void CalcJacobianBlock(const Eigen::Ref<const Eigen::VectorXd>& joint_vals,
                                                         Jacobian& jac_block) const
{
  // Reserve enough room in the sparse matrix
  jac_block.reserve(n_dof_ * 3);

  // Calculate collisions
  tesseract_collision::ContactResultVector dist_results;
  collision_evaluator_->CalcCollisions(joint_vals, dist_results);

  // Get gradients for all contacts
  std::vector<trajopt::GradientResults> grad_results;
  grad_results.reserve(dist_results.size());
  for (tesseract_collision::ContactResult& dist_result : dist_results)
  {
    trajopt::GradientResults result = collision_evaluator_->GetGradient(joint_vals, dist_result);
    grad_results.push_back(result);
  }

  for (std::size_t i = 0; i < grad_results.size(); ++i)
  {
    if (grad_results[i].gradients[0].has_gradient)
    {
      // This does work but could be faster
      for (int j = 0; j < n_dof_; j++)
      {
        // Collision is 1 x n_dof
        jac_block.coeffRef(static_cast<Eigen::Index>(i), j) = -1.0 * grad_results[i].data[2] * grad_results[i].gradients[0].gradient[j];
      }
    }
    else if (grad_results[i].gradients[1].has_gradient)
    {
      // This does work but could be faster
      for (int j = 0; j < n_dof_; j++)
      {
        // Collision is 1 x n_dof
        jac_block.coeffRef(static_cast<Eigen::Index>(i), j) = -1.0 * grad_results[i].data[2] * grad_results[i].gradients[1].gradient[j];
      }
    }
  }
}

DiscreteCollisionEvaluator::Ptr GetCollisionEvaluator() const
{
  return collision_evaluator_;
}
>>>>>>> fixup
private:
  /** @brief The number of joints in a single JointPosition */
  long n_dof_;

  /** @brief Bounds on the constraint value. Default: std::vector<Bounds>(1, ifopt::BoundSmallerZero) */
  std::vector<ifopt::Bounds> bounds_;

  /**
   * @brief Pointers to the vars used by this constraint.
   * Do not access them directly. Instead use this->GetVariables()->GetComponent(position_var->GetName())->GetValues()
   */
  JointPosition::ConstPtr position_var_;

  DiscreteCollisionEvaluator::Ptr collision_evaluator_;
};

<<<<<<< HEAD
class SimpleCollisionTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                            /**< Plotter */
=======
class CastTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Plotter */
>>>>>>> fixup

  void SetUp() override
  {
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.srdf");
<<<<<<< HEAD
    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_file, srdf_file, locator));
  }
};

void runSimpleCollisionTest(const trajopt_sqp::QPProblem::Ptr& qp_problem, const Environment::Ptr& env)
{
=======
    auto tmp = TRAJOPT_DIR;
    std::cout << tmp;

    ResourceLocator::Ptr locator = std::make_shared<SimpleResourceLocator>(locateResource);
    EXPECT_TRUE(env->init<OFKTStateSolver>(urdf_file, srdf_file, locator));

    gLogLevel = util::LevelError;
  }
};

TEST_F(CastTest, boxes)  // NOLINT
{
  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres");

>>>>>>> fixup
  std::unordered_map<std::string, double> ipos;
  ipos["spherebot_x_joint"] = -0.75;
  ipos["spherebot_y_joint"] = 0.75;
  env->setState(ipos);

  //  plotter_->plotScene();

  std::vector<ContactResultMap> collisions;
  tesseract_environment::StateSolver::Ptr state_solver = env->getStateSolver();
<<<<<<< HEAD
  DiscreteContactManager::Ptr manager = env->getDiscreteContactManager();
=======
  ContinuousContactManager::Ptr manager = env->getContinuousContactManager();
>>>>>>> fixup
  auto forward_kinematics = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  AdjacencyMap::Ptr adjacency_map = std::make_shared<AdjacencyMap>(
      env->getSceneGraph(), forward_kinematics->getActiveLinkNames(), env->getCurrentState()->link_transforms);

  manager->setActiveCollisionObjects(adjacency_map->getActiveLinkNames());
  manager->setDefaultCollisionMarginData(0);

  collisions.clear();

<<<<<<< HEAD
  // 3) Add Variables
  std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
=======
  // 2) Create the problem
  ifopt::Problem nlp;

  // 3) Add Variables
  std::vector<trajopt::JointPosition::ConstPtr> vars;
>>>>>>> fixup
  std::vector<Eigen::VectorXd> positions;
  {
    Eigen::VectorXd pos(2);
    pos << -0.75, 0.75;
    positions.push_back(pos);
<<<<<<< HEAD
    auto var =
        std::make_shared<trajopt_ifopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    qp_problem->addVariableSet(var);
=======
    auto var = std::make_shared<trajopt::JointPosition>(pos, forward_kinematics->getJointNames(), "Joint_Position_0");
    vars.push_back(var);
    nlp.AddVariableSet(var);
>>>>>>> fixup
  }

  // Step 3: Setup collision
  auto kin = env->getManipulatorManager()->getFwdKinematicSolver("manipulator");
  auto adj_map = std::make_shared<tesseract_environment::AdjacencyMap>(
      env->getSceneGraph(), kin->getActiveLinkNames(), env->getCurrentState()->link_transforms);

<<<<<<< HEAD
  auto trajopt_collision_cnt_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(0.2, 1);
  trajopt_collision_cnt_config->collision_margin_buffer = 0.05;

  auto collision_cnt_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  trajopt_ifopt::DiscreteCollisionEvaluator::Ptr collision_cnt_evaluator =
      std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
          collision_cnt_cache, kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_cnt_config);
  auto collision_cnt = std::make_shared<SimpleCollisionConstraintIfopt>(collision_cnt_evaluator, vars[0]);
  qp_problem->addConstraintSet(collision_cnt);

  auto trajopt_collision_cost_config = std::make_shared<trajopt_ifopt::TrajOptCollisionConfig>(0.3, 1);
  trajopt_collision_cost_config->collision_margin_buffer = 0.05;

  auto collision_cost_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
  trajopt_ifopt::DiscreteCollisionEvaluator::Ptr collision_cost_evaluator =
      std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
          collision_cost_cache, kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_cost_config);
  auto collision_cost = std::make_shared<SimpleCollisionConstraintIfopt>(collision_cost_evaluator, vars[0]);
  qp_problem->addCostSet(collision_cost, trajopt_sqp::CostPenaltyType::HINGE);

  Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(2, 1);
  auto jp_cost = std::make_shared<trajopt_ifopt::JointPosConstraint>(Eigen::Vector2d(0, 0), vars, coeffs);
  qp_problem->addCostSet(jp_cost, trajopt_sqp::CostPenaltyType::SQUARED);

  qp_problem->setup();
  qp_problem->print();

  auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return collision_cnt->CalcValues(x); };
  trajopt_ifopt::SparseMatrix num_jac_block = trajopt_ifopt::calcForwardNumJac(error_calculator, positions[0], 1e-4);
=======
  double margin_coeff = 10;
  double margin = 0.2;
  trajopt::TrajOptCollisionConfig trajopt_collision_config(margin, margin_coeff);
  trajopt_collision_config.collision_margin_buffer = 0.05;


  trajopt::DiscreteCollisionEvaluator::Ptr collision_evaluator = std::make_shared<trajopt::SingleTimestepCollisionEvaluator>(
        kin, env, adj_map, Eigen::Isometry3d::Identity(), trajopt_collision_config);

  auto cnt = std::make_shared<SimpleCollisionConstraintIfopt>(collision_evaluator, vars[0]);
  nlp.AddConstraintSet(cnt);

  nlp.PrintCurrent();
  std::cout << "Jacobian: \n" << nlp.GetJacobianOfConstraints() << std::endl;

  auto error_calculator = [&](const Eigen::Ref<const Eigen::VectorXd>& x) { return cnt->CalcValues(x); };
  trajopt::Jacobian num_jac_block = trajopt::calcForwardNumJac(error_calculator, positions[0], 1e-4);
>>>>>>> fixup
  std::cout << "Numerical Jacobian: \n" << num_jac_block << std::endl;

  // 5) choose solver and options
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_.settings()->setVerbosity(true);
  qp_solver->solver_.settings()->setWarmStart(true);
  qp_solver->solver_.settings()->setPolish(true);
  qp_solver->solver_.settings()->setAdaptiveRho(false);
<<<<<<< HEAD
  qp_solver->solver_.settings()->setMaxIteration(8192);
=======
  qp_solver->solver_.settings()->setMaxIteraction(8192);
>>>>>>> fixup
  qp_solver->solver_.settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_.settings()->setRelativeTolerance(1e-6);

  // 6) solve
  solver.verbose = true;
<<<<<<< HEAD
  solver.solve(qp_problem);
  Eigen::VectorXd x = qp_problem->getVariableValues();
=======
  solver.Solve(nlp);
  Eigen::VectorXd x = nlp.GetOptVariables()->GetValues();
>>>>>>> fixup

  std::cout << x.transpose() << std::endl;

  tesseract_common::TrajArray inputs(1, 2);
  inputs << -0.75, 0.75;
  Eigen::Map<tesseract_common::TrajArray> results(x.data(), 1, 2);

<<<<<<< HEAD
  bool found = checkTrajectory(
      collisions, *manager, *state_solver, forward_kinematics->getJointNames(), inputs, *trajopt_collision_cnt_config);
=======
  tesseract_collision::CollisionCheckConfig config;
  config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
  bool found =
      checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), inputs, config);
>>>>>>> fixup

  EXPECT_TRUE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

  collisions.clear();
<<<<<<< HEAD
  found = checkTrajectory(
      collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results, *trajopt_collision_cnt_config);
=======
  found = checkTrajectory(collisions, *manager, *state_solver, forward_kinematics->getJointNames(), results, config);
>>>>>>> fixup

  EXPECT_FALSE(found);
  CONSOLE_BRIDGE_logWarn((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

<<<<<<< HEAD
// TEST_F(SimpleCollisionTest, spheres_ifopt_problem)  // NOLINT
//{
//  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres_ifopt_problem");
//  auto qp_problem = std::make_shared<trajopt_sqp::IfoptQPProblem>();
//  runSimpleCollisionTest(qp_problem, env);
//}

TEST_F(SimpleCollisionTest, spheres_trajopt_problem)  // NOLINT
{
  /**
   * @todo This test should produce the same table for cost and constraints but they do not.
   * Though this is the same as the older trajopt, it should be corrected. I believe there
   * is something different in how we calculate the merit for constraints compared to costs.
   * Constraints use violation versus cost uses the error.
   */
  CONSOLE_BRIDGE_logDebug("SimpleCollisionTest, spheres_trajopt_problem");
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
  runSimpleCollisionTest(qp_problem, env);
}

=======
>>>>>>> fixup
int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
