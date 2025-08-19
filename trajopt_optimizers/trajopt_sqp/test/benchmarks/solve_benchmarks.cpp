#include <tesseract_common/macros.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <OsqpEigen/Solver.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <algorithm>
#include <ifopt/problem.h>
#include <ifopt/constraint_set.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>

#include <trajopt_ifopt/utils/numeric_differentiation.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_constraint.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>
#include <trajopt_ifopt/constraints/joint_position_constraint.h>
#include <trajopt_ifopt/constraints/joint_velocity_constraint.h>
#include <trajopt_ifopt/costs/squared_cost.h>
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

#include <trajopt_sqp/ifopt_qp_problem.h>
#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

using namespace trajopt_ifopt;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

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

      CalcJacobianBlock(joint_vals, jac_block);  // NOLINT
    }
  }

  Eigen::VectorXd CalcValues(const Eigen::Ref<const Eigen::VectorXd>& joint_vals) const
  {
    Eigen::VectorXd err = Eigen::VectorXd::Zero(3);

    // Check the collisions
    trajopt_common::CollisionCacheData::ConstPtr cdata =
        collision_evaluator_->CalcCollisions(joint_vals, bounds_.size());

    if (cdata->contact_results_map.empty())
      return err;

    Eigen::Index i{ 0 };
    for (const auto& pair : cdata->contact_results_map)
    {
      for (const auto& dist_result : pair.second)
      {
        double dist = collision_evaluator_->GetCollisionMarginData().getCollisionMargin(dist_result.link_names[0],
                                                                                        dist_result.link_names[1]);
        double coeff = collision_evaluator_->GetCollisionCoeffData().getCollisionCoeff(dist_result.link_names[0],
                                                                                       dist_result.link_names[1]);
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
    trajopt_common::CollisionCacheData::ConstPtr cdata =
        collision_evaluator_->CalcCollisions(joint_vals, bounds_.size());

    // Get gradients for all contacts
    /** @todo Use the cdata gradient results */
    std::vector<trajopt_common::GradientResults> grad_results;
    for (const auto& pair : cdata->contact_results_map)
    {
      for (const auto& dist_result : pair.second)
      {
        trajopt_common::GradientResults result = collision_evaluator_->GetGradient(joint_vals, dist_result);
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

/** @brief Benchmark trajopt ifopt simple collision solve */
static void BM_TRAJOPT_IFOPT_SIMPLE_COLLISION_SOLVE(benchmark::State& state, Environment::Ptr env)
{
  for (auto _ : state)
  {
    auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
    tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("manipulator");
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
    std::vector<Eigen::VectorXd> positions;
    {
      Eigen::VectorXd pos(2);
      pos << -0.75, 0.75;
      positions.push_back(pos);
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(pos, manip->getJointNames(), "Joint_Position_0");
      vars.push_back(var);
      qp_problem->addVariableSet(var);
    }

    // Step 3: Setup collision
    auto trajopt_collision_cnt_config = std::make_shared<trajopt_common::TrajOptCollisionConfig>(0.2, 1);
    trajopt_collision_cnt_config->collision_margin_buffer = 0.05;

    auto collision_cnt_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
    trajopt_ifopt::DiscreteCollisionEvaluator::Ptr collision_cnt_evaluator =
        std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
            collision_cnt_cache, manip, env, *trajopt_collision_cnt_config);
    auto collision_cnt = std::make_shared<SimpleCollisionConstraintIfopt>(collision_cnt_evaluator, vars[0]);
    qp_problem->addConstraintSet(collision_cnt);

    auto trajopt_collision_cost_config = std::make_shared<trajopt_common::TrajOptCollisionConfig>(0.3, 1);
    trajopt_collision_cost_config->collision_margin_buffer = 0.05;

    auto collision_cost_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
    trajopt_ifopt::DiscreteCollisionEvaluator::Ptr collision_cost_evaluator =
        std::make_shared<trajopt_ifopt::SingleTimestepCollisionEvaluator>(
            collision_cost_cache, manip, env, *trajopt_collision_cost_config);
    auto collision_cost = std::make_shared<SimpleCollisionConstraintIfopt>(collision_cost_evaluator, vars[0]);
    qp_problem->addCostSet(collision_cost, trajopt_sqp::CostPenaltyType::HINGE);

    Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(2, 1);
    auto jp_cost = std::make_shared<trajopt_ifopt::JointPosConstraint>(Eigen::Vector2d(0, 0), vars, coeffs);
    qp_problem->addCostSet(jp_cost, trajopt_sqp::CostPenaltyType::SQUARED);

    qp_problem->setup();

    // 5) choose solver and options
    auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
    trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
    qp_solver->solver_->settings()->setVerbosity(false);
    qp_solver->solver_->settings()->setWarmStart(true);
    qp_solver->solver_->settings()->setPolish(true);
    qp_solver->solver_->settings()->setAdaptiveRho(false);
    qp_solver->solver_->settings()->setMaxIteration(8192);
    qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
    qp_solver->solver_->settings()->setRelativeTolerance(1e-6);

    // 6) solve
    solver.verbose = false;
    solver.solve(qp_problem);
  }
}

static void BM_TRAJOPT_IFOPT_PLANNING_SOLVE(benchmark::State& state, Environment::Ptr env)
{
  for (auto _ : state)
  {
    auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>();
    tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");

    // Initial trajectory
    tesseract_common::TrajArray trajectory(6, 7);
    trajectory.row(0) << -1.832, -0.332, -1.011, -1.437, -1.1, -1.926, 3.074;
    trajectory.row(1) << -1.411, 0.028, -0.764, -1.463, -1.525, -1.698, 3.055;
    trajectory.row(2) << -0.99, 0.388, -0.517, -1.489, -1.949, -1.289, 3.036;
    trajectory.row(3) << -0.569, 0.747, -0.27, -1.515, -2.374, -0.881, 3.017;
    trajectory.row(4) << -0.148, 1.107, -0.023, -1.541, -2.799, -0.472, 2.998;
    trajectory.row(5) << 0.062, 1.287, 0.1, -1.554, -3.011, -0.268, 2.988;

    // Add Variables
    std::vector<trajopt_ifopt::JointPosition::ConstPtr> vars;
    for (Eigen::Index i = 0; i < 6; ++i)
    {
      auto var = std::make_shared<trajopt_ifopt::JointPosition>(
          trajectory.row(i), manip->getJointNames(), "Joint_Position_" + std::to_string(i));
      vars.push_back(var);
      qp_problem->addVariableSet(var);
    }

    double margin_coeff = 20;
    double margin = 0.025;
    auto trajopt_collision_config = std::make_shared<trajopt_common::TrajOptCollisionConfig>(margin, margin_coeff);
    trajopt_collision_config->collision_check_config.type = tesseract_collision::CollisionEvaluatorType::CONTINUOUS;
    trajopt_collision_config->collision_margin_buffer = 0.01;

    // Add costs
    {
      Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(1, 1);
      auto cnt = std::make_shared<JointVelConstraint>(Eigen::VectorXd::Zero(7), vars, coeffs);
      qp_problem->addCostSet(cnt, trajopt_sqp::CostPenaltyType::SQUARED);
    }

    // Add constraints
    {  // Fix start position
      std::vector<JointPosition::ConstPtr> fixed_vars = { vars[0] };
      Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
      auto cnt = std::make_shared<JointPosConstraint>(trajectory.row(0), fixed_vars, coeffs);
      qp_problem->addConstraintSet(cnt);
    }

    {  // Fix end position
      std::vector<trajopt_ifopt::JointPosition::ConstPtr> fixed_vars = { vars[5] };
      Eigen::VectorXd coeffs = Eigen::VectorXd::Constant(manip->numJoints(), 5);
      auto cnt = std::make_shared<trajopt_ifopt::JointPosConstraint>(trajectory.row(5), fixed_vars, coeffs);
      qp_problem->addConstraintSet(cnt);
    }

    auto collision_cache = std::make_shared<trajopt_ifopt::CollisionCache>(100);
    std::array<bool, 2> position_vars_fixed{ false, false };
    for (std::size_t i = 1; i < (vars.size() - 1); ++i)
    {
      auto collision_evaluator = std::make_shared<trajopt_ifopt::LVSContinuousCollisionEvaluator>(
          collision_cache, manip, env, *trajopt_collision_config);

      std::array<JointPosition::ConstPtr, 2> position_vars{ vars[i - 1], vars[i] };

      if (i == 1)
        position_vars_fixed = { true, false };
      else if (i == (vars.size() - 1))
        position_vars_fixed = { false, true };
      else
        position_vars_fixed = { false, false };

      auto cnt = std::make_shared<trajopt_ifopt::ContinuousCollisionConstraint>(
          collision_evaluator, position_vars, position_vars_fixed[0], position_vars_fixed[1], 5);

      qp_problem->addCostSet(cnt, trajopt_sqp::CostPenaltyType::HINGE);
    }

    qp_problem->setup();

    // Setup solver
    auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
    trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
    qp_solver->solver_->settings()->setVerbosity(false);
    qp_solver->solver_->settings()->setWarmStart(true);
    qp_solver->solver_->settings()->setPolish(true);
    qp_solver->solver_->settings()->setAdaptiveRho(false);
    qp_solver->solver_->settings()->setMaxIteration(8192);
    qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
    qp_solver->solver_->settings()->setRelativeTolerance(1e-6);

    // 6) solve
    solver.verbose = false;
    solver.solve(qp_problem);
  }
}

int main(int argc, char** argv)
{
  //////////////////////////////////////
  // Simple Collision Solve
  //////////////////////////////////////

  {
    std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<Environment>();
    env->init(urdf_file, srdf_file, locator);

    std::unordered_map<std::string, double> ipos;
    ipos["spherebot_x_joint"] = -0.75;
    ipos["spherebot_y_joint"] = 0.75;
    env->setState(ipos);

    std::function<void(benchmark::State&, Environment::Ptr)> BM_SOLVE_FUNC = BM_TRAJOPT_IFOPT_SIMPLE_COLLISION_SOLVE;
    std::string name = "BM_TRAJOPT_IFOPT_SIMPLE_COLLISION_SOLVE";
    benchmark::RegisterBenchmark(name, BM_SOLVE_FUNC, env)->UseRealTime()->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  //////////////////////////////////////
  // Planning Solve
  //////////////////////////////////////
  {
    std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");

    ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
    auto env = std::make_shared<Environment>();
    env->init(urdf_file, srdf_file, locator);

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0.0;
    ipos["r_shoulder_pan_joint"] = -1.832;
    ipos["r_shoulder_lift_joint"] = -0.332;
    ipos["r_upper_arm_roll_joint"] = -1.011;
    ipos["r_elbow_flex_joint"] = -1.437;
    ipos["r_forearm_roll_joint"] = -1.1;
    ipos["r_wrist_flex_joint"] = -1.926;
    ipos["r_wrist_roll_joint"] = 3.074;
    env->setState(ipos);

    std::function<void(benchmark::State&, Environment::Ptr)> BM_SOLVE_FUNC = BM_TRAJOPT_IFOPT_PLANNING_SOLVE;
    std::string name = "BM_TRAJOPT_IFOPT_PLANNING_SOLVE";
    benchmark::RegisterBenchmark(name, BM_SOLVE_FUNC, env)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMillisecond)
        ->MinTime(6);
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
