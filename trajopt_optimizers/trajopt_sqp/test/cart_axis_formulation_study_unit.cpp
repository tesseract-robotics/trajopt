/**
 * @file cart_axis_formulation_study_unit.cpp
 * @brief Seeded comparison of the cartesian axis constraints against alternative formulations
 *
 * Every formulation solves the same orientation-only problems with TrajOptQPProblem: the tool z axis of the PR2 right
 * arm must point along (align) or within a cone around (cone) the tool z axis at a random reachable configuration,
 * starting from another random configuration. Problems are feasible by construction. Two bands of start angle are
 * sampled: any angle, and within 30 degrees of antiparallel, where gradient methods meet the antipodal stationary
 * point of any smooth rotation-invariant measure on the sphere.
 *
 * The success table is printed; what is asserted is listed on the test.
 *
 * @author Jelle Feringa
 * @date October 1, 2026
 *
 * @copyright Copyright (c) 2026
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <random>
#include <gtest/gtest.h>

#include <OsqpEigen/OsqpEigen.h>

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/environment/environment.h>
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/trajopt_qp_problem.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/osqp_eigen_solver.h>

#include <trajopt_ifopt/constraints/cartesian_axis_align_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_axis_cone_constraint.h>
#include <trajopt_ifopt/constraints/cartesian_axis_kinematics.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>
#include <trajopt_ifopt/variable_sets/nodes_variables.h>
#include <trajopt_ifopt/variable_sets/node.h>
#include <trajopt_ifopt/variable_sets/var.h>
#include <trajopt_ifopt/utils/ifopt_utils.h>

using trajopt_ifopt::Bounds;
using trajopt_ifopt::CartAxisKinematics;
using trajopt_ifopt::Jacobian;

namespace
{
const std::string TOOL{ "r_gripper_tool_frame" };
const std::string BASE{ "base_footprint" };

/** Problems per band; with 2 bands and 7 formulations this is 1400 solves, a few seconds. */
constexpr int PROBLEMS_PER_BAND = 100;
constexpr unsigned SEED = 20261001;
/** Cone half angles [rad]: a moderate tolerance and a tight one, where the round boundary's curvature 1/theta shows */
const std::vector<double> HALF_ANGLES{ 0.2, 0.01 };
/**
 * Allowed shortfall of a new constraint against a baseline with the same feasible set [problems]: three standard
 * deviations of a binomial count at the observed success rate, sqrt(100 * 0.99 * 0.01) ~ 1, so 3.
 */
constexpr int PARITY_SLACK = 3;
/** Regression floor for the new constraints [problems out of PROBLEMS_PER_BAND] */
constexpr int NEW_FLOOR = 95;
/** A solve succeeds when the true axis angle is within this of its bound [rad], matching cnt_tolerance's scale. */
constexpr double SUCCESS_TOL = 1e-3;
/** Lower edge of the near-antiparallel band [rad]: 150 degrees */
constexpr double ANTIPARALLEL_BAND = 5 * M_PI / 6;
/** Joints without finite limits are sampled in [-pi, pi] [rad] */
constexpr double UNLIMITED_JOINT_RANGE = M_PI;
/** trajopt_sco's ConstraintFromFunc forward-difference step, which PR #6's terms relied on [rad] */
constexpr double SCO_FD_STEP = 1e-5;

/** @brief A constraint set from value and jacobian functions of the joint values, for the baselines below */
class FunctionConstraint : public trajopt_ifopt::ConstraintSet
{
public:
  using ValueFn = std::function<Eigen::VectorXd(const Eigen::VectorXd&)>;
  using JacobianFn = std::function<Eigen::MatrixXd(const Eigen::VectorXd&)>;

  FunctionConstraint(std::string name,
                     std::shared_ptr<const trajopt_ifopt::Var> var,
                     std::vector<Bounds> bounds,
                     ValueFn value,
                     JacobianFn jacobian)
    : ConstraintSet(std::move(name), static_cast<int>(bounds.size()))
    , var_(std::move(var))
    , bounds_(std::move(bounds))
    , value_(std::move(value))
    , jacobian_(std::move(jacobian))
  {
  }

  int update() override { return rows_; }
  Eigen::VectorXd getValues() const override { return value_(var_->value()); }
  Eigen::VectorXd getCoefficients() const override { return Eigen::VectorXd::Ones(rows_); }
  std::vector<Bounds> getBounds() const override { return bounds_; }

  Jacobian getJacobian() const override
  {
    const Eigen::MatrixXd dense = jacobian_(var_->value());
    Jacobian jac(rows_, variables_->getRows());
    for (Eigen::Index i = 0; i < rows_; ++i)
    {
      jac.startVec(i);
      for (Eigen::Index j = 0; j < dense.cols(); ++j)
        jac.insertBack(i, var_->getIndex() + j) = dense(i, j);
    }
    jac.finalize();  // NOLINT
    return jac;
  }

private:
  std::shared_ptr<const trajopt_ifopt::Var> var_;
  std::vector<Bounds> bounds_;
  ValueFn value_;
  JacobianFn jacobian_;
};

Eigen::MatrixXd forwardDifference(const FunctionConstraint::ValueFn& f, const Eigen::VectorXd& q)
{
  const Eigen::VectorXd f0 = f(q);
  Eigen::MatrixXd jac(f0.size(), q.size());
  for (Eigen::Index i = 0; i < q.size(); ++i)
  {
    Eigen::VectorXd qp = q;
    qp(i) += SCO_FD_STEP;
    jac.col(i) = (f(qp) - f0) / SCO_FD_STEP;
  }
  return jac;
}

/** A rotation whose z axis is @p axis */
Eigen::Isometry3d frameWithZ(const Eigen::Vector3d& axis)
{
  Eigen::Matrix3d r;
  r.col(2) = axis.normalized();
  r.col(0) = r.col(2).unitOrthogonal();
  r.col(1) = r.col(2).cross(r.col(0));
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = r;
  return pose;
}

double angleBetween(const Eigen::Vector3d& a, const Eigen::Vector3d& b)
{
  return std::atan2(a.cross(b).norm(), a.dot(b));
}

/** What the study asserts about a formulation */
enum class Role : std::uint8_t
{
  kNew,          // solves at least NEW_FLOOR problems per band, and within PARITY_SLACK of every same-set baseline
  kSameSet,      // same feasible set as the new constraint in its group
  kKnownDefect,  // solves under half the near-antiparallel band; pins down why it was replaced
  kDifferentSet  // different feasible set: reported, not asserted
};

struct Formulation
{
  std::string name;
  /** Formulations in one group solve the same task: "align", or "cone" at one half angle */
  std::string group;
  /** The task is solved when the true axis angle is at most this [rad] */
  double bound;
  Role role;
  /** Builds the constraint for target axis a (base coordinates) on the position variable */
  std::function<std::shared_ptr<trajopt_ifopt::ConstraintSet>(const std::shared_ptr<const trajopt_ifopt::Var>&,
                                                              const tesseract::kinematics::JointGroup::ConstPtr&,
                                                              const Eigen::Vector3d&)>
      make;
};

std::vector<Formulation> formulations()
{
  const std::string align = "align";
  const Eigen::Vector3d z = Eigen::Vector3d::UnitZ();
  using VarPtr = std::shared_ptr<const trajopt_ifopt::Var>;
  using ManipPtr = tesseract::kinematics::JointGroup::ConstPtr;
  std::vector<Formulation> out;

  out.push_back({ "align: log map (new)",
                  align,
                  0.0,
                  Role::kNew,
                  [z](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                    return std::make_shared<trajopt_ifopt::CartAxisAlignConstraint>(var, m, TOOL, z, BASE, a);
                  } });

  out.push_back({ "align: P v = 0, a.v >= 0",
                  align,
                  0.0,
                  Role::kKnownDefect,
                  [z](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                    auto kin = std::make_shared<CartAxisKinematics>(m, TOOL, z, BASE);
                    Eigen::Matrix3d basis;
                    basis.row(0) = a.unitOrthogonal().transpose();
                    basis.row(1) = a.cross(a.unitOrthogonal()).transpose();
                    basis.row(2) = a.transpose();
                    return std::make_shared<FunctionConstraint>(
                        "HalfSpaceAlign",
                        var,
                        std::vector<Bounds>{ trajopt_ifopt::BoundZero, trajopt_ifopt::BoundZero, Bounds(0, INFINITY) },
                        [kin, basis](const Eigen::VectorXd& q) -> Eigen::VectorXd { return basis * kin->calcAxis(q); },
                        [kin, basis](const Eigen::VectorXd& q) -> Eigen::MatrixXd {
                          Eigen::Vector3d v;
                          Eigen::Matrix3Xd dv;
                          kin->calcAxisAndJacobian(q, v, dv);
                          return basis * dv;
                        });
                  } });

  out.push_back({ "align: CartPos, r_z dropped",
                  align,
                  0.0,
                  Role::kSameSet,
                  [](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                    Eigen::VectorXd coeffs(6);
                    coeffs << 0, 0, 0, 1, 1, 0;
                    return std::make_shared<trajopt_ifopt::CartPosConstraint>(
                        var,
                        coeffs,
                        std::vector<Bounds>(6, trajopt_ifopt::BoundZero),
                        m,
                        TOOL,
                        BASE,
                        Eigen::Isometry3d::Identity(),
                        frameWithZ(a));
                  } });

  for (const double theta : HALF_ANGLES)
  {
    std::stringstream label;
    label << "cone " << theta;
    const std::string cone = label.str();

    out.push_back({ cone + ": atan2 angle (new)",
                    cone,
                    theta,
                    Role::kNew,
                    [z, theta](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                      return std::make_shared<trajopt_ifopt::CartAxisConeConstraint>(var, m, TOOL, z, BASE, a, theta);
                    } });

    out.push_back({ cone + ": acos, forward diff (PR #6)",
                    cone,
                    theta,
                    Role::kSameSet,
                    [z, theta](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                      auto kin = std::make_shared<CartAxisKinematics>(m, TOOL, z, BASE);
                      FunctionConstraint::ValueFn value = [kin, a](const Eigen::VectorXd& q) -> Eigen::VectorXd {
                        return Eigen::VectorXd::Constant(1, std::acos(std::clamp(a.dot(kin->calcAxis(q)), -1.0, 1.0)));
                      };
                      return std::make_shared<FunctionConstraint>(
                          "AcosCone",
                          var,
                          std::vector<Bounds>{ Bounds(-INFINITY, theta) },
                          value,
                          [value](const Eigen::VectorXd& q) { return forwardDifference(value, q); });
                    } });

    out.push_back({ cone + ": a.v >= cos(theta)",
                    cone,
                    theta,
                    Role::kSameSet,
                    [z, theta](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                      auto kin = std::make_shared<CartAxisKinematics>(m, TOOL, z, BASE);
                      return std::make_shared<FunctionConstraint>(
                          "CosineCone",
                          var,
                          std::vector<Bounds>{ Bounds(std::cos(theta), INFINITY) },
                          [kin, a](const Eigen::VectorXd& q) -> Eigen::VectorXd {
                            return Eigen::VectorXd::Constant(1, a.dot(kin->calcAxis(q)));
                          },
                          [kin, a](const Eigen::VectorXd& q) -> Eigen::MatrixXd {
                            Eigen::Vector3d v;
                            Eigen::Matrix3Xd dv;
                            kin->calcAxisAndJacobian(q, v, dv);
                            return a.transpose() * dv;
                          });
                    } });

    out.push_back({ cone + ": CartPos, inscribed box",
                    cone,
                    theta,
                    Role::kDifferentSet,
                    [theta](const VarPtr& var, const ManipPtr& m, const Eigen::Vector3d& a) {
                      Eigen::VectorXd coeffs(6);
                      coeffs << 0, 0, 0, 1, 1, 0;
                      const double half_width = theta / std::sqrt(2.0);
                      return std::make_shared<trajopt_ifopt::CartPosConstraint>(
                          var,
                          coeffs,
                          std::vector<Bounds>(6, Bounds(-half_width, half_width)),
                          m,
                          TOOL,
                          BASE,
                          Eigen::Isometry3d::Identity(),
                          frameWithZ(a));
                    } });
  }
  return out;
}

struct Problem
{
  Eigen::VectorXd start;
  Eigen::Vector3d target_axis;  // base coordinates
};

struct Outcome
{
  bool success;
  int iterations;
};

class Sampler
{
public:
  explicit Sampler(const tesseract::kinematics::JointGroup::ConstPtr& manip) : manip_(manip), rng_(SEED)
  {
    const Eigen::MatrixX2d limits = manip_->getLimits().joint_limits;
    lower_ = limits.col(0).cwiseMax(-UNLIMITED_JOINT_RANGE);
    upper_ = limits.col(1).cwiseMin(UNLIMITED_JOINT_RANGE);
  }

  Eigen::Vector3d toolAxis(const Eigen::VectorXd& q) const
  {
    const auto poses = manip_->calcFwdKin(q);
    return (poses.at(BASE).inverse() * poses.at(TOOL)).linear().col(2);
  }

  /** Start and target with start-to-target axis angle at least @p min_angle [rad] */
  Problem sample(double min_angle)
  {
    const Eigen::Vector3d target_axis = toolAxis(randomConfiguration());
    for (int attempt = 0; attempt < 100000; ++attempt)
    {
      const Eigen::VectorXd start = randomConfiguration();
      if (angleBetween(toolAxis(start), target_axis) >= min_angle)
        return { start, target_axis };
    }
    throw std::runtime_error("Sampler: no start found with axis angle >= " + std::to_string(min_angle));
  }

private:
  tesseract::kinematics::JointGroup::ConstPtr manip_;
  std::mt19937 rng_;
  Eigen::VectorXd lower_;
  Eigen::VectorXd upper_;

  Eigen::VectorXd randomConfiguration()
  {
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Eigen::VectorXd q(lower_.size());
    for (Eigen::Index i = 0; i < q.size(); ++i)
      q(i) = lower_(i) + unit(rng_) * (upper_(i) - lower_(i));
    return q;
  }
};

Outcome solve(const tesseract::kinematics::JointGroup::ConstPtr& manip,
              const Sampler& sampler,
              const Formulation& formulation,
              const Problem& problem)
{
  auto qp_solver = std::make_shared<trajopt_sqp::OSQPEigenSolver>();
  trajopt_sqp::TrustRegionSQPSolver solver(qp_solver);
  qp_solver->solver_->settings()->setVerbosity(false);
  qp_solver->solver_->settings()->setWarmStart(true);
  qp_solver->solver_->settings()->setPolish(true);
  qp_solver->solver_->settings()->setAdaptiveRho(false);
  qp_solver->solver_->settings()->setMaxIteration(8192);
  qp_solver->solver_->settings()->setAbsoluteTolerance(1e-4);
  qp_solver->solver_->settings()->setRelativeTolerance(1e-6);

  std::vector<std::unique_ptr<trajopt_ifopt::Node>> nodes;
  auto node = std::make_unique<trajopt_ifopt::Node>("Joint_Position_0");
  auto var = node->addVar("position",
                          tesseract::common::toNames(manip->getJointIds()),
                          problem.start,
                          trajopt_ifopt::toBounds(manip->getLimits().joint_limits));
  nodes.push_back(std::move(node));
  auto qp_problem = std::make_shared<trajopt_sqp::TrajOptQPProblem>(
      std::make_shared<trajopt_ifopt::NodesVariables>("joint_trajectory", std::move(nodes)));
  qp_problem->addConstraintSet(formulation.make(var, manip, problem.target_axis));
  qp_problem->setup();
  solver.solve(qp_problem);

  const double angle = angleBetween(sampler.toolAxis(qp_problem->getVariableValues()), problem.target_axis);
  const bool success =
      solver.getStatus() == trajopt_sqp::SQPStatus::kConverged && angle <= formulation.bound + SUCCESS_TOL;

  return { success, solver.getResults().overall_iteration };
}
}  // namespace

class CartAxisFormulationStudy : public testing::Test
{
public:
  tesseract::environment::Environment::Ptr env;

  void SetUp() override
  {
    tesseract::common::getLogger()->set_level(spdlog::level::off);
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
    const auto locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    env = std::make_shared<tesseract::environment::Environment>();
    ASSERT_TRUE(env->init(urdf_file, srdf_file, locator));
  }
};

/**
 * @brief Success counts and median iterations per formulation and start band.
 *
 * Asserted: each new constraint solves at least NEW_FLOOR problems per band and is within sampling noise
 * (PARITY_SLACK) of every baseline with the same feasible set; the half-space align rows it replaced fail in the
 * near-antiparallel band. The inscribed-box CartPos cone has a smaller feasible set and is reported only.
 *
 * What this shows, and what it does not: the new constraints are as robust as the alternatives with the same feasible
 * set, not more. The cone's distinguishing property is its feasible set; tight cones cost more SQP iterations than a
 * box because a linearizing solver approximates a boundary of curvature 1/theta.
 */
TEST_F(CartAxisFormulationStudy, SuccessRatesPerFormulation)  // NOLINT
{
  const tesseract::kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");
  const std::vector<Formulation> forms = formulations();

  const std::vector<std::pair<std::string, double>> bands{ { "any start", 0.0 },
                                                           { "start >= 150 deg", ANTIPARALLEL_BAND } };
  Sampler sampler(manip);

  // successes[band][formulation], median iterations of successful solves
  std::map<std::string, std::map<std::string, int>> successes;
  std::map<std::string, std::map<std::string, double>> median_iterations;
  for (const auto& [band, min_angle] : bands)
  {
    std::vector<Problem> problems;
    for (int i = 0; i < PROBLEMS_PER_BAND; ++i)
      problems.push_back(sampler.sample(min_angle));

    for (const auto& form : forms)
    {
      std::vector<int> iterations;
      for (const auto& problem : problems)
      {
        const Outcome outcome = solve(manip, sampler, form, problem);
        if (outcome.success)
          iterations.push_back(outcome.iterations);
      }
      successes[band][form.name] = static_cast<int>(iterations.size());
      std::sort(iterations.begin(), iterations.end());
      median_iterations[band][form.name] = iterations.empty() ? std::nan("") : iterations[iterations.size() / 2];
    }
  }

  std::cout << "\nCartesian axis formulations, " << PROBLEMS_PER_BAND << " problems per band, seed " << SEED
            << ", cone half angles in rad\n";
  std::cout << std::left << std::setw(40) << "formulation";
  for (const auto& [band, min_angle] : bands)
    std::cout << std::setw(28) << (band + ": solved / med it");
  std::cout << '\n';
  for (const auto& form : forms)
  {
    std::cout << std::setw(40) << form.name;
    for (const auto& [band, min_angle] : bands)
    {
      std::stringstream cell;
      cell << successes[band][form.name] << "/" << PROBLEMS_PER_BAND << "  " << median_iterations[band][form.name];
      std::cout << std::setw(28) << cell.str();
    }
    std::cout << '\n';
  }

  for (const auto& [band, min_angle] : bands)
  {
    for (const auto& form : forms)
    {
      if (form.role == Role::kKnownDefect && min_angle > 0)
        EXPECT_LT(successes[band][form.name], PROBLEMS_PER_BAND / 2) << band << ": " << form.name;

      if (form.role != Role::kNew)
        continue;
      EXPECT_GE(successes[band][form.name], NEW_FLOOR) << band << ": " << form.name;
      for (const auto& baseline : forms)
      {
        if (baseline.role == Role::kSameSet && baseline.group == form.group)
          EXPECT_GE(successes[band][form.name] + PARITY_SLACK, successes[band][baseline.name])
              << band << ": " << form.name << " vs " << baseline.name;
      }
    }
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
