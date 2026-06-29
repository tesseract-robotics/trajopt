/**
 * @file cart_position_optimization_trajopt_sco_unit.cpp
 * @brief This is example is made to pair with cart_position_example.cpp.
 * This is the same motion planning problem in the trajopt_sco framework
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/common/types.h>
#include <tesseract/common/utils.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/plot_callback.hpp>
#include <trajopt/utils.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/logging.hpp>

using namespace trajopt;
using namespace tesseract::environment;
using namespace tesseract::scene_graph;
using namespace tesseract::common;

const bool DEBUG = true;

// This is example is made to pair with cart_position_example.cpp. This is the same motion planning problem in the
// trajopt_sco framework
TEST(CartPositionOptimizationTrajoptSCO, cart_position_optimization_trajopt_sco)  // NOLINT
{
  if (DEBUG)  // NOLINT
  {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG);
    trajopt_common::gLogLevel = trajopt_common::LevelInfo;
  }
  else
  {
    console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
    trajopt_common::gLogLevel = trajopt_common::LevelError;
  }

  // 1)  Load Robot
  const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
  const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
  const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
  auto env = std::make_shared<Environment>();
  env->init(urdf_file, srdf_file, locator);

  // Extract necessary kinematic information
  const tesseract::kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");

  ProblemConstructionInfo pci(env);

  // Populate Basic Info
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = manip;

  // Populate Init Info
  const SceneState current_state = pci.env->getState();
  Eigen::VectorXd start_pos(manip->numJoints());
  start_pos << 0, 0, 0, -1.0, 0, -1, 0.0;
  if (DEBUG)
    std::cout << "Joint Limits:\n" << manip->getLimits().joint_limits.transpose() << '\n';

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract::common::TrajArray(1, pci.kin->numJoints());
  auto zero = Eigen::VectorXd::Zero(7);
  pci.init_info.data = zero.transpose();

  auto joint_target = start_pos;
  auto target_pose = manip->calcFwdKin(joint_target).at("r_gripper_tool_frame");

  if (DEBUG)
    std::cout << "target_pose:\n" << target_pose.matrix() << '\n';

  {
    auto pose = std::make_shared<CartPoseTermInfo>();
    pose->term_type = TermType::TT_CNT;
    pose->name = "waypoint_cart_0";
    pose->timestep = 0;
    pose->source_frame = "r_gripper_tool_frame";
    pose->target_frame = "base_footprint";
    pose->target_frame_offset = target_pose;
    pose->pos_coeffs = Eigen::Vector3d(1, 1, 1);
    pose->rot_coeffs = Eigen::Vector3d(1, 1, 1);

    pci.cnt_infos.push_back(pose);
  }

  pci.basic_info.convex_solver = sco::ModelType::OSQP;

  auto prob = ConstructProblem(pci);

  sco::BasicTrustRegionSQP opt(prob);

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  tesseract::common::TrajArray traj = getTraj(opt.x(), prob->GetVars());

  auto optimized_pose = manip->calcFwdKin(traj.row(0)).at("r_gripper_tool_frame");
  EXPECT_TRUE(target_pose.translation().isApprox(optimized_pose.translation(), 1e-4));
  const Eigen::Quaterniond target_q(target_pose.rotation());
  const Eigen::Quaterniond optimized_q(optimized_pose.rotation());
  EXPECT_TRUE(target_q.isApprox(optimized_q, 1e-5));

  if (DEBUG)
  {
    std::cout << "Initial: " << prob->GetInitTraj() << '\n';
    std::cout << "Results: " << traj << '\n';
  }
}

// ---------------------------------------------------------------------------
// Helper: build ABB IRB2400 environment (shared by both discriminator tests).
// Uses output references + ASSERT_* so a misconfigured resource locator or a
// failed Environment::init() produces a clean gtest failure instead of a
// null-pointer segfault.
// ---------------------------------------------------------------------------
static void buildAbbIrb2400Env(std::shared_ptr<Environment>& env_out,
                               tesseract::kinematics::JointGroup::ConstPtr& manip_out)
{
  const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();

  const auto urdf_resource = locator->locateResource("package://tesseract/support/urdf/abb_irb2400.urdf");
  ASSERT_NE(urdf_resource, nullptr) << "Failed to locate abb_irb2400.urdf -- is TESSERACT_RESOURCE_PATH set?";
  const auto srdf_resource = locator->locateResource("package://tesseract/support/urdf/abb_irb2400.srdf");
  ASSERT_NE(srdf_resource, nullptr) << "Failed to locate abb_irb2400.srdf -- is TESSERACT_RESOURCE_PATH set?";

  // NOTE: explicit std::filesystem::path conversion is required so init() resolves to the
  // file-path overload rather than the urdf-string overload.
  const std::filesystem::path urdf_path = urdf_resource->getFilePath();
  const std::filesystem::path srdf_path = srdf_resource->getFilePath();

  auto env = std::make_shared<Environment>();
  ASSERT_TRUE(env->init(urdf_path, srdf_path, locator)) << "Environment::init() failed for abb_irb2400";

  auto manip = env->getJointGroup("manipulator");
  ASSERT_NE(manip, nullptr) << "JointGroup 'manipulator' not found in abb_irb2400";

  env_out = std::move(env);
  manip_out = std::move(manip);
}

// ---------------------------------------------------------------------------
// Discriminator Test A: seed outside the rz tolerance band -- band-interior
// assertion.
//
// Bug symptom (pre-fix): the optimizer stops inside the band short of the
// JointPos target.  Pre-fix, J[rz,joint_6] is non-zero even when rz is
// inside the band, creating a spurious ABS-penalty row that resists joint_6
// motion.  Post-fix: the Jac row is zero inside the band so JointPos moves
// the joint freely to its target on the opposite side of the band.
//
// Mechanism: joint_6 (ABB IRB2400 tool-roll) creates rz error without any
// TCP translation coupling (verified: joint_6=0.3 -> rz=0.3, tx/ty/tz=0).
// Joints 0-4 are pinned via TT_CNT equality constraints so the problem is
// effectively 1-DOF on joint_6.  This prevents the optimizer from using
// redundant joints to compensate the rz Jacobian row.
//
// Setup:
//   - start_pos  = [0, 0.5, -0.5, 0, 0.5, 0]
//   - target_pose = FK(start_pos)
//   - seed:  joint_6 += 0.9  (rz_err ~= 0.9, outside the +/-0.5 band)
//   - CartPose TT_COST: wide bands on all six components
//       tx/ty/tz +/-0.05 m, rx/ry/rz +/-0.5 rad, coeff = 10
//   - TT_CNT pin on joints 0-4 at start_pos (coeff[0..4]=1, coeff[5]=0)
//   - JointPos TT_COST: joint_6 target = start_pos[5] - 0.4 = -0.4 rad
//                       (rz ~= -0.4, inside band on the OPPOSITE side)
//                       coeff = 20.0 (ratio 10:20)
//
// The strong JointPos pull (2*20*0.9 = 36 >> CartPose resistance 10) causes
// the SQP to overshoot the band edge by a small amount.  Once strictly inside
// the band:
//   Post-fix: J = 0 -> no resistance -> joint_6 reaches -0.4 exactly.
//   Pre-fix: J[rz,j6] = 1 -> resistance = 10. Equilibrium at
//            2*20*|c+0.4| = 10 -> c ~= -0.15.
//            |opt[5] - (-0.4)| ~= 0.25 > 0.15 -> assertion fails.
//
// Stash-revert failure (observed): opt joint_6 ~= -0.15, gap ~= 0.25 > 0.15.
// ---------------------------------------------------------------------------
TEST(CartPositionOptimizationTrajoptSCO, cart_position_seed_outside_band_snaps_to_edge)  // NOLINT
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
  trajopt_common::gLogLevel = trajopt_common::LevelError;

  std::shared_ptr<Environment> env;
  tesseract::kinematics::JointGroup::ConstPtr manip;
  ASSERT_NO_FATAL_FAILURE(buildAbbIrb2400Env(env, manip));
  ASSERT_EQ(manip->numJoints(), 6U);

  Eigen::VectorXd start_pos(6);
  start_pos << 0.0, 0.5, -0.5, 0.0, 0.5, 0.0;
  const Eigen::Isometry3d target_pose = manip->calcFwdKin(start_pos).at("tool0");

  // Seed: joint_6 (index 5, tool-roll, axis [1,0,0] in link_5) perturbed +0.9
  // rad so rz_err ~= 0.9 (outside the +/-0.5 band).
  // joint_6 rotation does not change TCP translation (verified empirically).
  Eigen::VectorXd seed_pos = start_pos;
  seed_pos[5] += 0.9;

  ProblemConstructionInfo pci(env);
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = "manipulator";
  pci.basic_info.use_time = false;
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  pci.kin = manip;

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract::common::TrajArray(1, manip->numJoints());
  pci.init_info.data.row(0) = seed_pos.transpose();

  // CartPose soft-penalty cost with WIDE bands on all six components.
  // At seed: rz ~= 0.9 (outside +/-0.5) -> drives optimizer toward 0.
  // Inside band: post-fix Jac = 0; pre-fix Jac non-zero -> resists crossing.
  {
    auto pose = std::make_shared<CartPoseTermInfo>();
    pose->term_type = TermType::TT_COST;
    pose->name = "cart_pose_wide_band_A";
    pose->timestep = 0;
    pose->source_frame = "tool0";
    pose->target_frame = "base_link";
    pose->target_frame_offset = target_pose;
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->lower_tolerance = (Eigen::VectorXd(6) << -0.05, -0.05, -0.05, -0.5, -0.5, -0.5).finished();
    pose->upper_tolerance = (Eigen::VectorXd(6) << 0.05, 0.05, 0.05, 0.5, 0.5, 0.5).finished();
    pci.cost_infos.push_back(pose);
  }

  // TT_CNT equality constraints to pin joints 0-4 at start_pos.
  // coeff[5] = 0 means no constraint on joint_6 (free).
  // Pinning joints 0-4 makes the problem 1-DOF on joint_6 and prevents the
  // optimizer from using redundant joints to compensate the rz Jac row.
  {
    auto jp_pin = std::make_shared<JointPosTermInfo>();
    jp_pin->term_type = TermType::TT_CNT;
    jp_pin->name = "joint_pos_pin_A";
    jp_pin->first_step = 0;
    jp_pin->last_step = 0;
    jp_pin->targets = std::vector<double>(start_pos.data(), start_pos.data() + start_pos.size());
    jp_pin->coeffs = { 1.0, 1.0, 1.0, 1.0, 1.0, 0.0 };  // pin joints 0-4; coeff=0 disables joint_6's constraint row
    pci.cnt_infos.push_back(jp_pin);
  }

  // JointPos cost: joint_6 target = -0.4 (rz ~= -0.4, INSIDE band from the
  // OPPOSITE side).  coeff = 20.0.
  // Post-fix: all Jac rows zero inside band -> optimizer crosses to -0.4.
  // Pre-fix: J[rz,joint_6] != 0; inside band, equilibrium 2*20*|c+0.4| = 10
  // -> c ~= -0.15, stalls short of the -0.4 target.
  const double j6_target = start_pos[5] - 0.4;  // = -0.4
  {
    auto jp = std::make_shared<JointPosTermInfo>();
    jp->term_type = TermType::TT_COST;
    jp->name = "joint_pos_pull_A";
    jp->first_step = 0;
    jp->last_step = 0;
    std::vector<double> targets(start_pos.data(), start_pos.data() + start_pos.size());
    targets[5] = j6_target;  // joint_6 target = -0.4
    jp->targets = targets;
    // JointPos pull on joint_6: target = -0.4 (opposite side of the +0.5 band edge),
    // coeff = 20 (CartPose:JointPos gradient ratio ~10:40 at seed; strong pull
    // overshoots the band edge and enters the interior). Inside the band:
    // post-fix J = 0 -> free to reach -0.4; pre-fix J = 1 -> stops at ~-0.15
    // (where 2*20*|c+0.4| = 10 -> c ~= -0.15).
    jp->coeffs = std::vector<double>(6, 20.0);
    pci.cost_infos.push_back(jp);
  }

  auto prob = ConstructProblem(pci);
  sco::BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  const tesseract::common::TrajArray traj = getTraj(opt.x(), prob->GetVars());
  const Eigen::Isometry3d optimized_pose = manip->calcFwdKin(traj.row(0)).at("tool0");
  const Eigen::VectorXd opt_joints = traj.row(0).transpose();

  const Eigen::VectorXd pose_err = tesseract::common::calcTransformError(target_pose, optimized_pose);
  const double rz_err_final = pose_err[5];

  if (DEBUG)
  {
    std::cout << "[Test A] opt joint_6   = " << opt_joints[5] << "  (target = " << j6_target << ")\n";
    std::cout << "[Test A] rz_err_final  = " << rz_err_final << " (expect inside +/-0.5)\n";
    std::cout << "[Test A] pose_err      = " << pose_err.transpose() << "\n";
  }

  // Post-fix: once inside the band Jac rows = 0 -> JointPos free to cross to
  // ~-0.4 -> |opt[5] - (-0.4)| < 0.15.
  // Pre-fix: J[rz,joint_6] = 1 inside band -> resistance = 10.
  // Equilibrium: 2*20*|c+0.4| = 10 -> c ~= -0.15.
  // |opt[5] - (-0.4)| ~= 0.25 > 0.15 -> assertion fails.
  EXPECT_LT(std::abs(opt_joints[5] - j6_target), 0.15)
      << "joint_6 did not reach target: opt = " << opt_joints[5] << "  target = " << j6_target
      << "  (pre-fix: stops short at ~-0.15, gap ~0.25)";

  // rz must be inside the allowed band.
  EXPECT_LT(std::abs(rz_err_final), 0.5) << "rz_err_final = " << rz_err_final << " outside band +/-0.5";
}

// ---------------------------------------------------------------------------
// Discriminator Test B: seed inside ALL tolerance bands -- band-freedom
// assertion.
//
// Bug symptom (pre-fix): the spurious non-zero Jac rows (CartPose Jac
// ignoring tolerances when all errors are inside the band) resist any joint
// motion and pin the joint at the seed.  Post-fix: all Jac rows are zero
// inside the band and the JointPos cost can move the joint freely.
//
// joint_6 (tool-roll) is used as the motion joint because it creates rz error
// with zero translation coupling (verified: joint_6=0.3 -> rz=0.3, others=0).
//
// Setup:
//   - start_pos  = [0, 0.5, -0.5, 0, 0.5, 0]
//   - target_pose = FK(start_pos)
//   - seed_pos = start_pos (all pose errors = 0 -- inside all bands)
//   - CartPose TT_COST: WIDE bands tx/ty/tz +/-0.05 m, rx/ry/rz +/-0.5 rad
//     coeff = 10.  All errors = 0 at seed -> all components in-band.
//     Post-fix: all six Jac rows = 0 inside band -> no CartPose gradient.
//     Pre-fix: Jac is raw (tolerance-blind) -> non-zero even when err = 0.
//   - JointPos TT_COST: joint_6 target = start_pos[5] + 0.3 = 0.3, coeff=1.
//     At joint_6 = 0.3: rz ~= 0.3 (inside +/-0.5), translation ~= 0.
//     Post-fix: no resistance -> joint_6 moves to 0.3.
//     Pre-fix: non-zero Jac (ratio 10 >> 1) blocks motion -> joint_6 ~= 0.
//
// Stash-revert failure (observed): opt joint_6 = 0 -> |opt[5] - 0.3| = 0.3.
// ---------------------------------------------------------------------------
TEST(CartPositionOptimizationTrajoptSCO, cart_position_seed_inside_band_uses_band_freedom)  // NOLINT
{
  console_bridge::setLogLevel(console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);
  trajopt_common::gLogLevel = trajopt_common::LevelError;

  std::shared_ptr<Environment> env;
  tesseract::kinematics::JointGroup::ConstPtr manip;
  ASSERT_NO_FATAL_FAILURE(buildAbbIrb2400Env(env, manip));
  ASSERT_EQ(manip->numJoints(), 6U);

  Eigen::VectorXd start_pos(6);
  start_pos << 0.0, 0.5, -0.5, 0.0, 0.5, 0.0;
  const Eigen::Isometry3d target_pose = manip->calcFwdKin(start_pos).at("tool0");

  // Seed = start_pos: all pose errors = 0 -- all components inside all bands.

  ProblemConstructionInfo pci(env);
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = "manipulator";
  pci.basic_info.use_time = false;
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  pci.kin = manip;

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract::common::TrajArray(1, manip->numJoints());
  pci.init_info.data.row(0) = start_pos.transpose();

  // CartPose soft-penalty cost with WIDE bands on all six components.
  // At the seed every component = 0 -> all in-band.
  // Post-fix: all six Jac rows = 0 (err=0, perturbed_err=0 for small motion
  // inside the band) -> no CartPose gradient -> JointPos dominates.
  // Pre-fix: Jac is raw (tolerance-blind) -> non-zero even when err = 0
  // -> resists any joint motion.
  {
    auto pose = std::make_shared<CartPoseTermInfo>();
    pose->term_type = TermType::TT_COST;
    pose->name = "cart_pose_wide_band_B";
    pose->timestep = 0;
    pose->source_frame = "tool0";
    pose->target_frame = "base_link";
    pose->target_frame_offset = target_pose;
    pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
    pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    // Translation bands +/-0.05 m, rotation bands +/-0.5 rad.
    // At joint_6 = 0.3 rad: rz ~= 0.3 (inside +/-0.5), translation ~= 0.
    pose->lower_tolerance = (Eigen::VectorXd(6) << -0.05, -0.05, -0.05, -0.5, -0.5, -0.5).finished();
    pose->upper_tolerance = (Eigen::VectorXd(6) << 0.05, 0.05, 0.05, 0.5, 0.5, 0.5).finished();
    pci.cost_infos.push_back(pose);
  }

  // JointPos cost: joint_6 target = start_pos[5] + 0.3 = 0.3, coeff = 1.0.
  // The resulting rz change (~0.3) stays inside the +/-0.5 band.
  // Post-fix: all Jac rows = 0 -> optimizer moves freely -> joint_6 ~= 0.3.
  // Pre-fix: non-zero Jac rows (CartPose coeff 10 >> JointPos coeff 1)
  // resist motion -> joint_6 stays near 0.0.
  const double delta_j6 = 0.3;
  {
    auto jp = std::make_shared<JointPosTermInfo>();
    jp->term_type = TermType::TT_COST;
    jp->name = "joint_pos_pull_B";
    jp->first_step = 0;
    jp->last_step = 0;
    std::vector<double> targets(start_pos.data(), start_pos.data() + start_pos.size());
    targets[5] += delta_j6;  // joint_6 target = 0.3
    jp->targets = targets;
    jp->coeffs = std::vector<double>(6, 1.0);
    pci.cost_infos.push_back(jp);
  }

  auto prob = ConstructProblem(pci);
  sco::BasicTrustRegionSQP opt(prob);
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  const tesseract::common::TrajArray traj = getTraj(opt.x(), prob->GetVars());
  const Eigen::Isometry3d optimized_pose = manip->calcFwdKin(traj.row(0)).at("tool0");
  const Eigen::VectorXd opt_joints = traj.row(0).transpose();

  const Eigen::VectorXd pose_err = tesseract::common::calcTransformError(target_pose, optimized_pose);
  const double rz_err_final = pose_err[5];

  if (DEBUG)
  {
    std::cout << "[Test B] opt joint_6   = " << opt_joints[5] << "  (target = " << (start_pos[5] + delta_j6) << ")\n";
    std::cout << "[Test B] rz_err_final  = " << rz_err_final << " (expect inside +/-0.5)\n";
    std::cout << "[Test B] pose_err      = " << pose_err.transpose() << "\n";
  }

  // Post-fix: joint_6 must reach its JointPos target (all Jac rows = 0 inside
  // the wide bands -> no resistance from CartPose).
  // Pre-fix: non-zero Jac rows (CartPose coeff 10 >> JointPos coeff 1) resist
  // motion -> joint_6 stays near 0 -> this assertion fails.
  EXPECT_LT(std::abs(opt_joints[5] - (start_pos[5] + delta_j6)), 0.1)
      << "joint_6 did not move to target: opt = " << opt_joints[5] << "  target = " << (start_pos[5] + delta_j6)
      << "  (pre-fix: joint pinned near " << start_pos[5] << ")";

  // rz must stay within the allowed band.
  EXPECT_LT(std::abs(rz_err_final), 0.5) << "rz_err_final = " << rz_err_final << " outside band +/-0.5";
}
