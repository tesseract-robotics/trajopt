/**
 * @file cart_position_optimization_trajopt_sco_unit.cpp
 * @brief This is example is made to pair with cart_position_example.cpp.
 * This is the same motion planning problem in the trajopt_sco framework
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
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <iostream>
#include <tesseract_environment/environment.h>
#include <tesseract_environment/utils.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/types.h>
#include <tesseract_common/macros.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/logging.hpp>

using namespace trajopt;
using namespace tesseract_environment;
using namespace tesseract_scene_graph;
using namespace tesseract_common;

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
  tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
  tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");
  ResourceLocator::Ptr locator = std::make_shared<tesseract_common::GeneralResourceLocator>();
  auto env = std::make_shared<Environment>();
  env->init(urdf_file, srdf_file, locator);

  // Extract necessary kinematic information
  tesseract_kinematics::JointGroup::ConstPtr manip = env->getJointGroup("right_arm");

  ProblemConstructionInfo pci(env);

  // Populate Basic Info
  pci.basic_info.n_steps = 1;
  pci.basic_info.manip = "right_arm";
  pci.basic_info.use_time = false;

  // Create Kinematic Object
  pci.kin = manip;

  // Populate Init Info
  SceneState current_state = pci.env->getState();
  Eigen::VectorXd start_pos(manip->numJoints());
  start_pos << 0, 0, 0, -1.0, 0, -1, 0.0;
  if (DEBUG)
    std::cout << "Joint Limits:\n" << manip->getLimits().joint_limits.transpose() << std::endl;

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = tesseract_common::TrajArray(1, pci.kin->numJoints());
  auto zero = Eigen::VectorXd::Zero(7);
  pci.init_info.data = zero.transpose();

  auto joint_target = start_pos;
  auto target_pose = manip->calcFwdKin(joint_target).at("r_gripper_tool_frame");

  if (DEBUG)
    std::cout << "target_pose:\n" << target_pose.matrix() << std::endl;

  {
    auto pose = std::make_shared<CartPoseTermInfo>();
    pose->term_type = TT_CNT;
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

  tesseract_common::TrajArray traj = getTraj(opt.x(), prob->GetVars());

  auto optimized_pose = manip->calcFwdKin(traj.row(0)).at("r_gripper_tool_frame");
  EXPECT_TRUE(target_pose.translation().isApprox(optimized_pose.translation(), 1e-4));
  Eigen::Quaterniond target_q(target_pose.rotation());
  Eigen::Quaterniond optimized_q(optimized_pose.rotation());
  EXPECT_TRUE(target_q.isApprox(optimized_q, 1e-5));

  if (DEBUG)
  {
    std::cout << "Initial: " << prob->GetInitTraj() << std::endl;
    std::cout << "Results: " << traj << std::endl;
  }
}
