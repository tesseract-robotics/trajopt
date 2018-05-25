/**
 * @file glass_up_right_plan.cpp
 * @brief Example using Trajopt for constrained free space planning
 *
 * @author Levi Armstrong
 * @date Dec 18, 2017
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
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
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <ros/ros.h>
#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_moveit/trajopt_moveit_env.h>
#include <trajopt_moveit/trajopt_moveit_plotting.h>

#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

#include <jsoncpp/json/json.h>

using namespace trajopt;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string TRAJOPT_DESCRIPTION_PARAM =
    "trajopt_description"; /**< Default ROS parameter for trajopt description */

bool plotting_ = false;
int steps_ = 5;
std::string method_ = "json";
robot_model_loader::RobotModelLoaderPtr loader_;  /**< Used to load the robot model */
moveit::core::RobotModelPtr robot_model_;         /**< Robot model */
planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene for the current robot model */
trajopt_moveit::TrajOptMoveItEnvPtr env_;         /**< Trajopt Basic Environment */

TrajOptProbPtr jsonMethod()
{
  ros::NodeHandle nh;
  std::string trajopt_config;

  nh.getParam(TRAJOPT_DESCRIPTION_PARAM, trajopt_config);

  Json::Value root;
  Json::Reader reader;
  bool parse_success = reader.parse(trajopt_config.c_str(), root);
  if (!parse_success)
  {
    ROS_FATAL("Failed to load trajopt json file from ros parameter");
  }

  return ConstructProblem(root, env_);
}

TrajOptProbPtr cppMethod()
{
  ProblemConstructionInfo pci(env_);

  // Populate Basic Info
  pci.basic_info.n_steps = steps_;
  pci.basic_info.manip = "manipulator";
  pci.basic_info.start_fixed = false;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
  Eigen::VectorXd end_pos;
  end_pos.resize(pci.kin->numJoints());
  end_pos << 0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0;

  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = TrajArray(steps_, pci.kin->numJoints());
  for (int idof = 0; idof < pci.kin->numJoints(); ++idof)
  {
    pci.init_info.data.col(idof) = VectorXd::LinSpaced(steps_, start_pos[idof], end_pos[idof]);
  }

  // Populate Cost Info
  std::shared_ptr<JointVelCostInfo> jv = std::shared_ptr<JointVelCostInfo>(new JointVelCostInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->name = "joint_vel";
  jv->term_type = TT_COST;
  pci.cost_infos.push_back(jv);

  std::shared_ptr<CollisionCostInfo> collision = std::shared_ptr<CollisionCostInfo>(new CollisionCostInfo);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
  pci.cost_infos.push_back(collision);

  // Populate Constraints
  double delta = 0.5 / pci.basic_info.n_steps;
  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    std::shared_ptr<PoseCostInfo> pose = std::shared_ptr<PoseCostInfo>(new PoseCostInfo);
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "tool0";
    pose->timestep = i;
    pose->xyz = Eigen::Vector3d(0.5, -0.2 + delta * i, 0.62);
    pose->wxyz = Eigen::Vector4d(0.0, 0.0, 1.0, 0.0);
    if (i == (pci.basic_info.n_steps - 1) || i == 0)
    {
      pose->pos_coeffs = Eigen::Vector3d(10, 10, 10);
      pose->rot_coeffs = Eigen::Vector3d(10, 10, 10);
    }
    else
    {
      pose->pos_coeffs = Eigen::Vector3d(0, 0, 0);
      pose->rot_coeffs = Eigen::Vector3d(10, 10, 0);
    }
    pci.cnt_infos.push_back(pose);
  }

  return ConstructProblem(pci);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "glass_up_right_plan");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  // Initial setup
  loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
  robot_model_ = loader_->getModel();
  env_ = trajopt_moveit::TrajOptMoveItEnvPtr(new trajopt_moveit::TrajOptMoveItEnv);
  planning_scene_.reset(new planning_scene::PlanningScene(robot_model_));
  assert(robot_model_ != nullptr);
  assert(planning_scene_ != nullptr);

  // Now assign collision detection plugin
  bool success;
  collision_detection::CollisionPluginLoader cd_loader;
  std::string class_name = "BULLET";

  success = cd_loader.activate(class_name, planning_scene_, true);
  assert(success);

  success = env_->init(planning_scene_);
  assert(success);

  // Create plotting tool
  trajopt_moveit::TrajoptMoveItPlottingPtr plotter(new trajopt_moveit::TrajoptMoveItPlotting(planning_scene_));

  // Add sphere
  moveit_msgs::CollisionObject sphere_world;
  geometry_msgs::Pose sphere_pose;
  shape_msgs::SolidPrimitive sphere;

  sphere.type = shape_msgs::SolidPrimitive::SPHERE;
  sphere.dimensions.resize(1);
  sphere.dimensions[0] = 0.15;

  sphere_pose.position.x = 0.5;
  sphere_pose.position.y = 0;
  sphere_pose.position.z = 0.55;
  sphere_pose.orientation.x = 0;
  sphere_pose.orientation.y = 0;
  sphere_pose.orientation.z = 0;
  sphere_pose.orientation.w = 1;

  sphere_world.header.frame_id = "base_link";
  sphere_world.header.stamp = ros::Time::now();

  sphere_world.id = "box_world";
  sphere_world.operation = moveit_msgs::CollisionObject::ADD;
  sphere_world.primitives.push_back(sphere);
  sphere_world.primitive_poses.push_back(sphere_pose);

  planning_scene_->processCollisionObjectMsg(sphere_world);

  ros::Publisher planning_scene_diff_publisher =
      nh.advertise<moveit_msgs::PlanningScene>("/trajopt/planning_scene", 1, true);

  moveit_msgs::PlanningScene msg;
  planning_scene_->getPlanningSceneMsg(msg);
  planning_scene_diff_publisher.publish(msg);
  ros::Duration(0.25).sleep();

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);
  pnh.param<std::string>("method", method_, method_);
  pnh.param<int>("steps", steps_, steps_);

  // Set the robot initial state
  robot_state::RobotState& rs = planning_scene_->getCurrentStateNonConst();
  std::map<std::string, double> ipos;
  ipos["joint_a1"] = -0.4;
  ipos["joint_a2"] = 0.2762;
  ipos["joint_a3"] = 0.0;
  ipos["joint_a4"] = -1.3348;
  ipos["joint_a5"] = 0.0;
  ipos["joint_a6"] = 1.4959;
  ipos["joint_a7"] = 0.0;
  rs.setVariablePositions(ipos);

  // Set Log Level
  gLogLevel = util::LevelInfo;

  // Setup Problem
  TrajOptProbPtr prob;
  if (method_ == "cpp")
    prob = cppMethod();
  else
    prob = jsonMethod();

  // Solve Trajectory
  ROS_INFO("basic cartesian plan example");

  tesseract::ContactResultVector collisions;
  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

  env_->continuousCollisionCheckTrajectory(joint_names, link_names, prob->GetInitTraj(), collisions);
  ROS_INFO("Initial trajector number of continuous collisions: %lui\n", collisions.size());

  BasicTrustRegionSQP opt(prob);
  if (plotting_)
  {
    opt.addCallback(PlotCallback(*prob, plotter));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
  {
    plotter->clear();
  }

  collisions.clear();
  env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
  ROS_INFO("Final trajectory number of continuous collisions: %lui\n", collisions.size());
}
