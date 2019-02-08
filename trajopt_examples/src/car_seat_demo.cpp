/**
 * @file car_seat_demo.cpp
 * @brief Example using Trajopt for installing car seat
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
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <jsoncpp/json/json.h>
#include <ros/ros.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/logging.hpp>

using namespace trajopt;
using namespace tesseract;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot
                                                                          description */
const std::string TRAJOPT_DESCRIPTION_PARAM =
    "trajopt_description"; /**< Default ROS parameter for trajopt description */

static bool plotting_ = false;
static std::string method_ = "json";
static urdf::ModelInterfaceSharedPtr urdf_model_; /**< URDF Model */
static srdf::ModelSharedPtr srdf_model_;          /**< SRDF Model */
static tesseract_ros::KDLEnvPtr env_;             /**< Trajopt Basic Environment */

std::unordered_map<std::string, std::unordered_map<std::string, double>> saved_positions_;

void addSeats()
{
  for (int i = 0; i < 3; ++i)
  {
    AttachableObjectPtr obj(new AttachableObject());

    obj->name = "seat_" + std::to_string(i + 1);
    std::shared_ptr<shapes::Mesh> visual_mesh(
        shapes::createMeshFromResource("package://trajopt_examples/meshes/car_seat/visual/seat.dae"));
    Eigen::Isometry3d seat_pose;
    seat_pose.setIdentity();

    obj->visual.shapes.push_back(visual_mesh);
    obj->visual.shape_poses.push_back(seat_pose);

    for (auto i = 1; i <= 10; ++i)
    {
      std::shared_ptr<shapes::Mesh> collision_mesh(shapes::createMeshFromResource(
          "package://trajopt_examples/meshes/car_seat/collision/seat_" + std::to_string(i) + ".stl"));

      obj->collision.shapes.push_back(collision_mesh);
      obj->collision.shape_poses.push_back(seat_pose);
      obj->collision.collision_object_types.push_back(CollisionObjectType::ConvexHull);
    }

    env_->addAttachableObject(obj);

    AttachedBodyInfo attached_body;
    attached_body.object_name = "seat_" + std::to_string(i + 1);
    attached_body.parent_link_name = "world";
    attached_body.transform = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                              Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(3.14159, Eigen::Vector3d::UnitZ());
    attached_body.transform.translation() = Eigen::Vector3d(0.5 + i, 2.15, 0.45);

    env_->attachBody(attached_body);
  }
}

// void addCar()
//{
//  AttachableObjectPtr obj(new AttachableObject());
//  std::shared_ptr<shapes::Mesh>
//  visual_mesh(shapes::createMeshFromResource("package://trajopt_examples/meshes/car_seat/visual/car.dae"));
//  std::shared_ptr<shapes::Mesh>
//  collision_mesh(shapes::createMeshFromResource("package://trajopt_examples/meshes/car_seat/collision/car.stl"));
//  Eigen::Isometry3d seat_pose;
//  seat_pose.setIdentity();

//  obj->name = "car";
//  obj->visual.shapes.push_back(visual_mesh);
//  obj->visual.shape_poses.push_back(seat_pose);
//  obj->collision.shapes.push_back(collision_mesh);
//  obj->collision.shape_poses.push_back(seat_pose);
//  obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);

//  env_->addAttachableObject(obj);

//  AttachedBodyInfo attached_body;
//  attached_body.object_name = obj->name;
//  attached_body.parent_link_name = "world";
//  attached_body.transform.setIdentity();
//  attached_body.transform.translation() = Eigen::Vector3d(0.0, -0.2, 0.0);
//  attached_body.touch_links = {"base_link", "carriage", "cell_logo",
//  "conveyor", "fence", "link_l", "link_s", "rail"};

//  env_->attachBody(attached_body);
//}

std::unordered_map<std::string, std::unordered_map<std::string, double>> getPredefinedPosition()
{
  std::unordered_map<std::string, std::unordered_map<std::string, double>> result;

  std::unordered_map<std::string, double> default_pos;
  default_pos["carriage_rail"] = 1.0;
  default_pos["joint_b"] = 0.0;
  default_pos["joint_e"] = 0.0;
  default_pos["joint_l"] = 0.0;
  default_pos["joint_r"] = 0.0;
  default_pos["joint_s"] = -1.5707;
  default_pos["joint_t"] = 0.0;
  default_pos["joint_u"] = -1.5707;
  result["Default"] = default_pos;

  std::unordered_map<std::string, double> pick1;
  pick1["carriage_rail"] = 2.22;
  pick1["joint_b"] = 0.45;
  pick1["joint_e"] = 0.0;
  pick1["joint_l"] = 0.53;
  pick1["joint_r"] = 0.0;
  pick1["joint_s"] = -3.14;
  pick1["joint_t"] = -0.29;
  pick1["joint_u"] = -1.49;
  result["Pick1"] = pick1;

  std::unordered_map<std::string, double> pick2;
  pick2["carriage_rail"] = 1.22;
  pick2["joint_b"] = 0.45;
  pick2["joint_e"] = 0.0;
  pick2["joint_l"] = 0.53;
  pick2["joint_r"] = 0.0;
  pick2["joint_s"] = -3.14;
  pick2["joint_t"] = -0.29;
  pick2["joint_u"] = -1.49;
  result["Pick2"] = pick2;

  std::unordered_map<std::string, double> pick3;
  pick3["carriage_rail"] = 0.22;
  pick3["joint_b"] = 0.45;
  pick3["joint_e"] = 0.0;
  pick3["joint_l"] = 0.53;
  pick3["joint_r"] = 0.0;
  pick3["joint_s"] = -3.14;
  pick3["joint_t"] = -0.29;
  pick3["joint_u"] = -1.49;
  result["Pick3"] = pick3;

  std::unordered_map<std::string, double> place1;
  place1["carriage_rail"] = 4.10529;
  place1["joint_b"] = 0.608497;
  place1["joint_e"] = 0.0167816;
  place1["joint_l"] = 0.869957;
  place1["joint_r"] = 0.0502274;
  place1["joint_s"] = -0.0394713;
  place1["joint_t"] = -0.318406;
  place1["joint_u"] = -1.30834;
  result["Place1"] = place1;

  std::unordered_map<std::string, double> home;
  home["carriage_rail"] = 0.0;
  home["joint_b"] = 0.0;
  home["joint_e"] = 0.0;
  home["joint_l"] = 0.0;
  home["joint_r"] = 0.0;
  home["joint_s"] = 0.0;
  home["joint_t"] = 0.0;
  home["joint_u"] = 0.0;
  result["Home"] = home;

  return result;
}

std::vector<double> getPositionVector(const BasicKinConstPtr& kin, const std::unordered_map<std::string, double>& pos)
{
  std::vector<double> result;
  for (const auto& joint_name : kin->getJointNames())
    result.push_back(pos.at(joint_name));

  return result;
}

Eigen::VectorXd getPositionVectorXd(const BasicKinConstPtr& kin, const std::unordered_map<std::string, double>& pos)
{
  Eigen::VectorXd result;
  result.resize(kin->numJoints());
  int cnt = 0;
  for (const auto& joint_name : kin->getJointNames())
    result[cnt++] = pos.at(joint_name);

  return result;
}

std::shared_ptr<ProblemConstructionInfo> cppMethod(const std::string& start, const std::string& finish)
{
  std::shared_ptr<ProblemConstructionInfo> pci(new ProblemConstructionInfo(env_));

  // Populate Basic Info
  pci->basic_info.n_steps = 50;
  pci->basic_info.manip = "manipulator";
  pci->basic_info.start_fixed = true;
  pci->basic_info.use_time = false;

  pci->opt_info.max_iter = 200;
  pci->opt_info.min_approx_improve = 1e-3;
  pci->opt_info.min_trust_box_size = 1e-3;

  // Create Kinematic Object
  pci->kin = pci->env->getManipulator(pci->basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = getPositionVectorXd(pci->kin, saved_positions_[start]);

  std::vector<double> joint_pose = getPositionVector(pci->kin, saved_positions_[finish]);
  pci->init_info.type = InitInfo::GIVEN_TRAJ;
  pci->init_info.data = start_pos.transpose().replicate(pci->basic_info.n_steps, 1);
  for (auto i = 0; i < start_pos.size(); ++i)
    pci->init_info.data.col(i) = Eigen::VectorXd::LinSpaced(
        static_cast<size_t>(pci->basic_info.n_steps), start_pos[i], joint_pose[static_cast<size_t>(i)]);

  // Populate Cost Info
  std::shared_ptr<JointVelTermInfo> joint_vel = std::shared_ptr<JointVelTermInfo>(new JointVelTermInfo);
  joint_vel->coeffs = std::vector<double>(8, 5.0);
  joint_vel->targets = std::vector<double>(8, 0.0);
  joint_vel->first_step = 0;
  joint_vel->last_step = pci->basic_info.n_steps - 1;
  joint_vel->name = "joint_vel";
  joint_vel->term_type = TT_COST;
  pci->cost_infos.push_back(joint_vel);

  std::shared_ptr<JointAccTermInfo> joint_acc = std::shared_ptr<JointAccTermInfo>(new JointAccTermInfo);
  joint_acc->coeffs = std::vector<double>(8, 5.0);
  joint_acc->targets = std::vector<double>(8, 0.0);
  joint_acc->first_step = 0;
  joint_acc->last_step = pci->basic_info.n_steps - 1;
  joint_acc->name = "joint_acc";
  joint_acc->term_type = TT_COST;
  pci->cost_infos.push_back(joint_acc);

  std::shared_ptr<JointJerkTermInfo> joint_jerk = std::shared_ptr<JointJerkTermInfo>(new JointJerkTermInfo);
  joint_jerk->coeffs = std::vector<double>(8, 5.0);
  joint_jerk->targets = std::vector<double>(8, 0.0);
  joint_jerk->first_step = 0;
  joint_jerk->last_step = pci->basic_info.n_steps - 1;
  joint_jerk->name = "joint_jerk";
  joint_jerk->term_type = TT_COST;
  pci->cost_infos.push_back(joint_jerk);

  std::shared_ptr<CollisionTermInfo> collision = std::shared_ptr<CollisionTermInfo>(new CollisionTermInfo);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = true;
  collision->first_step = 0;
  collision->last_step = pci->basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci->basic_info.n_steps, 0.05, 40);
  pci->cost_infos.push_back(collision);

  // Create place pose constraint
  std::shared_ptr<JointPosTermInfo> jpos(new JointPosTermInfo);
  jpos->term_type = TT_CNT;
  jpos->name = finish;
  jpos->first_step = pci->basic_info.n_steps - 1;
  jpos->last_step = pci->basic_info.n_steps - 1;
  jpos->targets = joint_pose;
  pci->cnt_infos.push_back(jpos);

  return pci;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "car_seat_demo");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  urdf_model_ = urdf::parseURDF(urdf_xml_string);

  srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model_->initString(*urdf_model_, srdf_xml_string);
  env_ = tesseract_ros::KDLEnvPtr(new tesseract_ros::KDLEnv);
  assert(urdf_model_ != nullptr);
  assert(env_ != nullptr);

  bool success = env_->init(urdf_model_, srdf_model_);
  assert(success);

  // Create plotting tool
  tesseract_ros::ROSBasicPlottingPtr plotter(new tesseract_ros::ROSBasicPlotting(env_));

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);

  // Get predefined positions
  saved_positions_ = getPredefinedPosition();

  //  // Add the car as a detailed mesh
  //  addCar();

  // Put three seats on the conveyor
  addSeats();

  // Move to home position
  env_->setState(saved_positions_["Home"]);

  // Plot the scene
  plotter->plotScene();
  plotter->plotScene();

  // Set Log Level
  util::gLogLevel = util::LevelInfo;

  // Solve Trajectory
  ROS_INFO("Car Seat Demo Started");

  // Go pick up first seat
  ros::Time tStart;
  std::shared_ptr<ProblemConstructionInfo> pci;
  TrajOptProbPtr prob;

  pci = cppMethod("Home", "Pick1");
  prob = ConstructProblem(*pci);
  sco::BasicTrustRegionSQP pick1_opt(prob);
  if (plotting_)
    pick1_opt.addCallback(PlotCallback(*prob, plotter));

  pick1_opt.initialize(trajToDblVec(prob->GetInitTraj()));
  tStart = ros::Time::now();
  pick1_opt.optimize();
  ROS_INFO("Pick seat #1 planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  // Plot the trajectory
  plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(pick1_opt.x(), prob->GetVars()));

  std::vector<tesseract::ContactResultMap> collisions;
  ContinuousContactManagerBasePtr manager = prob->GetEnv()->getContinuousContactManager();
  manager->setActiveCollisionObjects(prob->GetKin()->getLinkNames());
  manager->setContactDistanceThreshold(0);

  bool found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), getTraj(pick1_opt.x(), prob->GetVars()), collisions);

  ROS_INFO((found) ? ("Pick seat #1 trajectory is in collision") : ("Pick seat #1 trajectory is collision free"));

  // Get the state of at the end of pick 1 trajectory
  EnvStatePtr state =
      env_->getState(prob->GetKin()->getJointNames(), getPositionVectorXd(prob->GetKin(), saved_positions_["Pick1"]));

  // Now we to detach seat_1 and attach it to the robot end_effector
  AttachedBodyInfo attach_seat1;
  attach_seat1.object_name = "seat_1";
  attach_seat1.parent_link_name = "end_effector";
  attach_seat1.transform = state->transforms["end_effector"].inverse() * state->transforms["seat_1"];
  attach_seat1.touch_links = { "eff_link", "cell_logo", "fence", "link_b", "link_r", "link_t" };

  env_->detachBody(attach_seat1.object_name);
  env_->attachBody(attach_seat1);

  pci = cppMethod("Pick1", "Place1");
  prob = ConstructProblem(*pci);
  sco::BasicTrustRegionSQP place1_opt(prob);
  if (plotting_)
    place1_opt.addCallback(PlotCallback(*prob, plotter));

  place1_opt.initialize(trajToDblVec(prob->GetInitTraj()));
  tStart = ros::Time::now();
  place1_opt.optimize();
  ROS_INFO("Place seat #1 planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
    plotter->clear();

  // Plot the trajectory
  plotter->plotTrajectory(prob->GetKin()->getJointNames(), getTraj(place1_opt.x(), prob->GetVars()));

  collisions.clear();
  found = tesseract::continuousCollisionCheckTrajectory(
      *manager, *prob->GetEnv(), *prob->GetKin(), getTraj(place1_opt.x(), prob->GetVars()), collisions);

  ROS_INFO((found) ? ("Place seat #1 trajectory is in collision") : ("Place seat #1 trajectory is collision free"));
}
// int main(int argc, char **argv)
//{
//  // Set up ROS.
//  ros::init(argc, argv, "rss_demo_node");
//  ros::NodeHandle nh;

//  ros::AsyncSpinner spinner(4);
//  spinner.start();

//  std::vector<moveit_msgs::CollisionObject> collision_objects;
//  MoveGroup group("robot_rail");

//  // Add all three seats to the world environment
//  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
//  planning_scene_monitor::PlanningSceneMonitor
//  planning_scene_monitor("robot_description");
//  planning_scene_monitor.startSceneMonitor("/move_group/monitored_planning_scene");

//  collision_objects = createCollisionObjects();
//  planning_scene_interface.applyCollisionObjects(collision_objects);

//  group.setPlanningTime(30.0);
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Pick1");

//  MoveGroup::Plan plan;
//  MoveItErrorCode e;

//  do {
//    e = group.plan(plan);
//  } while(e != MoveItErrorCode::SUCCESS);

//  group.execute(plan);

//  sleep_t.sleep();
//  std::string id = "seat_1";
//  std::string eff_link = "end_effector";
//  std::vector<std::string> touch_links = {eff_link, "cell_logo", "fence",
//  "link_b", "link_r", "link_t", id};

//  group.attachObject(id, eff_link, touch_links);
//  sleep_t.sleep();

//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Place1");

//  do {
//    e = group.plan(plan);
//  } while(e != MoveItErrorCode::SUCCESS);

//  group.execute(plan);

//  sleep_t.sleep();
//  group.detachObject(id);
//  planning_scene_interface.removeCollisionObjects({id});
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Pick2");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  id = "seat_2";
//  group.attachObject(id, eff_link, touch_links);
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Place1");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  group.detachObject(id);
//  planning_scene_interface.removeCollisionObjects({id});
//  sleep_t.sleep();
//  group.setStartState(*group.getCurrentState());
//  group.setNamedTarget("Pick3");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  id = "seat_3";
//  group.attachObject(id, eff_link, touch_links);
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Place1");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//  sleep_t.sleep();
//  group.detachObject(id);
//  planning_scene_interface.removeCollisionObjects({id});
//  sleep_t.sleep();
//  group.setStartState(planning_scene_monitor.getPlanningScene()->getCurrentState());
//  group.setNamedTarget("Pick2");

//  e = group.plan(plan);
//  if (e = MoveItErrorCode::SUCCESS)
//    group.execute(plan);

//}
