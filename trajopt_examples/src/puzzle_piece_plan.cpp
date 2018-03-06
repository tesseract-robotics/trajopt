#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_plugin_loader/collision_plugin_loader.h>
#include <trajopt/ros_env.h>
#include <trajopt/ros_kin_chain.h>
#include <trajopt/problem_description.hpp>
#include <trajopt/plot_callback.hpp>

#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/config.hpp>

// For loading the pose file from a local package
#include <ros/package.h>
#include <fstream>

using namespace trajopt;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

bool plotting_ = false;
robot_model_loader::RobotModelLoaderPtr loader_;  /**< Used to load the robot model */
moveit::core::RobotModelPtr robot_model_;         /**< Robot model */
planning_scene::PlanningScenePtr planning_scene_; /**< Planning scene for the current robot model */
ROSEnvPtr env_;                                   /**< Trajopt Basic Environment */


static EigenSTL::vector_Affine3d makePuzzleToolPoses()
{
  EigenSTL::vector_Affine3d path; // results
  std::ifstream indata; // input file

  // You could load your parts from anywhere, but we are transporting them with the git repo
  std::string filename = ros::package::getPath("trajopt_examples") + "/config/puzzle_bent.csv";

  // In a non-trivial app, you'll of course want to check that calls like 'open' succeeded
  indata.open(filename);

  std::string line;
  int lnum = 0;
  while (std::getline(indata, line))
  {
      ++lnum;
      if (lnum < 3)
        continue;

      std::stringstream lineStream(line);
      std::string  cell;
      Eigen::Matrix<double, 6, 1> xyzijk;
      int i = -2;
      while (std::getline(lineStream, cell, ','))
      {
        ++i;
        if (i == -1)
          continue;

        xyzijk(i) = std::stod(cell);
      }

      Eigen::Vector3d pos = xyzijk.head<3>();
      pos = pos / 1000.0; // Most things in ROS use meters as the unit of length. Our part was exported in mm.
      Eigen::Vector3d norm = xyzijk.tail<3>();
      norm.normalize();

      // This code computes two extra directions to turn the normal direction into a full defined frame. Descartes
      // will search around this frame for extra poses, so the exact values do not matter as long they are valid.
      Eigen::Vector3d temp_x = (-1 * pos).normalized();
      Eigen::Vector3d y_axis = (norm.cross(temp_x)).normalized();
      Eigen::Vector3d x_axis = (y_axis.cross(norm)).normalized();
      Eigen::Affine3d pose;
      pose.matrix().col(0).head<3>() = x_axis;
      pose.matrix().col(1).head<3>() = y_axis;
      pose.matrix().col(2).head<3>() = norm;
      pose.matrix().col(3).head<3>() = pos;

      path.push_back(pose);
  }
  indata.close();

  return path;
}

TrajOptProbPtr cppMethod()
{
  ProblemConstructionInfo pci(env_);

  EigenSTL::vector_Affine3d tool_poses = makePuzzleToolPoses();

  // Populate Basic Info
  pci.basic_info.n_steps = tool_poses.size();
  pci.basic_info.manip = "manipulator";
  pci.basic_info.start_fixed = false;

  pci.opt_info.max_iter = 200;

  // Create Kinematic Object
  pci.kin = pci.env->getManipulator(pci.basic_info.manip);

  // Populate Init Info
  Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());


  pci.init_info.type = InitInfo::GIVEN_TRAJ;
  pci.init_info.data = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
//  pci.init_info.data.col(6) = VectorXd::LinSpaced(steps_, start_pos[6], end_pos[6]);


  // Populate Cost Info
  boost::shared_ptr<JointVelCostInfo> joint_vel = boost::shared_ptr<JointVelCostInfo>(new JointVelCostInfo);
  joint_vel->coeffs = std::vector<double>(7, 1.0);
  joint_vel->name = "joint_vel";
  joint_vel->term_type = TT_COST;
  pci.cost_infos.push_back(joint_vel);

  boost::shared_ptr<JointAccCostInfo> joint_acc = boost::shared_ptr<JointAccCostInfo>(new JointAccCostInfo);
  joint_acc->coeffs = std::vector<double>(7, 2.0);
  joint_acc->name = "joint_acc";
  joint_acc->term_type = TT_COST;
  pci.cost_infos.push_back(joint_acc);

  boost::shared_ptr<JointJerkCostInfo> joint_jerk = boost::shared_ptr<JointJerkCostInfo>(new JointJerkCostInfo);
  joint_jerk->coeffs = std::vector<double>(7, 5.0);
  joint_jerk->name = "joint_jerk";
  joint_jerk->term_type = TT_COST;
  pci.cost_infos.push_back(joint_jerk);

//  boost::shared_ptr<CollisionCostInfo> collision = boost::shared_ptr<CollisionCostInfo>(new CollisionCostInfo);
//  collision->name = "collision";
//  collision->term_type = TT_COST;
//  collision->continuous = false;
//  collision->first_step = 0;
//  collision->last_step = pci.basic_info.n_steps - 1;
//  collision->gap = 1;
//  collision->coeffs = DblVec(pci.basic_info.n_steps, 20.0);
//  collision->dist_pen = DblVec(pci.basic_info.n_steps, 0.02);
//  pci.cost_infos.push_back(collision);

  // Populate Constraints
  Eigen::Affine3d grinder_frame = env_->getLinkTransform("grinder_frame");
  Eigen::Quaterniond q(grinder_frame.linear());

  Eigen::Vector3d stationary_xyz = grinder_frame.translation();
  Eigen::Vector4d stationary_wxyz = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());

  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    boost::shared_ptr<PoseCostInfo> pose = boost::shared_ptr<PoseCostInfo>(new PoseCostInfo);
    pose->term_type = TT_CNT;
    pose->name = "waypoint_cart_" + std::to_string(i);
    pose->link = "part";
    pose->tcp = tool_poses[i];
    pose->timestep = i;
    pose->xyz = stationary_xyz;
    pose->wxyz = stationary_wxyz;
    pose->pos_coeffs = Eigen::Vector3d(5, 5, 5);
    pose->rot_coeffs = Eigen::Vector3d(5, 5, 0);

    pci.cnt_infos.push_back(pose);
  }

  return ConstructProblem(pci);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "puzzle_piece_plan");
  ros::NodeHandle pnh("~");
  ros::NodeHandle nh;

  // Initial setup
  loader_.reset(new robot_model_loader::RobotModelLoader(ROBOT_DESCRIPTION_PARAM));
  robot_model_ = loader_->getModel();
  env_ = ROSEnvPtr(new ROSEnv);
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

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);

  // Set the robot initial state
  robot_state::RobotState &rs = planning_scene_->getCurrentStateNonConst();
  std::map<std::string, double> ipos;
  ipos["joint_a1"] = -0.785398;
  ipos["joint_a2"] = 0.4;
  ipos["joint_a3"] = 0.0;
  ipos["joint_a4"] = -1.9;
  ipos["joint_a5"] = 0.0;
  ipos["joint_a6"] = 1.0;
  ipos["joint_a7"] = 0.0;
  rs.setVariablePositions(ipos);

  // Set Log Level
  gLogLevel = util::LevelInfo;

  // Setup Problem
  TrajOptProbPtr prob = cppMethod();

  // Solve Trajectory
  ROS_INFO("puzzle piece plan");

  std::vector<trajopt::BasicEnv::DistanceResult> collisions;
  std::vector<std::string> joint_names, link_names;
  prob->GetKin()->getJointNames(joint_names);
  prob->GetKin()->getLinkNames(link_names);

  env_->continuousCollisionCheckTrajectory(joint_names, link_names, prob->GetInitTraj(), collisions);
  ROS_INFO("Initial trajector number of continuous collisions: %lui\n", collisions.size());

  BasicTrustRegionSQP opt(prob);
  if (plotting_)
  {
    opt.addCallback(PlotCallback(*prob));
  }

  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  ros::Time tStart = ros::Time::now();
  opt.optimize();
  ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

  if (plotting_)
  {
    prob->GetEnv()->plotClear();
  }

  // Plot the final trajectory
  env_->plotTrajectory("", joint_names, getTraj(opt.x(), prob->GetVars()));

  collisions.clear();
  env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
  ROS_INFO("Final trajectory number of continuous collisions: %lui\n", collisions.size());

}
