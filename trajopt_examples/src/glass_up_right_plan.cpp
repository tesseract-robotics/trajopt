#include <ros/ros.h>
#include <tesseract_ros/bullet/bullet_env.h>
#include <tesseract_ros/kdl/kdl_chain_kin.h>
#include <trajopt/problem_description.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/config.hpp>
#include <urdf_parser/urdf_parser.h>
#include <jsoncpp/json/json.h>
#include <srdfdom/model.h>

using namespace trajopt;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot description */
const std::string TRAJOPT_DESCRIPTION_PARAM = "trajopt_description"; /**< Default ROS parameter for trajopt description */

bool plotting_ = false;
int steps_ = 5;
std::string method_ = "json";
urdf::ModelInterfaceSharedPtr model_;  /**< URDF Model */
srdf::ModelSharedPtr srdf_model_;      /**< SRDF Model */
tesseract::BulletEnvPtr env_;   /**< Trajopt Basic Environment */

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
  boost::shared_ptr<JointVelCostInfo> jv = boost::shared_ptr<JointVelCostInfo>(new JointVelCostInfo);
  jv->coeffs = std::vector<double>(7, 1.0);
  jv->name = "joint_vel";
  jv->term_type = TT_COST;
  pci.cost_infos.push_back(jv);

  boost::shared_ptr<CollisionCostInfo> collision = boost::shared_ptr<CollisionCostInfo>(new CollisionCostInfo);
  collision->name = "collision";
  collision->term_type = TT_COST;
  collision->continuous = false;
  collision->first_step = 0;
  collision->last_step = pci.basic_info.n_steps - 1;
  collision->gap = 1;
  collision->info = createSafetyMarginDataVector(pci.basic_info.n_steps, 0.025, 20);
  for (auto& info : collision->info)
  {
    info->SetPairSafetyMarginData("base_link", "link_5", 0.05, 10);
    info->SetPairSafetyMarginData("link_3", "link_5", 0.01, 10);
    info->SetPairSafetyMarginData("link_3", "link_6", 0.01, 10);
  }
  pci.cost_infos.push_back(collision);

  // Populate Constraints
  double delta = 0.5/pci.basic_info.n_steps;
  for (auto i = 0; i < pci.basic_info.n_steps; ++i)
  {
    boost::shared_ptr<PoseCostInfo> pose = boost::shared_ptr<PoseCostInfo>(new PoseCostInfo);
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
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);
  model_ = urdf::parseURDF(urdf_xml_string);

  srdf_model_ = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model_->initString(*model_, srdf_xml_string);
  env_ = tesseract::BulletEnvPtr(new tesseract::BulletEnv);
  assert(model_ != nullptr);
  assert(env_ != nullptr);

  bool success = env_->init(model_, srdf_model_);
  assert(success);

  // Add sphere
  tesseract::AttachableObjectPtr obj(new tesseract::AttachableObject());
  shapes::Sphere* sphere = new shapes::Sphere();
  Eigen::Affine3d sphere_pose;

  sphere->radius = 0.15;

  sphere_pose.setIdentity();
  sphere_pose.translation() = Eigen::Vector3d(0.5, 0, 0.55);

  obj->name = "sphere_attached";
  obj->shapes.push_back(shapes::ShapeConstPtr(sphere));
  obj->shapes_trans.push_back(sphere_pose);
  env_->addAttachableObject(obj);

  tesseract::AttachedBodyInfo attached_body;
  attached_body.name = "attached_body";
  attached_body.object_name = "sphere_attached";
  attached_body.parent_link_name = "base_link";

  env_->attachBody(attached_body);

  // Get ROS Parameters
  pnh.param("plotting", plotting_, plotting_);
  pnh.param<std::string>("method", method_, method_);
  pnh.param<int>("steps", steps_, steps_);

  // Set the robot initial state
  std::map<std::string, double> ipos;
  ipos["joint_a1"] = -0.4;
  ipos["joint_a2"] = 0.2762;
  ipos["joint_a3"] = 0.0;
  ipos["joint_a4"] = -1.3348;
  ipos["joint_a5"] = 0.0;
  ipos["joint_a6"] = 1.4959;
  ipos["joint_a7"] = 0.0;
  env_->setState(ipos);

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

  tesseract::DistanceResultVector collisions;
  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

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

  collisions.clear();
  env_->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);
  ROS_INFO("Final trajectory number of continuous collisions: %lui\n", collisions.size());

}
