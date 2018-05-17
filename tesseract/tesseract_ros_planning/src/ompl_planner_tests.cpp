#include <ros/ros.h>

#include "tesseract_ros_planning/ompl/chain_ompl_interface.h"
#include "tesseract_ros_planning/ompl/conversions.h"

#include <ompl/geometric/planners/rrt/RRTConnect.h>
//#include <ompl/geometric/planners/prm/PRM.h>    // These are other options for planners
//#include <ompl/geometric/planners/prm/PRMstar.h>
//#include <ompl/geometric/planners/prm/LazyPRMstar.h>
//#include <ompl/geometric/planners/prm/SPARS.h>

#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <urdf_parser/urdf_parser.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <functional>

static void addSphere(tesseract::tesseract_ros::ROSBasicEnv& env)
{
  // Add sphere
  using namespace tesseract;
  tesseract::AttachableObjectPtr obj (new tesseract::AttachableObject());
  std::shared_ptr<shapes::Box> sphere (new shapes::Box(0.5, 0.5, 0.5));

  Eigen::Affine3d sphere_pose;
  sphere_pose.setIdentity();
  sphere_pose.translation() = Eigen::Vector3d(0.5, 0, 0.55);

  obj->name = "sphere_attached";
  obj->visual.shapes.push_back(sphere);
  obj->visual.shape_poses.push_back(sphere_pose);
  obj->collision.shapes.push_back(sphere);
  obj->collision.shape_poses.push_back(sphere_pose);
  obj->collision.collision_object_types.push_back(CollisionObjectType::UseShapeType);

  env.addAttachableObject(obj);

  tesseract::AttachedBodyInfo attached_body;
  attached_body.object_name = "sphere_attached";
  attached_body.parent_link_name = "base_link";
  attached_body.transform.setIdentity();

  env.attachBody(attached_body);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ompl_planner_tests");
  ros::NodeHandle nh;

  // Step 1: Load the damn robot description
  const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
  const std::string ROBOT_SEMANTIC_PARAM = "robot_description_semantic"; /**< Default ROS parameter for robot description */

  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(ROBOT_DESCRIPTION_PARAM, urdf_xml_string);
  nh.getParam(ROBOT_SEMANTIC_PARAM, srdf_xml_string);

  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;

  urdf_model = urdf::parseURDF(urdf_xml_string);
  srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model, srdf_xml_string);

  // Step 2: Create a "tesseract" environment
  tesseract::tesseract_ros::KDLEnvPtr env (new tesseract::tesseract_ros::KDLEnv);
  env->init(urdf_model, srdf_model);
  addSphere(*env);

  // A tesseract plotter makes generating and publishing visualization messages easy
  tesseract::tesseract_ros::ROSBasicPlottingPtr plotter = std::make_shared<tesseract::tesseract_ros::ROSBasicPlotting>(env);

  // Step 3: Create a planning context for OMPL - this sets up the OMPL state environment for your given chain
  tesseract_ros_planning::ChainOmplInterface ompl_context (env, "manipulator");

  // Step 4: Create an OMPL planner that we want to use
  ompl::base::PlannerPtr planner (new ompl::geometric::RRTConnect(ompl_context.spaceInformation()));

  // Step 5: Create a start and terminal state for the robot to move between
  tesseract_ros_planning::OmplPlanParameters params;
  std::vector<double> start {-1.2, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0};
  std::vector<double> goal = {1.2, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0};

  // Step 6: Call plan. This returns an optional which is set if the plan succeeded
  auto maybe_path = ompl_context.plan(planner, start, goal, params);

  // Plot results
  if (maybe_path)
  {
    const ompl::geometric::PathGeometric& path = *maybe_path;
    const auto& names = env->getManipulator("manipulator")->getJointNames();
    plotter->plotTrajectory(names, tesseract_ros_planning::toTrajArray(path));
  }
  else
  {
    ROS_WARN_STREAM("Planning failed");
  }

  ROS_INFO("Waiting for user shutdown");
  ros::spin();
}
