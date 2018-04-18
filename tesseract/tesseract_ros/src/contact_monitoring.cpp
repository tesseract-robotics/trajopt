#include <ros/ros.h>
#include <tesseract_ros/ros_basic_env.h>
#include <tesseract_msgs/ContactResultVector.h>
#include <tesseract_msgs/ModifyTesseractEnv.h>
#include <tesseract_ros/ros_tesseract_utils.h>
#include <pluginlib/class_loader.hpp>
#include <sensor_msgs/JointState.h>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>

using namespace tesseract;
using namespace tesseract::tesseract_ros;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */
const std::string TESSERACT_ENV_SINGLETON_PLUGIN_PARAM = "tesseract_ros/BulletEnvSingleton";

ROSBasicEnvSingletonPtr env;
ros::Subscriber joint_states_sub;
ros::Publisher contact_results_pub;
ros::ServiceServer modify_env_service;
ContactResultVector contacts;
tesseract_msgs::ContactResultVector contacts_msg;
boost::shared_ptr<pluginlib::ClassLoader<ROSBasicEnvSingleton> > env_loader;

void callbackJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
  contacts.clear();
  contacts_msg.constacts.clear();

  env->setState(msg->name, msg->position);
  env->calcDistancesDiscrete(contacts);

  contacts_msg.constacts.reserve(contacts.size());
  for (const auto& contact : contacts)
  {
   tesseract_msgs::ContactResult msg;
   tesseractContactResultToContactResultMsg(msg, contact);
   contacts_msg.constacts.push_back(msg);
  }
  contact_results_pub.publish(contacts_msg);
}

bool callbackModifyTesseractEnv(tesseract_msgs::ModifyTesseractEnvRequest& request, tesseract_msgs::ModifyTesseractEnvResponse& response)
{
  response.success = processTesseractStateMsg(*env, request.state);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_contact_monitoring");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  urdf::ModelInterfaceSharedPtr urdf_model; /**< URDF Model */
  srdf::ModelSharedPtr srdf_model;          /**< SRDF Model */
  std::string robot_description;
  std::string plugin;

  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<std::string>("plugin", plugin, TESSERACT_ENV_SINGLETON_PLUGIN_PARAM);

  env_loader.reset(new pluginlib::ClassLoader<ROSBasicEnvSingleton>("tesseract_ros", "tesseract::tesseract_ros::ROSBasicEnvSingleton"));
  env.reset(env_loader->createUnmanagedInstance(plugin));
  if (env == nullptr)
  {
    ROS_ERROR("Failed to load tesseract environment plugin: %s.", plugin.c_str());
    return 0;
  }

  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam(robot_description, urdf_xml_string);
  nh.getParam(robot_description + "_semantic", srdf_xml_string);

  urdf_model = urdf::parseURDF(urdf_xml_string);
  srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model, srdf_xml_string);
  if (urdf_model == nullptr)
  {
    ROS_ERROR("Failed to parse URDF.");
    return 0;
  }

  if (!env->init(urdf_model, srdf_model))
  {
    ROS_ERROR("Failed to initialize environment.");
    return 0;
  }

  joint_states_sub = nh.subscribe("joint_states", 1, &callbackJointState);
  contact_results_pub = pnh.advertise<tesseract_msgs::ContactResultVector>("contact_results", 1, true);
  modify_env_service = pnh.advertiseService<tesseract_msgs::ModifyTesseractEnvRequest, tesseract_msgs::ModifyTesseractEnvResponse>("modify_tesseract_env", &callbackModifyTesseractEnv);

  ros::spin();

  return 0;
}
