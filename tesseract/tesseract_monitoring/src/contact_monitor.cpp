#include <ros/ros.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_collision/contact_checker_base.h>
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

const double DEFAULT_CONTACT_DISTANCE = 0.1;

KDLEnvPtr env;
ros::Subscriber joint_states_sub;
ros::Publisher contact_results_pub;
ros::Publisher environment_pub;
ros::ServiceServer modify_env_service;
ContactResultMap contacts;
tesseract_msgs::ContactResultVector contacts_msg;
bool publish_environment;
boost::mutex modify_mutex;

void callbackJointState(const sensor_msgs::JointState::ConstPtr& msg)
{
  boost::mutex::scoped_lock(modify_mutex);
  contacts.clear();
  contacts_msg.constacts.clear();

  env->setState(msg->name, msg->position);
  env->calcDistancesDiscrete(contacts);

  if (publish_environment)
  {
    tesseract_msgs::TesseractState state_msg;
    tesseract_ros::tesseractToTesseractStateMsg(state_msg, *env);
    environment_pub.publish(state_msg);
  }

  ContactResultVector contacts_vector;
  tesseract::moveContactResultsMapToContactResultsVector(contacts, contacts_vector);
  contacts_msg.constacts.reserve(contacts_vector.size());
  for (const auto& contact : contacts_vector)
  {
   tesseract_msgs::ContactResult msg;
   tesseractContactResultToContactResultMsg(msg, contact);
   contacts_msg.constacts.push_back(msg);
  }
  contact_results_pub.publish(contacts_msg);
}

bool callbackModifyTesseractEnv(tesseract_msgs::ModifyTesseractEnvRequest& request, tesseract_msgs::ModifyTesseractEnvResponse& response)
{
  boost::mutex::scoped_lock(modify_mutex);
  response.success = processTesseractStateMsg(*env, request.state);
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_contact_monitoring");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  urdf::ModelInterfaceSharedPtr urdf_model;
  srdf::ModelSharedPtr srdf_model;
  std::string robot_description;
  std::string plugin;

  env.reset(new KDLEnv());

  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<bool>("publish_environment", publish_environment, false);
  if (pnh.hasParam("plugin"))
  {
    pnh.getParam("plugin", plugin);
    env->loadContactCheckerPlugin(plugin);
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

  // Setup request information
  ContactRequest req;
  req.isContactAllowed = env->getIsContactAllowedFn();
  pnh.param<double>("contact_distance", req.contact_distance, DEFAULT_CONTACT_DISTANCE);
  pnh.getParam("monitor_links", req.link_names);
  if (req.link_names.empty())
    req.link_names = env->getLinkNames();

  req.type = ContactRequestTypes::ALL;
  env->setContactRequest(req);

  joint_states_sub = nh.subscribe("joint_states", 1, &callbackJointState);
  contact_results_pub = pnh.advertise<tesseract_msgs::ContactResultVector>("contact_results", 1, true);
  modify_env_service = pnh.advertiseService<tesseract_msgs::ModifyTesseractEnvRequest, tesseract_msgs::ModifyTesseractEnvResponse>("modify_tesseract_env", &callbackModifyTesseractEnv);

  if (publish_environment)
    environment_pub = pnh.advertise<tesseract_msgs::TesseractState>("tesseract", 100, false);

  ros::spin();

  return 0;
}
