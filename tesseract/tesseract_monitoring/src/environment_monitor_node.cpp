#include <ros/ros.h>
#include <tesseract_ros/ros_basic_env.h>
#include <tesseract_monitoring/environment_monitor.h>

using namespace tesseract;
using namespace tesseract::tesseract_ros;

const std::string ROBOT_DESCRIPTION_PARAM = "robot_description"; /**< Default ROS parameter for robot description */

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tesseract_environment_monitor");
  ros::NodeHandle pnh("~");
  std::string robot_description;
  std::string plugin;
  std::string joint_state_topic;
  std::string environment_topic;
  std::string monitored_environment_topic;

  pnh.param<std::string>("robot_description", robot_description, ROBOT_DESCRIPTION_PARAM);
  pnh.param<std::string>("plugin", plugin, "");
  pnh.param<std::string>("joint_state_topic", joint_state_topic, "");
  pnh.param<std::string>("environment_topic", environment_topic, "");
  pnh.param<std::string>("monitored_environment_topic", monitored_environment_topic, "");

  tesseract_monitoring::EnvironmentMonitor monitor(robot_description, "", plugin);
  if (environment_topic.empty())
    monitor.startEnvironmentMonitor();
  else
    monitor.startEnvironmentMonitor(environment_topic);

  if (monitored_environment_topic.empty())
    monitor.startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT);
  else
    monitor.startPublishingEnvironment(tesseract_monitoring::EnvironmentMonitor::UPDATE_ENVIRONMENT, monitored_environment_topic);

  if (joint_state_topic.empty())
    monitor.startStateMonitor();
  else
    monitor.startStateMonitor(joint_state_topic);

  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::waitForShutdown();

  return 0;
}
