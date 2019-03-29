#include <tesseract_core/basic_types.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_basic_plotting.h>
#include <tesseract_planning/trajopt/trajopt_freespace_planner.h>
#include <tesseract_planning/trajopt/trajopt_array_planner.h>

#include <urdf_parser/urdf_parser.h>

#include <trajopt_utils/logging.hpp>
#include <memory>

int main(int argc, char** argv)
{
  //////////////////////
  /// INITIALIZATION ///
  //////////////////////

  ros::init(argc, argv, "raster_path_plan");
  ros::NodeHandle nh, pnh("~");

  bool plotting_cb;
  pnh.param<bool>("plotting", plotting_cb, false);

  // Set Log Level
  util::gLogLevel = util::LevelInfo;

  /////////////
  /// SETUP ///
  /////////////

  // Pull ROS params
  std::string urdf_xml_string, srdf_xml_string;
  nh.getParam("robot_description", urdf_xml_string);
  nh.getParam("robot_description_semantic", srdf_xml_string);

  // Initialize the environment
  urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model, srdf_xml_string);
  tesseract::tesseract_ros::KDLEnvPtr env(new tesseract::tesseract_ros::KDLEnv);
  assert(urdf_model != nullptr);
  assert(env != nullptr);
  bool success = env->init(urdf_model, srdf_model);
  assert(success);
  tesseract::tesseract_ros::ROSBasicPlotting plotter(env);

  /////////////////
  /// Freespace ///
  /////////////////

  // The freespace planner plans between 2 points with a specified number of points in the middle
  ROS_ERROR("Press enter to continue to freespace planning");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  {
    // Create the planner and the responses that will store the results
    tesseract::tesseract_planning::TrajOptFreespacePlanner planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;

    // Set the parameters (Most are being left as defaults)
    tesseract::tesseract_planning::TrajOptFreespacePlannerConfig config;
    // These are required
    config.link_ = "iiwa_link_ee";
    config.manipulator_ = "Manipulator";
    config.kin_ = env->getManipulator(config.manipulator_);
    config.env_ = env;

    // Specify a JointWaypoint as the start (could use any type)
    tesseract::tesseract_planning::JointWaypointPtr start_waypoint =
        std::make_shared<tesseract::tesseract_planning::JointWaypoint>();
    Eigen::VectorXd joint_positions(7);
    joint_positions << 0, 0, 0, -1.57, 0, 0, 0;
    start_waypoint->joint_positions_ = joint_positions;
    auto type = start_waypoint->getType();
    config.start_waypoint_ = start_waypoint;

    // Specify a CartesianWaypoint as the finish (could use any type)
    tesseract::tesseract_planning::CartesianWaypointPtr end_waypoint =
        std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
    end_waypoint->cartesian_position_.translation() = Eigen::Vector3d(-.20, .4, 0.8);
    end_waypoint->cartesian_position_.linear() = Eigen::Quaterniond(0, 0, 1.0, 0).toRotationMatrix();
    config.end_waypoint_ = end_waypoint;

    // Flag to plot each iteration (mostly for debugging)
    config.plot_callback_ = plotting_cb;

    // Solve problem. Results are stored in the response
    planner.solve(planning_response, config);

    // Plot the resulting trajectory
    plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory.leftCols(env->getJointNames().size()));
    std::cout << "Resulting Trajectory (timesteps x joints):\n" << planning_response.trajectory << '\n';
  }
  //////////////////////
  /// Array planning ///
  //////////////////////

  // The array planner plans along a continuous string of waypoints.
  ROS_ERROR("Press enter to continue to array planning");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  {
    // Create the planner and the responses that will store the results
    tesseract::tesseract_planning::TrajOptArrayPlanner planner;
    tesseract::tesseract_planning::PlannerResponse planning_response;

    // Set the parameters (Most are being left as defaults)
    tesseract::tesseract_planning::TrajOptArrayPlannerConfig config;
    // These are required
    config.link_ = "iiwa_link_ee";
    config.manipulator_ = "Manipulator";
    config.kin_ = env->getManipulator(config.manipulator_);
    config.env_ = env;

    // These specify the series of points to be optimized
    for (int ind = 0; ind < 20; ind++)
    {
      tesseract::tesseract_planning::CartesianWaypointPtr waypoint =
          std::make_shared<tesseract::tesseract_planning::CartesianWaypoint>();
      waypoint->cartesian_position_.translation() = Eigen::Vector3d(-0.2 + ind * .02, 0.4, 0.8);
      waypoint->cartesian_position_.linear() = Eigen::Quaterniond(0, 1.0, 0, 0).toRotationMatrix();
      config.target_waypoints_.push_back(waypoint);

      // Plot the waypoints
      plotter.plotAxis(waypoint->cartesian_position_, 0.1);
    }

    config.plot_callback_ = plotting_cb;

    // Solve problem. Results are stored in the response
    planner.solve(planning_response, config);

    // Plot the resulting trajectory
    plotter.plotTrajectory(env->getJointNames(), planning_response.trajectory.leftCols(env->getJointNames().size()));
    std::cout << "Resulting Trajectory (timesteps x joints):\n" << planning_response.trajectory << '\n';
  }
  ROS_INFO("Done");
  ros::spin();
}
