#ifndef ROS_BASIC_PLOTTING_H
#define ROS_BASIC_PLOTTING_H

#include <tesseract_core/basic_plotting.h>
#include <tesseract_ros/ros_tesseract_utils.h>
#include <tesseract_msgs/Trajectory.h>
#include <tesseract_msgs/TesseractState.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/publisher.h>

namespace tesseract
{
namespace tesseract_ros
{

/** @brief The BasicPlotting class */
class ROSBasicPlotting : public BasicPlotting
{
public:

  ROSBasicPlotting(ROSBasicEnvConstPtr env) : env_(env)
  {
    ros::NodeHandle nh;

    scene_pub_ = nh.advertise<tesseract_msgs::TesseractState>("/trajopt/display_tesseract_state", 1, true);
    trajectory_pub_ = nh.advertise<tesseract_msgs::Trajectory>("/trajopt/display_tesseract_trajectory", 1, true);
    collisions_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_collisions", 1, true);
    arrows_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_arrows", 1, true);
    axes_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/trajopt/display_axes", 1, true);
  }

  void plotScene() const
  {
    tesseract_msgs::TesseractState msg;
    tesseract_ros::tesseractToTesseractStateMsg(msg, *env_);

    scene_pub_.publish(msg);
  }

  void plotTrajectory(const std::vector<std::string> &joint_names, const TrajArray &traj)
  {
    tesseract_msgs::Trajectory msg;

    // Set the model id
    msg.model_id = env_->getURDF()->getName();

    // Set the Robot State so attached objects show up
    tesseract_ros::tesseractToTesseractStateMsg(msg.trajectory_start, *env_);

    // Initialze the whole traject with the current state.
    const EnvStateConstPtr& current_state = env_->getState();
    std::map<std::string, int> jn_to_index;

    msg.joint_trajectory.joint_names.resize(current_state->joints.size());
    msg.joint_trajectory.points.resize(traj.rows());
    for (int i = 0; i < traj.rows(); ++i)
    {
      trajectory_msgs::JointTrajectoryPoint jtp;
      jtp.positions.resize(current_state->joints.size());

      int j = 0;
      for (const auto& joint : current_state->joints)
      {
        if (i == 0)
        {
          msg.joint_trajectory.joint_names[j] = joint.first;
          jn_to_index[joint.first] = j;
        }
        jtp.positions[j] =  joint.second;

        ++j;
      }
      jtp.time_from_start = ros::Duration(i);
      msg.joint_trajectory.points[i] = jtp;
    }

    // Update only the joints which were provided.
    for (int i = 0; i < traj.rows(); ++i)
    {
      for (int j = 0; j < traj.cols(); ++j)
      {
        msg.joint_trajectory.points[i].positions[jn_to_index[joint_names[j]]] = traj(i, j);
      }
    }
    trajectory_pub_.publish(msg);
  }

  void plotContactResults(const std::vector<std::string> &link_names, const DistanceResultVector &dist_results, const Eigen::VectorXd& safety_distances)
  {
    visualization_msgs::MarkerArray msg;
    for (int i = 0; i < dist_results.size(); ++i)
    {
      const DistanceResult &dist = dist_results[i];
      const double& safety_distance = safety_distances[i];

      if (!dist.valid)
        continue;

      Eigen::Vector4d rgba;
      if (dist.distance < 0)
      {
        rgba << 1.0, 0.0, 0.0, 1.0;
      }
      else if (dist.distance < safety_distance)
      {
        rgba << 1.0, 1.0, 0.0, 1.0;
      }
      else
      {
        rgba << 0.0, 1.0, 0.0, 1.0;
      }

      Eigen::Vector3d ptA, ptB;
      ptA = dist.nearest_points[0];
      ptB = dist.nearest_points[1];

      auto it = std::find(link_names.begin(), link_names.end(), dist.link_names[0]);
      if (it != link_names.end())
      {
        ptA = dist.nearest_points[1];
        ptB = dist.nearest_points[0];
      }

      if(dist.cc_type == ContinouseCollisionType::CCType_Between)
      {
        Eigen::Vector4d cc_rgba;
        cc_rgba << 0.0, 0.0, 0.0, 1.0;
        msg.markers.push_back(getMarkerArrowMsg(ptB, dist.cc_nearest_points[1], cc_rgba, 0.01));

        // DEGUG: This was added to see what the original contact point was for the cast continuous
        //        collision checking. Should be removed as everything has been integrated and tested.
        Eigen::Vector4d temp_rgba;
        temp_rgba << 0.0, 0.0, 1.0, 1.0;
        msg.markers.push_back(getMarkerArrowMsg(ptA, dist.cc_nearest_points[0], temp_rgba, 0.01));

        ptB = ((1 - dist.cc_time) * ptB + dist.cc_time * dist.cc_nearest_points[1]);

      }

       msg.markers.push_back(getMarkerArrowMsg(ptA, ptB, rgba, 0.01));
    }

    if (dist_results.size() > 0)
    {
      collisions_pub_.publish(msg);
    }
  }

  void plotArrow(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale)
  {
    visualization_msgs::MarkerArray msg;
    msg.markers.push_back(getMarkerArrowMsg(pt1, pt2, rgba, scale));
    arrows_pub_.publish(msg);
  }

  void plotAxis(const Eigen::Affine3d &axis, double scale)
  {
    visualization_msgs::MarkerArray msg;
    Eigen::Vector3d x_axis = axis.matrix().block<3, 1>(0, 0);
    Eigen::Vector3d y_axis = axis.matrix().block<3, 1>(0, 1);
    Eigen::Vector3d z_axis = axis.matrix().block<3, 1>(0, 2);
    Eigen::Vector3d position = axis.matrix().block<3, 1>(0, 3);

    msg.markers.push_back(getMarkerCylinderMsg(position, position + 0.1 * x_axis, Eigen::Vector4d(1, 0, 0, 1), scale));
    msg.markers.push_back(getMarkerCylinderMsg(position, position + 0.1 * y_axis, Eigen::Vector4d(0, 1, 0, 1), scale));
    msg.markers.push_back(getMarkerCylinderMsg(position, position + 0.1 * z_axis, Eigen::Vector4d(0, 0, 1, 1), scale));
    axes_pub_.publish(msg);
  }

  void clear()
  {
    // Remove old arrows
    marker_counter_ = 0;
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.header.frame_id = env_->getURDF()->getRoot()->name;
    marker.header.stamp = ros::Time();
    marker.ns = "trajopt";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    collisions_pub_.publish(msg);
    arrows_pub_.publish(msg);
    axes_pub_.publish(msg);

    ros::Duration(0.5).sleep();
  }

  void waitForInput()
  {
    ROS_ERROR("Hit enter key to step optimization!");
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n');
  }

private:
  ROSBasicEnvConstPtr env_;                               /**< The ROS Env */
  int marker_counter_;                                    /**< Counter when plotting */
  ros::Publisher scene_pub_;                              /**< Scene publisher */
  ros::Publisher trajectory_pub_;                         /**< Trajectory publisher */
  ros::Publisher collisions_pub_;                         /**< Collision Data publisher */
  ros::Publisher arrows_pub_;                             /**< Used for publishing arrow markers */
  ros::Publisher axes_pub_;                               /**< Used for publishing axis markers */

  visualization_msgs::Marker getMarkerArrowMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = env_->getURDF()->getRoot()->name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajopt";
    marker.id = ++marker_counter_;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d x, y, z;
    x = (pt2 - pt1).normalized();
    marker.pose.position.x = pt1(0);
    marker.pose.position.y = pt1(1);
    marker.pose.position.z = pt1(2);

    y = x.unitOrthogonal();
    z = (x.cross(y)).normalized();
    Eigen::Matrix3d rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    Eigen::Quaterniond q(rot);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = std::abs((pt2 - pt1).norm());
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = rgba(0);
    marker.color.g = rgba(1);
    marker.color.b = rgba(2);
    marker.color.a = rgba(3);

    return marker;
  }

  visualization_msgs::Marker getMarkerCylinderMsg(const Eigen::Vector3d &pt1, const Eigen::Vector3d &pt2, const Eigen::Vector4d &rgba, double scale)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = env_->getURDF()->getRoot()->name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajopt";
    marker.id = ++marker_counter_;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;

    Eigen::Vector3d x, y, z;
    x = (pt2 - pt1).normalized();
    marker.pose.position.x = pt1(0);
    marker.pose.position.y = pt1(1);
    marker.pose.position.z = pt1(2);

    y = x.unitOrthogonal();
    z = (x.cross(y)).normalized();
    Eigen::Matrix3d rot;
    rot.col(0) = x;
    rot.col(1) = y;
    rot.col(2) = z;
    Eigen::Quaterniond q(rot);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    double length = std::abs((pt2 - pt1).norm());
    marker.scale.x = scale * length/20.0;
    marker.scale.y = scale * length/20.0;
    marker.scale.z = scale * length;

    marker.color.r = rgba(0);
    marker.color.g = rgba(1);
    marker.color.b = rgba(2);
    marker.color.a = rgba(3);

    return marker;
  }
};
typedef boost::shared_ptr<ROSBasicPlotting> ROSBasicPlottingPtr;
typedef boost::shared_ptr<const ROSBasicPlotting> ROSBasicPlottingConstPtr;
}
}
#endif // ROS_BASIC_PLOTTING_H
