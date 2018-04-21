/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan */

#include <tesseract_msgs/TesseractState.h>
#include <tesseract_ros/ros_basic_types.h>
#include <tesseract_ros/ros_tesseract_utils.h>
#include <geometric_shapes/solid_primitive_dims.h>

static const std::string ROBOT_DESCRIPTION = "robot_description";
using namespace tesseract;

ros::Publisher pub_env;

void sendSphere()
{
  tesseract_msgs::TesseractState msg;
  msg.is_diff = true;

  // Add sphere
  tesseract_ros::AttachableObject obj;
  std::shared_ptr<shapes::Sphere> sphere(new shapes::Sphere());
  Eigen::Affine3d sphere_pose;

  sphere->radius = 0.15;

  sphere_pose.setIdentity();
  sphere_pose.translation() = Eigen::Vector3d(0.5, 0, 0.55);

  obj.name = "sphere_attached";
  obj.visual.shapes.push_back(sphere);
  obj.visual.shape_poses.push_back(sphere_pose);
  obj.collision.shapes.push_back(sphere);
  obj.collision.shape_poses.push_back(sphere_pose);

  tesseract_msgs::AttachableObject ao_msg;
  tesseract_ros::attachableObjectToAttachableObjectMsg(ao_msg, obj);
  ao_msg.operation = tesseract_msgs::AttachableObject::ADD;
  msg.attachable_objects.push_back(ao_msg);

  tesseract_ros::AttachedBodyInfo attached_body;
  attached_body.name = "attached_body";
  attached_body.object_name = "sphere_attached";
  attached_body.parent_link_name = "base_link";

  tesseract_msgs::AttachedBodyInfo ab_info_msg;
  tesseract_ros::attachedBodyToAttachedBodyInfoMsg(ab_info_msg, attached_body);
  ab_info_msg.operation = tesseract_msgs::AttachableObject::ADD;
  msg.attached_bodies.push_back(ab_info_msg);

  pub_env.publish(msg);
  ros::Duration(1.5).sleep();
  pub_env.publish(msg);

  ROS_INFO("Environment published.");
  ros::Duration(1.5).sleep();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "demo", ros::init_options::AnonymousName);
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  pub_env = nh.advertise<tesseract_msgs::TesseractState>("tesseract", 10);

  sendSphere();

  ros::waitForShutdown();

  return 0;
}
