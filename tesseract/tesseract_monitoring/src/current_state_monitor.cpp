/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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

#include <tesseract_ros_monitor/current_state_monitor.h>
#include <limits>

using namespace tesseract;
using namespace tesseract::tesseract_ros;
using namespace tesseract::tesseract_ros::tesseract_ros_monitor;

CurrentStateMonitor::CurrentStateMonitor(const tesseract_ros::ROSBasicEnvConstPtr& env)
  : CurrentStateMonitor(env, ros::NodeHandle())
{
}

CurrentStateMonitor::CurrentStateMonitor(const tesseract_ros::ROSBasicEnvConstPtr& env, ros::NodeHandle nh)
  : nh_(nh)
  , env_(env)
  , state_(*env->getState())
  , state_monitor_started_(false)
  , copy_dynamics_(false)
  , error_(std::numeric_limits<double>::epsilon())
{
//  robot_state_.setToDefaultValues();
}

CurrentStateMonitor::~CurrentStateMonitor()
{
  stopStateMonitor();
}

EnvStatePtr CurrentStateMonitor::getCurrentState() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  return EnvStatePtr(new EnvState(state_));
}

ros::Time CurrentStateMonitor::getCurrentStateTime() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  return current_state_time_;
}

std::pair<EnvStatePtr, ros::Time>
CurrentStateMonitor::getCurrentStateAndTime() const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  return std::make_pair(EnvStatePtr(new EnvState(state_)), current_state_time_);
}

std::unordered_map<std::string, double> CurrentStateMonitor::getCurrentStateValues() const
{
  std::map<std::string, double> m;
  boost::mutex::scoped_lock slock(state_update_lock_);
  return state_.joints;
}

void CurrentStateMonitor::setToCurrentState(EnvState& upd) const
{
  boost::mutex::scoped_lock slock(state_update_lock_);
  upd = state_;
}

void CurrentStateMonitor::addUpdateCallback(const JointStateUpdateCallback& fn)
{
  if (fn)
    update_callbacks_.push_back(fn);
}

void CurrentStateMonitor::clearUpdateCallbacks()
{
  update_callbacks_.clear();
}

void CurrentStateMonitor::startStateMonitor(const std::string& joint_states_topic)
{
  if (!state_monitor_started_ && env_)
  {
    joint_time_.clear();
    if (joint_states_topic.empty())
      ROS_ERROR("The joint states topic cannot be an empty string");
    else
      joint_state_subscriber_ = nh_.subscribe(joint_states_topic, 25, &CurrentStateMonitor::jointStateCallback, this);
    state_monitor_started_ = true;
    monitor_start_time_ = ros::Time::now();
    ROS_DEBUG("Listening to joint states on topic '%s'", nh_.resolveName(joint_states_topic).c_str());
  }
}

bool CurrentStateMonitor::isActive() const
{
  return state_monitor_started_;
}

void CurrentStateMonitor::stopStateMonitor()
{
  if (state_monitor_started_)
  {
    joint_state_subscriber_.shutdown();
    ROS_DEBUG("No longer listening o joint states");
    state_monitor_started_ = false;
  }
}

std::string CurrentStateMonitor::getMonitoredTopic() const
{
  if (joint_state_subscriber_)
    return joint_state_subscriber_.getTopic();
  else
    return "";
}

bool CurrentStateMonitor::isPassiveOrMimicDOF(const std::string& dof) const
{
  // TODO: Levi Need to implement

//  if (robot_model_->hasJointModel(dof))
//  {
//    if (robot_model_->getJointModel(dof)->isPassive() || robot_model_->getJointModel(dof)->getMimic())
//      return true;
//  }
//  else
//  {
//    // check if this DOF is part of a multi-dof passive joint
//    std::size_t slash = dof.find_last_of("/");
//    if (slash != std::string::npos)
//    {
//      std::string joint_name = dof.substr(0, slash);
//      if (robot_model_->hasJointModel(joint_name))
//        if (robot_model_->getJointModel(joint_name)->isPassive() || robot_model_->getJointModel(joint_name)->getMimic())
//          return true;
//    }
//  }
  return false;
}

bool CurrentStateMonitor::haveCompleteState() const
{
  bool result = true;
  boost::mutex::scoped_lock slock(state_update_lock_);
  const std::unordered_map<std::string, double>& joints = state_.joints;
  for (const auto& joint : joints)
    if (joint_time_.find(joint.first) == joint_time_.end())
    {
      if (!isPassiveOrMimicDOF(joint.first))
      {
        ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
        result = false;
      }
    }
  return result;
}

bool CurrentStateMonitor::haveCompleteState(std::vector<std::string>& missing_states) const
{
  bool result = true;
  boost::mutex::scoped_lock slock(state_update_lock_);
  const std::unordered_map<std::string, double>& joints = state_.joints;
  for (const auto& joint : joints)
    if (joint_time_.find(joint.first) == joint_time_.end())
      if (!isPassiveOrMimicDOF(joint.first))
      {
        missing_states.push_back(joint.first);
        result = false;
      }
  return result;
}

bool CurrentStateMonitor::haveCompleteState(const ros::Duration& age) const
{
  bool result = true;
  ros::Time now = ros::Time::now();
  ros::Time old = now - age;
  boost::mutex::scoped_lock slock(state_update_lock_);
  const std::unordered_map<std::string, double>& joints = state_.joints;
  for (const auto& joint : joints)
  {
    if (isPassiveOrMimicDOF(joint.first))
      continue;
    std::map<std::string, ros::Time>::const_iterator it = joint_time_.find(joint.first);
    if (it == joint_time_.end())
    {
      ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
      result = false;
    }
    else if (it->second < old)
    {
      ROS_DEBUG("Joint variable '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                joint.first.c_str(), (now - it->second).toSec(), age.toSec());
      result = false;
    }
  }
  return result;
}

bool CurrentStateMonitor::haveCompleteState(const ros::Duration& age, std::vector<std::string>& missing_states) const
{
  bool result = true;
  ros::Time now = ros::Time::now();
  ros::Time old = now - age;
  boost::mutex::scoped_lock slock(state_update_lock_);

  const std::unordered_map<std::string, double>& joints = state_.joints;
  for (const auto& joint : joints)
  {
    if (isPassiveOrMimicDOF(joint.first))
      continue;
    std::map<std::string, ros::Time>::const_iterator it = joint_time_.find(joint.first);
    if (it == joint_time_.end())
    {
      ROS_DEBUG("Joint variable '%s' has never been updated", joint.first.c_str());
      missing_states.push_back(joint.first);
      result = false;
    }
    else if (it->second < old)
    {
      ROS_DEBUG("Joint variable '%s' was last updated %0.3lf seconds ago (older than the allowed %0.3lf seconds)",
                joint.first.c_str(), (now - it->second).toSec(), age.toSec());
      missing_states.push_back(joint.first);
      result = false;
    }
  }
  return result;
}

bool CurrentStateMonitor::waitForCurrentState(const ros::Time t, double wait_time) const
{
  ros::WallTime start = ros::WallTime::now();
  ros::WallDuration elapsed(0, 0);
  ros::WallDuration timeout(wait_time);

  boost::mutex::scoped_lock slock(state_update_lock_);
  while (current_state_time_ < t)
  {
    state_update_condition_.wait_for(slock, boost::chrono::nanoseconds((timeout - elapsed).toNSec()));
    elapsed = ros::WallTime::now() - start;
    if (elapsed > timeout)
      return false;
  }
  return true;
}

bool CurrentStateMonitor::waitForCompleteState(double wait_time) const
{
  double slept_time = 0.0;
  double sleep_step_s = std::min(0.05, wait_time / 10.0);
  ros::Duration sleep_step(sleep_step_s);
  while (!haveCompleteState() && slept_time < wait_time)
  {
    sleep_step.sleep();
    slept_time += sleep_step_s;
  }
  return haveCompleteState();
}

bool CurrentStateMonitor::waitForCompleteState(const std::string& manip, double wait_time) const
{
  if (waitForCompleteState(wait_time))
    return true;
  bool ok = true;

  // check to see if we have a fully known state for the joints we want to record
  std::vector<std::string> missing_joints;
  if (!haveCompleteState(missing_joints))
  {
    const BasicKinConstPtr jmg = env_->getManipulator(manip);
    if (jmg)
    {
      std::set<std::string> mj;
      mj.insert(missing_joints.begin(), missing_joints.end());
      const std::vector<std::string>& names = jmg->getJointNames();
      bool ok = true;
      for (std::size_t i = 0; ok && i < names.size(); ++i)
        if (mj.find(names[i]) != mj.end())
          ok = false;
    }
    else
      ok = false;
  }
  return ok;
}

void CurrentStateMonitor::jointStateCallback(const sensor_msgs::JointStateConstPtr& joint_state)
{
  if (joint_state->name.size() != joint_state->position.size())
  {
    ROS_ERROR_THROTTLE(1, "State monitor received invalid joint state (number of joint names does not match number of "
                          "positions)");
    return;
  }
  bool update = false;

  {
    boost::mutex::scoped_lock slock(state_update_lock_);
    // read the received values, and update their time stamps
    current_state_time_ = joint_state->header.stamp;
    for (auto i = 0; i < joint_state->name.size(); ++i)
    {
      if (state_.joints.find(joint_state->name[i]) != state_.joints.end())
      {
        double diff = state_.joints[joint_state->name[i]] - joint_state->position[i];
        if (std::fabs(diff) > std::numeric_limits<double>::epsilon())
        {
          state_.joints[joint_state->name[i]] = joint_state->position[i];
          update = true;
        }
      }
    }
  }

  // callbacks, if needed
  if (update)
    for (std::size_t i = 0; i < update_callbacks_.size(); ++i)
      update_callbacks_[i](joint_state);

  // notify waitForCurrentState() *after* potential update callbacks
  state_update_condition_.notify_all();
}
