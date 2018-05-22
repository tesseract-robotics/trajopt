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

#include <tesseract_ros_monitor/environment_monitor.h>
#include <tesseract_ros/ros_tesseract_utils.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <dynamic_reconfigure/server.h>
#include <moveit_ros_planning/PlanningSceneMonitorDynamicReconfigureConfig.h>
#include <memory>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>


using namespace tesseract;
using namespace tesseract::tesseract_ros;
using namespace tesseract::tesseract_ros::tesseract_ros_monitor;

static const std::string LOGNAME = "environment_monitor";
static const std::string DEFAULT_TESSERACT_ENV_PLUGIN = "tesseract_ros/BulletEnv";

//class PlanningSceneMonitor::DynamicReconfigureImpl
//{
//public:
//  DynamicReconfigureImpl(PlanningSceneMonitor* owner)
//    : owner_(owner), dynamic_reconfigure_server_(ros::NodeHandle(decideNamespace(owner->getName())))
//  {
//    dynamic_reconfigure_server_.setCallback(
//        boost::bind(&DynamicReconfigureImpl::dynamicReconfigureCallback, this, _1, _2));
//  }

//private:
//  // make sure we do not advertise the same service multiple times, in case we use multiple PlanningSceneMonitor
//  // instances in a process
//  static std::string decideNamespace(const std::string& name)
//  {
//    std::string ns = "~/" + name;
//    std::replace(ns.begin(), ns.end(), ' ', '_');
//    std::transform(ns.begin(), ns.end(), ns.begin(), ::tolower);
//    if (ros::service::exists(ns + "/set_parameters", false))
//    {
//      unsigned int c = 1;
//      while (ros::service::exists(ns + boost::lexical_cast<std::string>(c) + "/set_parameters", false))
//        c++;
//      ns += boost::lexical_cast<std::string>(c);
//    }
//    return ns;
//  }

//  void dynamicReconfigureCallback(PlanningSceneMonitorDynamicReconfigureConfig& config, uint32_t level)
//  {
//    PlanningSceneMonitor::SceneUpdateType event = PlanningSceneMonitor::UPDATE_NONE;
//    if (config.publish_geometry_updates)
//      event = (PlanningSceneMonitor::SceneUpdateType)((int)event | (int)PlanningSceneMonitor::UPDATE_GEOMETRY);
//    if (config.publish_state_updates)
//      event = (PlanningSceneMonitor::SceneUpdateType)((int)event | (int)PlanningSceneMonitor::UPDATE_STATE);
//    if (config.publish_transforms_updates)
//      event = (PlanningSceneMonitor::SceneUpdateType)((int)event | (int)PlanningSceneMonitor::UPDATE_TRANSFORMS);
//    if (config.publish_planning_scene)
//    {
//      owner_->setPlanningScenePublishingFrequency(config.publish_planning_scene_hz);
//      owner_->startPublishingPlanningScene(event);
//    }
//    else
//      owner_->stopPublishingPlanningScene();
//  }

//  PlanningSceneMonitor* owner_;
//  dynamic_reconfigure::Server<PlanningSceneMonitorDynamicReconfigureConfig> dynamic_reconfigure_server_;
//};
//}

const std::string EnvironmentMonitor::DEFAULT_JOINT_STATES_TOPIC =  "joint_states";
const std::string EnvironmentMonitor::DEFAULT_ENVIRONMENT_TOPIC =   "tesseract";
const std::string EnvironmentMonitor::DEFAULT_ENVIRONMENT_SERVICE = "get_tesseract";
const std::string EnvironmentMonitor::MONITORED_ENVIRONMENT_TOPIC = "monitored_tesseract";

EnvironmentMonitor::EnvironmentMonitor(const std::string& robot_description,
                                       const std::string& name,
                                       const std::string& plugin)
  : monitor_name_(name), plugin_name_(plugin), nh_("~")
{
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  root_nh_.getParam(robot_description, urdf_xml_string);
  root_nh_.getParam(robot_description + "_semantic", srdf_xml_string);
  urdf_model_ = urdf::parseURDF(urdf_xml_string);

  srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
  srdf_model->initString(*urdf_model_, srdf_xml_string);
  srdf_model_ = srdf_model;

  initialize(urdf_model_, srdf_model_);
}

EnvironmentMonitor::EnvironmentMonitor(const urdf::ModelInterfaceConstSharedPtr& urdf_model,
                                       const srdf::ModelConstSharedPtr& srdf_model,
                                       const std::string& name,
                                       const std::string& plugin)
  : monitor_name_(name), plugin_name_(plugin), nh_("~"), urdf_model_(urdf_model), srdf_model_(srdf_model)
{
  initialize(urdf_model, srdf_model);
}


EnvironmentMonitor::~EnvironmentMonitor()
{
  stopPublishingEnvironment();
  stopStateMonitor();
  stopEnvironmentMonitor();

//  delete reconfigure_impl_;
  current_state_monitor_.reset();
  env_const_.reset();
  env_.reset();
  urdf_model_.reset();
  srdf_model_.reset();
}

void EnvironmentMonitor::initialize(const urdf::ModelInterfaceConstSharedPtr& urdf_model, const srdf::ModelConstSharedPtr& /*srdf_model*/)
{
  enforce_next_state_update_ = false;

  if (monitor_name_.empty())
    monitor_name_ = "tesseract_monitor";

  if (plugin_name_.empty())
    plugin_name_ = DEFAULT_TESSERACT_ENV_PLUGIN;

  if (!urdf_model)
  {
    ROS_FATAL_NAMED(LOGNAME, "Tried to initalize environment monitor with nullptr to ModelInterfaceConstSharedPtr.");
    return;
  }

  if (urdf_model)
  {
    try
    {
      env_loader.reset(new pluginlib::ClassLoader<ROSBasicEnv>("tesseract_ros", "tesseract::tesseract_ros::ROSBasicEnv"));
      env_.reset(env_loader->createUnmanagedInstance(plugin_name_));
      if (env_ == nullptr)
      {
        ROS_ERROR("Failed to load tesseract environment plugin: %s.", plugin_name_.c_str());
        throw;
      }

      if (!env_->init(urdf_model_, srdf_model_))
      {
        ROS_ERROR("Failed to initialize tesseract environment.");
      }

      env_const_ = env_;
    }
    catch (int& e)
    {
      ROS_ERROR_NAMED(LOGNAME, "Failed to load tesseract environment");
      env_.reset();
      env_const_ = env_;
    }
  }

  publish_environment_frequency_ = 2.0;
  new_environment_update_ = UPDATE_NONE;

  last_update_time_ = last_robot_motion_time_ = ros::Time::now();
  last_robot_state_update_wall_time_ = ros::WallTime::now();
  dt_state_update_ = ros::WallDuration(0.1);

  state_update_pending_ = false;
  state_update_timer_ = nh_.createWallTimer(dt_state_update_, &EnvironmentMonitor::stateUpdateTimerCallback, this,
                                            false,   // not a oneshot timer
                                            false);  // do not start the timer yet

//  reconfigure_impl_ = new DynamicReconfigureImpl(this);
}

void EnvironmentMonitor::monitorDiffs(bool flag)
{
  if (env_)
  {
    if (!flag)
    {
      if (publish_environment_)
      {
        ROS_WARN_NAMED(LOGNAME, "Diff monitoring was stopped while publishing environment diffs. "
                                "Stopping environment diff publisher");
        stopPublishingEnvironment();
      }
    }
  }
}

void EnvironmentMonitor::stopPublishingEnvironment()
{
  if (publish_environment_)
  {
    std::unique_ptr<boost::thread> copy;
    copy.swap(publish_environment_);
    new_environment_update_condition_.notify_all();
    copy->join();
    monitorDiffs(false);
    environment_publisher_.shutdown();
    ROS_INFO_NAMED(LOGNAME, "Stopped publishing maintained environment.");
  }
}

void EnvironmentMonitor::startPublishingEnvironment(EnvironmentUpdateType update_type, const std::string& environment_topic)
{
  publish_update_types_ = update_type;
  if (!publish_environment_ && env_)
  {
    environment_publisher_ = nh_.advertise<tesseract_msgs::TesseractState>(environment_topic, 100, false);
    ROS_INFO_NAMED(LOGNAME, "Publishing maintained environment on '%s'", environment_topic.c_str());
    monitorDiffs(true);
    publish_environment_.reset(new boost::thread(boost::bind(&EnvironmentMonitor::environmentPublishingThread, this)));
  }
}

void EnvironmentMonitor::environmentPublishingThread()
{
  ROS_DEBUG_NAMED(LOGNAME, "Started environment publishing thread ...");

  // publish the full planning scene
  tesseract_msgs::TesseractState start_msg;
  tesseract_ros::tesseractToTesseractStateMsg(start_msg, *env_);

  environment_publisher_.publish(start_msg);
  ros::Duration(1.5).sleep();
  environment_publisher_.publish(start_msg);

  ROS_DEBUG_NAMED(LOGNAME, "Published the full planning scene: '%s'", start_msg.name.c_str());

  do
  {
    tesseract_msgs::TesseractState msg;
    bool publish_msg = false;
    bool is_full = false;
    ros::Rate rate(publish_environment_frequency_);
    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      while (new_environment_update_ == UPDATE_NONE && publish_environment_)
        new_environment_update_condition_.wait(ulock);
      if (new_environment_update_ != UPDATE_NONE)
      {
        if ((publish_update_types_ & new_environment_update_) || new_environment_update_ == UPDATE_ENVIRONMENT)
        {
          if (new_environment_update_ == UPDATE_ENVIRONMENT)
          {
            is_full = true;
            tesseract_ros::tesseractToTesseractStateMsg(msg, *env_);
          }
          else
          {
            tesseract_ros::tesseractToTesseractStateMsg(msg, *env_);
            msg.is_diff = true;
          }

          // also publish timestamp of this robot_state
          msg.joint_state.header.stamp = last_robot_motion_time_;
          publish_msg = true;
        }
        new_environment_update_ = UPDATE_NONE;
      }
    }

    if (publish_msg)
    {
      rate.reset();
      environment_publisher_.publish(msg);
      if (is_full)
      {
        ROS_DEBUG_NAMED(LOGNAME, "Published full environment: '%s'", msg.name.c_str());
      }
      rate.sleep();
    }
  } while (publish_environment_);
}

void EnvironmentMonitor::getMonitoredTopics(std::vector<std::string>& topics) const
{
  topics.clear();
  if (current_state_monitor_)
  {
    const std::string& t = current_state_monitor_->getMonitoredTopic();
    if (!t.empty())
      topics.push_back(t);
  }
  if (environment_subscriber_)
    topics.push_back(environment_subscriber_.getTopic());
}

void EnvironmentMonitor::triggerEnvironmentUpdateEvent(EnvironmentUpdateType update_type)
{
  // do not modify update functions while we are calling them
  boost::recursive_mutex::scoped_lock lock(update_lock_);

  for (std::size_t i = 0; i < update_callbacks_.size(); ++i)
    update_callbacks_[i](update_type);
  new_environment_update_ = (EnvironmentUpdateType)((int)new_environment_update_ | (int)update_type);
  new_environment_update_condition_.notify_all();
}

bool EnvironmentMonitor::requestEnvironmentState(const std::string& service_name)
{
  ROS_ERROR("tesseract environment request environment state is currently not implemented, %s", service_name.c_str());

//  // use global namespace for service
//  ros::ServiceClient client = ros::NodeHandle().serviceClient<moveit_msgs::GetPlanningScene>(service_name);
//  moveit_msgs::GetPlanningScene srv;
//  srv.request.components.components =
//      srv.request.components.SCENE_SETTINGS | srv.request.components.ROBOT_STATE |
//      srv.request.components.ROBOT_STATE_ATTACHED_OBJECTS | srv.request.components.WORLD_OBJECT_NAMES |
//      srv.request.components.WORLD_OBJECT_GEOMETRY | srv.request.components.OCTOMAP |
//      srv.request.components.TRANSFORMS | srv.request.components.ALLOWED_COLLISION_MATRIX |
//      srv.request.components.LINK_PADDING_AND_SCALING | srv.request.components.OBJECT_COLORS;

//  // Make sure client is connected to server
//  if (!client.exists())
//  {
//    ROS_DEBUG_STREAM_NAMED(LOGNAME, "Waiting for service `" << service_name << "` to exist.");
//    client.waitForExistence(ros::Duration(5.0));
//  }

//  if (client.call(srv))
//  {
//    newEnvironmentMessage(srv.response.scene);
//  }
//  else
//  {
//    ROS_INFO_NAMED(LOGNAME, "Failed to call service %s, have you launched move_group? at %s:%d", service_name.c_str(),
//                   __FILE__, __LINE__);
//    return false;
//  }
  return true;
}

void EnvironmentMonitor::newEnvironmentCallback(const tesseract_msgs::TesseractStateConstPtr& env)
{
  newEnvironmentMessage(*env);
}

bool EnvironmentMonitor::newEnvironmentMessage(const tesseract_msgs::TesseractState& env_msg)
{
  if (!env_)
    return false;

  bool result;

  EnvironmentUpdateType upd = UPDATE_ENVIRONMENT;
  std::string old_scene_name;
  {
    boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);

    last_update_time_ = ros::Time::now();
    last_robot_motion_time_ = env_msg.joint_state.header.stamp;
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "environment update " << fmod(last_update_time_.toSec(), 10.) << " robot stamp: " << fmod(last_robot_motion_time_.toSec(), 10.));
    old_scene_name = env_->getName();
    result = tesseract_ros::processTesseractStateMsg(env_, env_msg);
  }

  // if we have a diff, try to more accuratelly determine the update type
  if (env_msg.is_diff)
  {
//    bool no_other_scene_upd = (env_msg.name.empty() || env_msg.name == old_scene_name) &&
//                               env_msg.allowed_collision_matrix.entry_names.empty();

    // TODO: Levi Need to add back allowed collision matrix
    bool no_other_scene_upd = (env_msg.name.empty() || env_msg.name == old_scene_name);

    if (no_other_scene_upd)
    {
      upd = UPDATE_NONE;
      if (!env_msg.attachable_objects.empty() || !env_msg.attached_bodies.empty())
        upd = (EnvironmentUpdateType)((int)upd | (int)UPDATE_GEOMETRY);

//      if (!env.fixed_frame_transforms.empty())
//        upd = (EnvironmentUpdateType)((int)upd | (int)UPDATE_TRANSFORMS);

      if (!isMsgEmpty(env_msg.multi_dof_joint_state) || !isMsgEmpty(env_msg.multi_dof_joint_state))
        upd = (EnvironmentUpdateType)((int)upd | (int)UPDATE_STATE);
    }
  }
  triggerEnvironmentUpdateEvent(upd);
  return result;
}

//void EnvironmentMonitor::newPlanningSceneWorldCallback(
//    const moveit_msgs::PlanningSceneWorldConstPtr& world)
//{
//  if (scene_)
//  {
//    updateFrameTransforms();
//    {
//      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//      last_update_time_ = ros::Time::now();
//      scene_->getWorldNonConst()->clearObjects();
//      scene_->processPlanningSceneWorldMsg(*world);
//      if (octomap_monitor_)
//      {
//        if (world->octomap.octomap.data.empty())
//        {
//          octomap_monitor_->getOcTreePtr()->lockWrite();
//          octomap_monitor_->getOcTreePtr()->clear();
//          octomap_monitor_->getOcTreePtr()->unlockWrite();
//        }
//      }
//    }
//    triggerSceneUpdateEvent(UPDATE_SCENE);
//  }
//}

//void EnvironmentMonitor::collisionObjectFailTFCallback(
//    const moveit_msgs::CollisionObjectConstPtr& obj, tf::filter_failure_reasons::FilterFailureReason reason)
//{
//  // if we just want to remove objects, the frame does not matter
//  if (reason == tf::filter_failure_reasons::EmptyFrameID && obj->operation == moveit_msgs::CollisionObject::REMOVE)
//    collisionObjectCallback(obj);
//}

//void EnvironmentMonitor::attachableObjectCallback(const tesseract_msgs::AttachableObjectConstPtr &ao_msg)
//{
//  if (env_)
//  {
//    {
//      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//      last_update_time_ = ros::Time::now();

//      tesseract_ros::processAttachableObjectMsg(env_, *ao_msg);
//    }
//    triggerEnvironmentUpdateEvent(UPDATE_GEOMETRY);
//  }
//}

//void EnvironmentMonitor::attachedBodyInfoCallback(const tesseract_msgs::AttachedBodyInfoConstPtr &ab_info_msg)
//{
//  if (env_)
//  {
//    {
//      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
//      last_update_time_ = ros::Time::now();

//      tesseract_ros::processAttachedBodyInfoMsg(env_, *ab_info_msg);
//    }
//    triggerEnvironmentUpdateEvent(UPDATE_GEOMETRY);
//  }
//}

bool EnvironmentMonitor::waitForCurrentState(const ros::Time& t, double wait_time)
{
  if (t.isZero())
    return false;
  ros::WallTime start = ros::WallTime::now();
  ros::WallDuration timeout(wait_time);

  ROS_DEBUG_NAMED(LOGNAME, "sync robot state to: %.3fs", fmod(t.toSec(), 10.));

  if (current_state_monitor_)
  {
    // Wait for next robot update in state monitor. Those updates are only passed to PSM when robot actually moved!
    enforce_next_state_update_ = true;  // enforce potential updates to be directly applied
    bool success = current_state_monitor_->waitForCurrentState(t, wait_time);
    enforce_next_state_update_ =
        false;  // back to normal throttling behavior, not applying incoming updates immediately

    /* If the robot doesn't move, we will never receive an update from CSM in planning scene.
       As we ensured that an update, if it is triggered by CSM, is directly passed to the scene,
       we can early return true here (if we successfully received a CSM update). Otherwise return false. */
    if (success)
      return true;

    ROS_WARN_NAMED(LOGNAME, "Failed to fetch current robot state.");
    return false;
  }

  // Sometimes there is no state monitor. In this case state updates are received as part of scene updates only.
  // However, scene updates are only published if the robot actually moves. Hence we need a timeout!
  // As publishing planning scene updates is throttled (2Hz by default), a 1s timeout is a suitable default.
  boost::shared_lock<boost::shared_mutex> lock(scene_update_mutex_);
  ros::Time prev_robot_motion_time = last_robot_motion_time_;
  while (last_robot_motion_time_ < t &&  // Wait until the state update actually reaches the scene.
         timeout > ros::WallDuration())
  {
    ROS_DEBUG_STREAM_NAMED(LOGNAME, "last robot motion: " << (t - last_robot_motion_time_).toSec() << " ago");
    new_environment_update_condition_.wait_for(lock, boost::chrono::nanoseconds(timeout.toNSec()));
    timeout -= ros::WallTime::now() - start;  // compute remaining wait_time
  }
  bool success = last_robot_motion_time_ >= t;
  // suppress warning if we received an update at all
  if (!success && prev_robot_motion_time != last_robot_motion_time_)
    ROS_WARN_NAMED(LOGNAME, "Maybe failed to update robot state, time diff: %.3fs",
                   (t - last_robot_motion_time_).toSec());

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "sync done: robot motion: " << (t - last_robot_motion_time_).toSec()
                                                              << " scene update: " << (t - last_update_time_).toSec());
  return success;
}

void EnvironmentMonitor::lockEnvironmentRead()
{
  scene_update_mutex_.lock_shared();
}

void EnvironmentMonitor::unlockEnvironmentRead()
{
  scene_update_mutex_.unlock_shared();
}

void EnvironmentMonitor::lockEnvironmentWrite()
{
  scene_update_mutex_.lock();
}

void EnvironmentMonitor::unlockEnvironmentWrite()
{
  scene_update_mutex_.unlock();
}

void EnvironmentMonitor::startEnvironmentMonitor(const std::string& environment_topic)
{
  stopEnvironmentMonitor();

  ROS_INFO_NAMED(LOGNAME, "Starting environment monitor");
  // listen for environment updates; these messages include transforms, so no need for filters
  if (!environment_topic.empty())
  {
    environment_subscriber_ = root_nh_.subscribe(environment_topic, 100, &EnvironmentMonitor::newEnvironmentCallback, this);
    ROS_INFO_NAMED(LOGNAME, "Listening to '%s'", root_nh_.resolveName(environment_topic).c_str());
  }
}

void EnvironmentMonitor::stopEnvironmentMonitor()
{
  ROS_INFO_NAMED(LOGNAME, "Stopping environment monitor");

  if (environment_subscriber_)
    environment_subscriber_.shutdown();
}

void EnvironmentMonitor::startStateMonitor(const std::string& joint_states_topic)
{
  stopStateMonitor();
  if (env_)
  {

    if (!current_state_monitor_)
      current_state_monitor_.reset(new CurrentStateMonitor(env_, root_nh_));

    current_state_monitor_->addUpdateCallback(boost::bind(&EnvironmentMonitor::onStateUpdate, this, _1));
    current_state_monitor_->startStateMonitor(joint_states_topic);

    {
      boost::mutex::scoped_lock lock(state_pending_mutex_);
      if (!dt_state_update_.isZero())
        state_update_timer_.start();
    }
  }
  else
    ROS_ERROR_NAMED(LOGNAME, "Cannot monitor robot state because planning scene is not configured");
}

void EnvironmentMonitor::stopStateMonitor()
{
  if (current_state_monitor_)
    current_state_monitor_->stopStateMonitor();

  // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
  state_update_timer_.stop();
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    state_update_pending_ = false;
  }
}

void EnvironmentMonitor::onStateUpdate(const sensor_msgs::JointStateConstPtr& /* joint_state */)
{
  const ros::WallTime& n = ros::WallTime::now();
  ros::WallDuration dt = n - last_robot_state_update_wall_time_;

  bool update = enforce_next_state_update_;
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);

    if (dt < dt_state_update_ && !update)
    {
      state_update_pending_ = true;
    }
    else
    {
      state_update_pending_ = false;
      last_robot_state_update_wall_time_ = n;
      update = true;
    }
  }
  // run the state update with state_pending_mutex_ unlocked
  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::stateUpdateTimerCallback(const ros::WallTimerEvent& /*event*/)
{
  if (state_update_pending_)
  {
    bool update = false;

    const ros::WallTime& n = ros::WallTime::now();
    ros::WallDuration dt = n - last_robot_state_update_wall_time_;

    {
      // lock for access to dt_state_update_ and state_update_pending_
      boost::mutex::scoped_lock lock(state_pending_mutex_);
      if (state_update_pending_ && dt >= dt_state_update_)
      {
        state_update_pending_ = false;
        last_robot_state_update_wall_time_ = ros::WallTime::now();
        update = true;
        ROS_DEBUG_STREAM_NAMED(LOGNAME, "performPendingStateUpdate: " << fmod(last_robot_state_update_wall_time_.toSec(), 10));
      }
    }

    // run the state update with state_pending_mutex_ unlocked
    if (update)
    {
      updateEnvironmentWithCurrentState();
      ROS_DEBUG_NAMED(LOGNAME, "performPendingStateUpdate done");
    }
  }
}

void EnvironmentMonitor::setStateUpdateFrequency(double hz)
{
  bool update = false;
  if (hz > std::numeric_limits<double>::epsilon())
  {
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_.fromSec(1.0 / hz);
    state_update_timer_.setPeriod(dt_state_update_);
    state_update_timer_.start();
  }
  else
  {
    // stop must be called with state_pending_mutex_ unlocked to avoid deadlock
    state_update_timer_.stop();
    boost::mutex::scoped_lock lock(state_pending_mutex_);
    dt_state_update_ = ros::WallDuration(0, 0);
    if (state_update_pending_)
      update = true;
  }
  ROS_INFO_NAMED(LOGNAME, "Updating internal planning scene state at most every %lf seconds", dt_state_update_.toSec());

  if (update)
    updateEnvironmentWithCurrentState();
}

void EnvironmentMonitor::updateEnvironmentWithCurrentState()
{
  if (current_state_monitor_)
  {
    std::vector<std::string> missing;
    if (!current_state_monitor_->haveCompleteState(missing) &&
        (ros::Time::now() - current_state_monitor_->getMonitorStartTime()).toSec() > 1.0)
    {
      std::string missing_str = boost::algorithm::join(missing, ", ");
      ROS_WARN_THROTTLE_NAMED(1, LOGNAME, "The complete state of the robot is not yet known.  Missing %s",
                              missing_str.c_str());
    }

    {
      boost::unique_lock<boost::shared_mutex> ulock(scene_update_mutex_);
      last_update_time_ = last_robot_motion_time_ = current_state_monitor_->getCurrentStateTime();
      ROS_DEBUG_STREAM_NAMED(LOGNAME, "robot state update " << fmod(last_robot_motion_time_.toSec(), 10.));

      EnvState current_state;
      current_state_monitor_->setToCurrentState(current_state);
      env_->setState(current_state.joints);
    }
    triggerEnvironmentUpdateEvent(UPDATE_STATE);
  }
  else
    ROS_ERROR_THROTTLE_NAMED(1, LOGNAME, "State monitor is not active. Unable to set the planning scene state");
}

void EnvironmentMonitor::addUpdateCallback(const boost::function<void(EnvironmentUpdateType)>& fn)
{
  boost::recursive_mutex::scoped_lock lock(update_lock_);
  if (fn)
    update_callbacks_.push_back(fn);
}

void EnvironmentMonitor::clearUpdateCallbacks()
{
  boost::recursive_mutex::scoped_lock lock(update_lock_);
  update_callbacks_.clear();
}

void EnvironmentMonitor::setEnvironmentPublishingFrequency(double hz)
{
  publish_environment_frequency_ = hz;
  ROS_DEBUG_NAMED(LOGNAME, "Maximum frquency for publishing an environment is now %lf Hz", publish_environment_frequency_);
}
