/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include <tesseract_rviz/tesseract_state_plugin/tesseract_state_display.h>
#include <tesseract_ros/kdl/kdl_env.h>
#include <tesseract_ros/ros_tesseract_utils.h>

#include <urdf_parser/urdf_parser.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf/transform_listener.h>
#include <eigen_conversions/eigen_msg.h>
#include <octomap_msgs/conversions.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

namespace tesseract_rviz
{
using namespace tesseract;
// ******************************************************************************************
// Base class contructor
// ******************************************************************************************
TesseractStateDisplay::TesseractStateDisplay() : Display(), update_state_(false), load_env_(false)
{
  urdf_description_property_ = new rviz::StringProperty(
      "URDF Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded",
      this, SLOT(changedURDFDescription()), this);

  tesseract_state_topic_property_ = new rviz::RosTopicProperty(
      "Tesseract State Topic", "display_tesseract_state", ros::message_traits::datatype<tesseract_msgs::TesseractState>(),
      "The topic on which the tesseract_msgs::TesseractState messages are received", this, SLOT(changedTesseractStateTopic()),
      this);

  // Planning scene category -------------------------------------------------------------------------------------------
  root_link_name_property_ = new rviz::StringProperty("Robot Root Link", "", "Shows the name of the root link for the robot model", this,
                               SLOT(changedRootLinkName()), this);
  root_link_name_property_->setReadOnly(true);

  urdf_alpha_property_ = new rviz::FloatProperty("URDF Alpha", 1.0f, "Specifies the alpha for the urdf links", this,
                                                  SLOT(changedURDFSceneAlpha()), this);
  urdf_alpha_property_->setMin(0.0);
  urdf_alpha_property_->setMax(1.0);

  attached_body_color_property_ =
      new rviz::ColorProperty("Attached Body Color", QColor(150, 50, 150), "The color for the attached bodies", this,
                              SLOT(changedAttachedBodyColor()), this);

  enable_link_highlight_ =
      new rviz::BoolProperty("Show Highlights", true, "Specifies whether link highlighting is enabled", this,
                             SLOT(changedEnableLinkHighlight()), this);
  enable_visual_visible_ =
      new rviz::BoolProperty("Show Robot Visual", true, "Whether to display the visual representation of the robot.", this,
                             SLOT(changedEnableVisualVisible()), this);

  enable_collision_visible_ = new rviz::BoolProperty("Show Robot Collision", false,
                                                     "Whether to display the collision representation of the robot.",
                                                     this, SLOT(changedEnableCollisionVisible()), this);
  enable_attached_visual_visible_ =
      new rviz::BoolProperty("Show Attached Visual", true, "Whether to display the visual representation of the attached objects.", this,
                             SLOT(changedEnableAttachedVisualVisible()), this);

  enable_attached_collision_visible_ = new rviz::BoolProperty("Show Attached Collision", false,
                                                     "Whether to display the collision representation of the attached objects.",
                                                     this, SLOT(changedEnableAttachedCollisionVisible()), this);

  show_all_links_ = new rviz::BoolProperty("Show All Links", true, "Toggle all links visibility on or off.", this,
                                           SLOT(changedAllLinks()), this);


}

// ******************************************************************************************
// Deconstructor
// ******************************************************************************************
TesseractStateDisplay::~TesseractStateDisplay()
{
}

void TesseractStateDisplay::onInitialize()
{
  Display::onInitialize();
  state_.reset(new StateVisualization(scene_node_, context_, "Tesseract State", this));
  changedEnableVisualVisible();
  changedEnableCollisionVisible();
  changedEnableAttachedVisualVisible();
  changedEnableAttachedCollisionVisible();
  state_->setVisible(false);
}

void TesseractStateDisplay::reset()
{
  state_->clear();
//  rdf_loader_.reset();
  Display::reset();

  loadURDFModel();
}

void TesseractStateDisplay::changedAllLinks()
{
  Property* links_prop = subProp("Links");
  QVariant value(show_all_links_->getBool());

  for (int i = 0; i < links_prop->numChildren(); ++i)
  {
    Property* link_prop = links_prop->childAt(i);
    link_prop->setValue(value);
  }
}

void TesseractStateDisplay::setHighlightedLink(const std::string& link_name, const std_msgs::ColorRGBA& color)
{
  rviz::RobotLink* link = state_->getRobot().getLink(link_name);
  if (link)
  {
    link->setColor(color.r, color.g, color.b);
    link->setRobotAlpha(color.a * urdf_alpha_property_->getFloat());
  }
}

void TesseractStateDisplay::unsetHighlightedLink(const std::string& link_name)
{
  rviz::RobotLink* link = state_->getRobot().getLink(link_name);
  if (link)
  {
    link->unsetColor();
    link->setRobotAlpha(urdf_alpha_property_->getFloat());
  }
}

void TesseractStateDisplay::changedEnableLinkHighlight()
{
  if (enable_link_highlight_->getBool())
  {
    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
    {
      setHighlightedLink(it->first, it->second);
    }
  }
  else
  {
    for (std::map<std::string, std_msgs::ColorRGBA>::iterator it = highlights_.begin(); it != highlights_.end(); ++it)
    {
      unsetHighlightedLink(it->first);
    }
  }
}

void TesseractStateDisplay::changedEnableVisualVisible()
{
  state_->setVisualVisible(enable_visual_visible_->getBool());
}

void TesseractStateDisplay::changedEnableCollisionVisible()
{
  state_->setCollisionVisible(enable_collision_visible_->getBool());
}

void TesseractStateDisplay::changedEnableAttachedVisualVisible()
{
  state_->setAttachedVisualVisible(enable_attached_visual_visible_->getBool());
}

void TesseractStateDisplay::changedEnableAttachedCollisionVisible()
{
  state_->setAttachedCollisionVisible(enable_attached_collision_visible_->getBool());
}

static bool operator!=(const std_msgs::ColorRGBA& a, const std_msgs::ColorRGBA& b)
{
  return a.r != b.r || a.g != b.g || a.b != b.b || a.a != b.a;
}

void TesseractStateDisplay::setHighlightedLinks(const tesseract_msgs::TesseractState::_highlight_links_type& highlight_links)
{
  if (highlight_links.empty() && highlights_.empty())
    return;

  std::map<std::string, std_msgs::ColorRGBA> highlights;
  for (tesseract_msgs::TesseractState::_highlight_links_type::const_iterator it = highlight_links.begin(); it != highlight_links.end(); ++it)
  {
    highlights[it->name] = it->visual[0];
  }

  if (enable_link_highlight_->getBool())
  {
    std::map<std::string, std_msgs::ColorRGBA>::iterator ho = highlights_.begin();
    std::map<std::string, std_msgs::ColorRGBA>::iterator hn = highlights.begin();
    while (ho != highlights_.end() || hn != highlights.end())
    {
      if (ho == highlights_.end())
      {
        setHighlightedLink(hn->first, hn->second);
        ++hn;
      }
      else if (hn == highlights.end())
      {
        unsetHighlightedLink(ho->first);
        ++ho;
      }
      else if (hn->first < ho->first)
      {
        setHighlightedLink(hn->first, hn->second);
        ++hn;
      }
      else if (hn->first > ho->first)
      {
        unsetHighlightedLink(ho->first);
        ++ho;
      }
      else if (hn->second != ho->second)
      {
        setHighlightedLink(hn->first, hn->second);
        ++ho;
        ++hn;
      }
      else
      {
        ++ho;
        ++hn;
      }
    }
  }

  swap(highlights, highlights_);
}

void TesseractStateDisplay::changedAttachedBodyColor()
{
  if (state_)
  {
    QColor color = attached_body_color_property_->getColor();
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = urdf_alpha_property_->getFloat();
    state_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void TesseractStateDisplay::changedURDFDescription()
{
  if (isEnabled())
    reset();
}

void TesseractStateDisplay::changedRootLinkName()
{
}

void TesseractStateDisplay::changedURDFSceneAlpha()
{
  if (state_)
  {
    state_->setAlpha(urdf_alpha_property_->getFloat());
    QColor color = attached_body_color_property_->getColor();
    std_msgs::ColorRGBA color_msg;
    color_msg.r = color.redF();
    color_msg.g = color.greenF();
    color_msg.b = color.blueF();
    color_msg.a = urdf_alpha_property_->getFloat();
    state_->setDefaultAttachedObjectColor(color_msg);
    update_state_ = true;
  }
}

void TesseractStateDisplay::changedTesseractStateTopic()
{
  tesseract_state_subscriber_.shutdown();

  // reset model to default state, we don't want to show previous messages
//  current_state_ = nullptr;
  update_state_ = true;

  tesseract_state_subscriber_ = nh_.subscribe(tesseract_state_topic_property_->getStdString(), 10,
                                              &TesseractStateDisplay::newTesseractStateCallback, this);
}

void TesseractStateDisplay::newTesseractStateCallback(const tesseract_msgs::TesseractStateConstPtr state_msg)
{
  if (!env_)
    return;

  tesseract_ros::processTesseractStateMsg(env_, *state_msg);

  setLinkColor(state_msg->object_colors);
  setHighlightedLinks(state_msg->highlight_links);
  update_state_ = true;
}

void TesseractStateDisplay::setLinkColor(const tesseract_msgs::TesseractState::_object_colors_type& link_colors)
{
  for (tesseract_msgs::TesseractState::_object_colors_type::const_iterator it = link_colors.begin(); it != link_colors.end(); ++it)
  {
    setLinkColor(it->name, QColor(it->visual[0].r, it->visual[0].g, it->visual[0].b));
  }
}

void TesseractStateDisplay::setLinkColor(const std::string& link_name, const QColor& color)
{
  setLinkColor(&state_->getRobot(), link_name, color);
}

void TesseractStateDisplay::unsetLinkColor(const std::string& link_name)
{
  unsetLinkColor(&state_->getRobot(), link_name);
}

void TesseractStateDisplay::setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor& color)
{
  rviz::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->setColor(color.redF(), color.greenF(), color.blueF());
}

void TesseractStateDisplay::unsetLinkColor(rviz::Robot* robot, const std::string& link_name)
{
  rviz::RobotLink* link = robot->getLink(link_name);

  // Check if link exists
  if (link)
    link->unsetColor();
}

// ******************************************************************************************
// Load
// ******************************************************************************************
void TesseractStateDisplay::loadURDFModel()
{
  load_env_ = false;
  // Initial setup
  std::string urdf_xml_string, srdf_xml_string;
  nh_.getParam(urdf_description_property_->getString().toStdString(), urdf_xml_string);
  nh_.getParam(urdf_description_property_->getString().toStdString() + "_semantic", srdf_xml_string);

  // Load URDF model
  if (urdf_xml_string.empty())
  {
    setStatus(rviz::StatusProperty::Error, "TesseractState", "No URDF model loaded");
  }
  else
  {
    urdf::ModelInterfaceSharedPtr urdf_model = urdf::parseURDF(urdf_xml_string);
    if (urdf_model != nullptr)
    {

      // Load SRDF model (Not required)
      srdf::ModelSharedPtr srdf_model = srdf::ModelSharedPtr(new srdf::Model);
      srdf_model->initString(*urdf_model, srdf_xml_string);

      tesseract_ros::KDLEnvPtr env = tesseract_ros::KDLEnvPtr(new tesseract_ros::KDLEnv);
      assert(env != nullptr);

      bool success = env->init(urdf_model, srdf_model);
      assert(success);

      if (success)
      {
        env_ = env;
        state_->load(*env_->getURDF());
        bool oldState = root_link_name_property_->blockSignals(true);
        root_link_name_property_->setStdString(env_->getRootLinkName());
        root_link_name_property_->blockSignals(oldState);
        update_state_ = true;
        setStatus(rviz::StatusProperty::Ok, "TesseractState", "Tesseract Environment Loaded Successfully");

        changedEnableVisualVisible();
        changedEnableCollisionVisible();
        changedEnableAttachedVisualVisible();
        changedEnableAttachedCollisionVisible();
        state_->setVisible(true);
      }
      else
      {
        setStatus(rviz::StatusProperty::Error, "TesseractState", "Tesseract Environment Failed to Load");
      }
    }
    else
    {
      setStatus(rviz::StatusProperty::Error, "TesseractState", "URDF file failed to parse");
    }
  }

  highlights_.clear();
}

void TesseractStateDisplay::onEnable()
{
  Display::onEnable();
  load_env_ = true;  // allow loading of robot model in update()
  calculateOffsetPosition();
}

// ******************************************************************************************
// Disable
// ******************************************************************************************
void TesseractStateDisplay::onDisable()
{
  tesseract_state_subscriber_.shutdown();
  if (state_)
    state_->setVisible(false);
  Display::onDisable();
}

void TesseractStateDisplay::update(float wall_dt, float ros_dt)
{
  Display::update(wall_dt, ros_dt);

  if (load_env_)
  {
    loadURDFModel();
    changedTesseractStateTopic();
  }

  calculateOffsetPosition();
  if (state_ && update_state_ && env_)
  {
    update_state_ = false;
    state_->update(env_, env_->getState());
  }
}

// ******************************************************************************************
// Calculate Offset Position
// ******************************************************************************************
void TesseractStateDisplay::calculateOffsetPosition()
{
  if (!env_)
    return;

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;

  context_->getFrameManager()->getTransform(env_->getRootLinkName(), ros::Time(0), position, orientation);
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void TesseractStateDisplay::fixedFrameChanged()
{
  Display::fixedFrameChanged();
  calculateOffsetPosition();
}

}  // namespace tesseract_rviz
