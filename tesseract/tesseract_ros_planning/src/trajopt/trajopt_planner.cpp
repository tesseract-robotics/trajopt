/**
 * @file trajopt_planner.cpp
 * @brief Tesseract ROS Trajopt planner
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <tesseract_ros_planning/trajopt/trajopt_planner.h>
#include <trajopt/problem_description.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/config.hpp>
#include <jsoncpp/json/json.h>
#include <ros/console.h>

using namespace trajopt;

namespace tesseract
{

namespace tesseract_ros
{

  bool TrajoptPlanner::solve(PlannerResponse& res)
  {
    Json::Value root;
    Json::Reader reader;
    if (request_.config_format == "json")
    {
      bool parse_success = reader.parse(request_.config.c_str(), root);
      if (!parse_success)
      {
        ROS_FATAL("Failed to load trajopt json file from ros parameter");
      }
    }
    else
    {
      ROS_FATAL("Invalid config format: %s. Only json format is currently support for this planner.");
    }

    TrajOptProbPtr prob = ConstructProblem(root, request_.env);
    BasicTrustRegionSQP opt(prob);
    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    ros::Time tStart = ros::Time::now();
    opt.optimize();
    ROS_INFO("planning time: %.3f", (ros::Time::now() - tStart).toSec());

    tesseract::ContactResultMap collisions;
    const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
    const std::vector<std::string>& link_names = prob->GetKin()->getLinkNames();

    collisions.clear();
    request_.env->continuousCollisionCheckTrajectory(joint_names, link_names, getTraj(opt.x(), prob->GetVars()), collisions);

    tesseract::ContactResultVector collision_vector;
    tesseract::moveContactResultsMapToContactResultsVector(collisions, collision_vector);
    ROS_INFO("Final trajectory number of continuous collisions: %lui\n", collision_vector.size());
  }

  bool terminate()
  {

  }

  void clear()
  {

  }

}
}
