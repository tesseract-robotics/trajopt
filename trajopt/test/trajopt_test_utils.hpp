#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <json/json.h>
#include <stdexcept>
#include <ros/package.h>
#include <Eigen/Core>

#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_environment/core/environment.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/typedefs.hpp>

Json::Value readJsonFile(const std::string& fname)
{
  Json::Value root;
  Json::Reader reader;
  std::ifstream fh(fname.c_str());
  bool parse_success = reader.parse(fh, root);
  if (!parse_success)
    throw std::runtime_error("failed to parse " + fname);
  return root;
}

std::string locateResource(const std::string& url)
{
  std::string mod_url = url;
  if (url.find("package://") == 0)
  {
    mod_url.erase(0, strlen("package://"));
    size_t pos = mod_url.find("/");
    if (pos == std::string::npos)
    {
      return std::string();
    }

    std::string package = mod_url.substr(0, pos);
    mod_url.erase(0, pos);
    std::string package_path = ros::package::getPath(package);

    if (package_path.empty())
    {
      return std::string();
    }

    mod_url = package_path + mod_url; // "file://" + package_path + mod_url;
  }

  return mod_url;
}

/**
 * @brief Should perform a continuous collision check over the trajectory and stop on first collision.
 * @param manager A continuous contact manager
 * @param env The environment
 * @param joint_names JointNames corresponding to the values in traj (must be in same order)
 * @param link_names Name of the links to calculate collision data for.
 * @param traj The joint values at each time step
 * @param contacts A vector of vector of ContactMap where each indicie corrisponds to a timestep
 * @param first_only Indicates if it should return on first contact
 * @return True if collision was found, otherwise false.
 */
bool checkTrajectory(tesseract_collision::ContinuousContactManager& manager,
                     const tesseract_environment::Environment& env,
                     const std::vector<std::string>& joint_names,
                     const std::vector<std::string>& active_link_names,
                     const trajopt::TrajArray& traj,
                     std::vector<tesseract_collision::ContactResultMap>& contacts,
                     bool first_only = true)
{
  bool found = false;

  contacts.reserve(static_cast<size_t>(traj.rows() - 1));
  for (int iStep = 0; iStep < traj.rows() - 1; ++iStep)
  {
    tesseract_collision::ContactResultMap collisions;

    tesseract_environment::EnvStatePtr state0 = env.getState(joint_names, traj.row(iStep));
    tesseract_environment::EnvStatePtr state1 = env.getState(joint_names, traj.row(iStep + 1));

    for (const auto& link_name : active_link_names)
      manager.setCollisionObjectsTransform(link_name, state0->transforms[link_name], state1->transforms[link_name]);

    manager.contactTest(collisions, tesseract_collision::ContactTestTypes::FIRST);

    if (collisions.size() > 0)
      found = true;

    contacts.push_back(collisions);

    if (found && first_only)
      break;
  }

  return found;
}
