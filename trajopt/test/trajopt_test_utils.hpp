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
