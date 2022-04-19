#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <fstream>
#include <json/json.h>
#include <stdexcept>
#include <Eigen/Core>
#include <tesseract_common/resource_locator.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/typedefs.hpp>

inline Json::Value readJsonFile(const std::string& fname)
{
  Json::Value root;
  Json::Reader reader;
  std::ifstream fh(fname.c_str());
  bool parse_success = reader.parse(fh, root);
  if (!parse_success)
    throw std::runtime_error("failed to parse " + fname);
  return root;
}

class TrajOptSupportResourceLocator : public tesseract_common::ResourceLocator
{
public:
  using Ptr = std::shared_ptr<TrajOptSupportResourceLocator>;
  using ConstPtr = std::shared_ptr<const TrajOptSupportResourceLocator>;

  std::shared_ptr<tesseract_common::Resource> locateResource(const std::string& url) const override
  {
    std::string mod_url = url;
    if (url.find("package://trajopt") == 0)
    {
      mod_url.erase(0, strlen("package://trajopt"));
      size_t pos = mod_url.find('/');
      if (pos == std::string::npos)
        return nullptr;

      std::string package = mod_url.substr(0, pos);
      mod_url.erase(0, pos);
      std::string package_path = std::string(TRAJOPT_DIR);

      if (package_path.empty())
        return nullptr;

      mod_url = package_path + mod_url;
    }

    if (!tesseract_common::fs::path(mod_url).is_complete())
      return nullptr;

    return std::make_shared<tesseract_common::SimpleLocatedResource>(
        url, mod_url, std::make_shared<TrajOptSupportResourceLocator>(*this));
  }
};
