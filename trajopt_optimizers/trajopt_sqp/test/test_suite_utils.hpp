/**
 * @file test_suite_utils.cpp
 * @brief test utils
 *
 * @author Levi Armstrong
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <string>
#include <tesseract_common/resource_locator.h>
TRAJOPT_IGNORE_WARNINGS_POP

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
