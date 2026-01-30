/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Tyler Marr
 * @date August 8, 2025
 *
 * @copyright Copyright (c) 2025, Tyler Marr, Confinity Robotics
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

#ifndef TRAJOPT_COMMON_YAML_EXTENSIONS_H
#define TRAJOPT_COMMON_YAML_EXTENSIONS_H

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_types.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <tesseract_common/yaml_extensions.h>

namespace YAML
{
//=========================== CollisionCoeffData ===========================
template <>
struct convert<trajopt_common::CollisionCoeffData>
{
  static Node encode(const trajopt_common::CollisionCoeffData& rhs)
  {
    Node node;

    // Encode default coefficient
    node["default_coeff"] = rhs.getDefaultCollisionCoeff();

    // Encode all pair-specific coefficients
    const auto& pair_data = rhs.getCollisionCoeffPairData();
    if (!pair_data.empty())
    {
      Node pair_coeff_data_node(YAML::NodeType::Sequence);
      for (const auto& pair : pair_data)
      {
        Node key_node(NodeType::Sequence);
        key_node.push_back(pair.first.first);
        key_node.push_back(pair.first.second);

        // tell yaml-cpp “emit this sequence in [a, b] inline style”
        key_node.SetStyle(YAML::EmitterStyle::Flow);

        pair_coeff_data_node[key_node] = pair.second;
      }
      node["pair_coeff_data"] = pair_coeff_data_node;
    }

    return node;
  }

  static bool decode(const Node& node, trajopt_common::CollisionCoeffData& rhs)
  {
    if (!node.IsMap())
      return false;

    if (const YAML::Node& n = node["default_coeff"])
      rhs.setDefaultCollisionCoeff(n.as<double>());

    if (const YAML::Node& pair_coeff_data_node = node["pair_coeff_data"])
    {
      for (auto it = pair_coeff_data_node.begin(); it != pair_coeff_data_node.end(); ++it)
      {
        Node key_node = it->first;
        if (!key_node.IsSequence() || key_node.size() != 2)
          return false;

        rhs.setCollisionCoeff(key_node[0].as<std::string>(), key_node[1].as<std::string>(), it->second.as<double>());
      }
    }

    return true;
  }
};

//=========================== TrajOptCollisionConfig ===========================
template <>
struct convert<trajopt_common::TrajOptCollisionConfig>
{
  static Node encode(const trajopt_common::TrajOptCollisionConfig& rhs)
  {
    Node node;
    node["enabled"] = rhs.enabled;
    node["contact_manager_config"] = rhs.contact_manager_config;
    node["collision_check_config"] = rhs.collision_check_config;
    node["collision_coeff_data"] = rhs.collision_coeff_data;
    node["collision_margin_buffer"] = rhs.collision_margin_buffer;
    node["max_num_cnt"] = rhs.max_num_cnt;
    return node;
  }

  static bool decode(const Node& node, trajopt_common::TrajOptCollisionConfig& rhs)
  {
    if (!node.IsMap())
      return false;

    if (const YAML::Node& n = node["enabled"])
      rhs.enabled = n.as<bool>();
    if (const YAML::Node& n = node["contact_manager_config"])
      rhs.contact_manager_config = n.as<tesseract_collision::ContactManagerConfig>();
    if (const YAML::Node& n = node["collision_check_config"])
      rhs.collision_check_config = n.as<tesseract_collision::CollisionCheckConfig>();
    if (const YAML::Node& n = node["collision_coeff_data"])
      rhs.collision_coeff_data = n.as<trajopt_common::CollisionCoeffData>();

    // Accept both 'collision_margin_buffer' and legacy alias 'buffer'
    if (const YAML::Node& n = node["collision_margin_buffer"])
      rhs.collision_margin_buffer = n.as<double>();

    if (const YAML::Node& n = node["max_num_cnt"])
      rhs.max_num_cnt = n.as<int>();

    // Optional: scale contact manager margins if provided
    if (const YAML::Node& n = node["scale_margins"])
      rhs.contact_manager_config.scaleMargins(n.as<double>());

    return true;
  }
};

}  // namespace YAML

#endif  // TRAJOPT_COMMON_YAML_EXTENSIONS_H
