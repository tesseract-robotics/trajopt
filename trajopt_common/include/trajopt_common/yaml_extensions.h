/**
 * @file yaml_extensions.h
 * @brief YAML Type conversions
 *
 * @author Tyler Marr
 * @date August 8, 2025
 * @version TODO
 * @bug No known bugs
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

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

    // Encode default coeff inferred from an unknown pair
    node["default_coeff"] = rhs.getCollisionCoeff("__DEFAULT_COEFF_A__", "__DEFAULT_COEFF_B__");

    // NOTE: The default coefficient and full lookup table are private.
    // We encode only pairs known to be zero via getPairsWithZeroCoeff().
    const auto& zero_pairs = rhs.getPairsWithZeroCoeff();
    if (!zero_pairs.empty())
    {
      Node unique_pairs(YAML::NodeType::Sequence);
      for (const auto& p : zero_pairs)
      {
        Node entry;
        Node pair(YAML::NodeType::Sequence);
        pair.push_back(p.first);
        pair.push_back(p.second);
        entry["pair"] = pair;
        entry["coeff"] = 0.0;
        unique_pairs.push_back(entry);
      }
      node["unique_pairs"] = unique_pairs;
    }

    return node;
  }

  static bool decode(const Node& node, trajopt_common::CollisionCoeffData& rhs)
  {
    if (!node || !node.IsMap())
    {
      rhs = trajopt_common::CollisionCoeffData();
      return true;
    }

    double default_coeff = 1.0;
    if (const YAML::Node& n = node["default_coeff"])
      default_coeff = n.as<double>();

    trajopt_common::CollisionCoeffData out(default_coeff);

    if (const YAML::Node& up = node["unique_pairs"])
    {
      if (!up.IsSequence())
        return false;
      for (const auto& entry : up)
      {
        if (!entry.IsMap())
          return false;
        const YAML::Node& pair_node = entry["pair"];
        if (!pair_node || !pair_node.IsSequence() || pair_node.size() != 2)
          return false;
        std::string a = pair_node[0].as<std::string>();
        std::string b = pair_node[1].as<std::string>();
        if (const YAML::Node& cn = entry["coeff"])
        {
          double c = cn.as<double>();
          out.setCollisionCoeff(a, b, c);
        }
      }
    }

    rhs = out;
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
    if (const YAML::Node& n = node["buffer"])
      rhs.collision_margin_buffer = n.as<double>();

    if (const YAML::Node& n = node["max_num_cnt"])
      rhs.max_num_cnt = n.as<int>();

    // Optional: scale contact manager margins if provided
    if (const YAML::Node& n = node["collision_scale"])
      rhs.contact_manager_config.scaleMargins(n.as<double>());

    return true;
  }
};

}  // namespace YAML

#endif  // TRAJOPT_COMMON_YAML_EXTENSIONS_H