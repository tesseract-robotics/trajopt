/**
 * @file trajopt_common_yaml_conversions_tests.cpp
 * @brief This contains unit test for TrajOpt YAML conversions
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <yaml-cpp/yaml.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_common/yaml_extensions.h>
#include <tesseract_collision/core/yaml_extensions.h>
#include <trajopt_common/yaml_extensions.h>

#include <trajopt_common/collision_types.h>

class TrajoptCommonYAMLTestFixture : public ::testing::Test
{
public:
  TrajoptCommonYAMLTestFixture() = default;
  using ::testing::Test::Test;
};

TEST(TrajoptCommonYAMLTestFixture, CollisionCoeffDataConversionsUnit)  // NOLINT
{
  const std::string yaml_string = R"(
    default_coeff: 2.5
    unique_pairs:
      - { pair: [link_a, link_b], coeff: 0.0 }
      - { pair: [link_c, link_d], coeff: 1.5 }
  )";

  {  // decode
    trajopt_common::CollisionCoeffData d;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<trajopt_common::CollisionCoeffData>::decode(n, d);
    EXPECT_TRUE(success);

    // default for unknown
    EXPECT_NEAR(d.getCollisionCoeff("foo", "bar"), 2.5, 1e-8);
    // specified
    EXPECT_NEAR(d.getCollisionCoeff("link_c", "link_d"), 1.5, 1e-8);
    EXPECT_NEAR(d.getCollisionCoeff("link_a", "link_b"), 0.0, 1e-8);
  }

  {  // encode
    trajopt_common::CollisionCoeffData d(3.0);
    d.setCollisionCoeff("link_a", "link_b", 0.0);
    d.setCollisionCoeff("link_c", "link_d", 1.5);

    YAML::Node out = YAML::convert<trajopt_common::CollisionCoeffData>::encode(d);
    EXPECT_NEAR(out["default_coeff"].as<double>(), 3.0, 1e-8);
    if (out["unique_pairs"])  // optional
    {
      auto up = out["unique_pairs"];
      ASSERT_TRUE(up.IsSequence());
      ASSERT_EQ(up.size(), 2U);

      // Check that both pairs are present (order may vary)
      bool found_ab = false;
      bool found_cd = false;
      for (const auto& entry : up)
      {
        auto first = entry["pair"][0].as<std::string>();
        auto second = entry["pair"][1].as<std::string>();
        auto coeff = entry["coeff"].as<double>();

        if ((first == "link_a" && second == "link_b") || (first == "link_b" && second == "link_a"))
        {
          EXPECT_NEAR(coeff, 0.0, 1e-8);
          found_ab = true;
        }
        else if ((first == "link_c" && second == "link_d") || (first == "link_d" && second == "link_c"))
        {
          EXPECT_NEAR(coeff, 1.5, 1e-8);
          found_cd = true;
        }
      }
      EXPECT_TRUE(found_ab);
      EXPECT_TRUE(found_cd);
    }
  }
}

TEST(TrajoptCommonYAMLTestFixture, TrajOptCollisionConfigConversionsUnit)  // NOLINT
{
  const std::string yaml_string = R"(
    enabled: false
    contact_manager_config:
      default_margin: 0.02
    collision_check_config:
      type: DISCRETE
      longest_valid_segment_length: 0.01
      check_program_mode: ALL
    collision_coeff_data:
      default_coeff: 3.0
      unique_pairs:
        - { pair: [l0, l1], coeff: 0.0 }
    collision_margin_buffer: 0.05
    max_num_cnt: 10
    collision_scale: 0.5
  )";

  {  // decode
    trajopt_common::TrajOptCollisionConfig c;
    YAML::Node n = YAML::Load(yaml_string);
    auto success = YAML::convert<trajopt_common::TrajOptCollisionConfig>::decode(n, c);
    EXPECT_TRUE(success);

    EXPECT_FALSE(c.enabled);
    // default_margin scaled by 0.5
    EXPECT_TRUE(c.contact_manager_config.default_margin.has_value());
    if (c.contact_manager_config.default_margin.has_value())
    {
      EXPECT_NEAR(c.contact_manager_config.default_margin.value(), 0.01, 1e-8);
    }
    EXPECT_NEAR(c.collision_check_config.longest_valid_segment_length, 0.01, 1e-8);
    EXPECT_NEAR(c.collision_margin_buffer, 0.05, 1e-8);
    EXPECT_EQ(c.max_num_cnt, 10);

    // coeff data
    EXPECT_NEAR(c.collision_coeff_data.getCollisionCoeff("foo", "bar"), 3.0, 1e-8);
    EXPECT_NEAR(c.collision_coeff_data.getCollisionCoeff("l0", "l1"), 0.0, 1e-8);
  }

  {  // encode
    trajopt_common::TrajOptCollisionConfig c;
    c.enabled = false;
    c.contact_manager_config.default_margin = 0.01;
    c.collision_check_config.longest_valid_segment_length = 0.02;
    c.collision_coeff_data = trajopt_common::CollisionCoeffData(2.0);
    c.collision_coeff_data.setCollisionCoeff("l0", "l1", 0.0);
    c.collision_margin_buffer = 0.1;
    c.max_num_cnt = 5;

    YAML::Node out = YAML::convert<trajopt_common::TrajOptCollisionConfig>::encode(c);
    EXPECT_FALSE(out["enabled"].as<bool>());
    EXPECT_NEAR(out["contact_manager_config"]["default_margin"].as<double>(), 0.01, 1e-8);
    EXPECT_NEAR(out["collision_check_config"]["longest_valid_segment_length"].as<double>(), 0.02, 1e-8);
    EXPECT_NEAR(out["collision_margin_buffer"].as<double>(), 0.1, 1e-8);
    EXPECT_EQ(out["max_num_cnt"].as<int>(), 5);

    auto up = out["collision_coeff_data"]["unique_pairs"];
    ASSERT_TRUE(up.IsSequence());
    ASSERT_EQ(up.size(), 1U);
    // Check that the zero coefficient pair is present (order may vary due to internal ordering)
    bool found_l0_l1 = false;
    for (const auto& entry : up)
    {
      auto first = entry["pair"][0].as<std::string>();
      auto second = entry["pair"][1].as<std::string>();
      auto coeff = entry["coeff"].as<double>();

      if ((first == "l0" && second == "l1") || (first == "l1" && second == "l0"))
      {
        EXPECT_NEAR(coeff, 0.0, 1e-8);
        found_l0_l1 = true;
      }
    }
    EXPECT_TRUE(found_l0_l1);
  }
}

TEST(TrajoptCommonYAMLTestFixture, CollisionCoeffDataGetterMethodsUnit)  // NOLINT
{
  trajopt_common::CollisionCoeffData d(3.5);
  d.setCollisionCoeff("link1", "link2", 2.0);
  d.setCollisionCoeff("link3", "link4", 0.0);
  d.setCollisionCoeff("link5", "link6", 1.5);

  // Test getDefaultCollisionCoeff
  EXPECT_NEAR(d.getDefaultCollisionCoeff(), 3.5, 1e-12);

  // Test getCollisionCoeffPairData
  const auto& pair_data = d.getCollisionCoeffPairData();
  EXPECT_EQ(pair_data.size(), 3U);

  // Verify the pairs are stored correctly (note: pairs are ordered internally)
  EXPECT_NEAR(d.getCollisionCoeff("link1", "link2"), 2.0, 1e-12);
  EXPECT_NEAR(d.getCollisionCoeff("link3", "link4"), 0.0, 1e-12);
  EXPECT_NEAR(d.getCollisionCoeff("link5", "link6"), 1.5, 1e-12);

  // Test getPairsWithZeroCoeff
  const auto& zero_pairs = d.getPairsWithZeroCoeff();
  EXPECT_EQ(zero_pairs.size(), 1U);

  // Verify that the zero coefficient pair is in the zero_pairs set
  bool found_zero_pair = false;
  for (const auto& pair : zero_pairs)
  {
    if ((pair.first == "link3" && pair.second == "link4") || (pair.first == "link4" && pair.second == "link3"))
    {
      found_zero_pair = true;
      break;
    }
  }
  EXPECT_TRUE(found_zero_pair);
}

TEST(TrajoptCommonYAMLTestFixture, CollisionCoeffDataRoundTripUnit)  // NOLINT
{
  trajopt_common::CollisionCoeffData d_in(2.5);
  d_in.setCollisionCoeff("a", "b", 0.0);
  d_in.setCollisionCoeff("c", "d", 1.8);

  YAML::Node n = YAML::convert<trajopt_common::CollisionCoeffData>::encode(d_in);
  trajopt_common::CollisionCoeffData d_out;
  ASSERT_TRUE(YAML::convert<trajopt_common::CollisionCoeffData>::decode(n, d_out));

  EXPECT_NEAR(d_out.getCollisionCoeff("x", "y"), 2.5, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("a", "b"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("b", "a"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("c", "d"), 1.8, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("d", "c"), 1.8, 1e-12);
}

TEST(TrajoptCommonYAMLTestFixture, TrajOptCollisionConfigRoundTripUnit)  // NOLINT
{
  trajopt_common::TrajOptCollisionConfig c_in;
  c_in.enabled = false;
  c_in.contact_manager_config.default_margin = 0.02;
  c_in.collision_check_config.longest_valid_segment_length = 0.01;
  c_in.collision_coeff_data = trajopt_common::CollisionCoeffData(3.3);
  c_in.collision_coeff_data.setCollisionCoeff("l0", "l1", 0.0);
  c_in.collision_margin_buffer = 0.05;
  c_in.max_num_cnt = 7;

  YAML::Node n = YAML::convert<trajopt_common::TrajOptCollisionConfig>::encode(c_in);
  trajopt_common::TrajOptCollisionConfig c_out;
  ASSERT_TRUE(YAML::convert<trajopt_common::TrajOptCollisionConfig>::decode(n, c_out));

  EXPECT_EQ(c_out.enabled, c_in.enabled);
  ASSERT_TRUE(c_out.contact_manager_config.default_margin.has_value());
  ASSERT_TRUE(c_in.contact_manager_config.default_margin.has_value());
  if (c_out.contact_manager_config.default_margin.has_value() && c_in.contact_manager_config.default_margin.has_value())
  {
    EXPECT_NEAR(
        c_out.contact_manager_config.default_margin.value(), c_in.contact_manager_config.default_margin.value(), 1e-12);
  }
  EXPECT_NEAR(c_out.collision_check_config.longest_valid_segment_length,
              c_in.collision_check_config.longest_valid_segment_length,
              1e-12);
  EXPECT_NEAR(c_out.collision_coeff_data.getCollisionCoeff("x", "y"), 3.3, 1e-12);
  EXPECT_NEAR(c_out.collision_coeff_data.getCollisionCoeff("l0", "l1"), 0.0, 1e-12);
  EXPECT_NEAR(c_out.collision_margin_buffer, c_in.collision_margin_buffer, 1e-12);
  EXPECT_EQ(c_out.max_num_cnt, c_in.max_num_cnt);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}