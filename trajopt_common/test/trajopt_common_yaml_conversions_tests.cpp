/**
 * @file trajopt_common_yaml_conversions_tests.cpp
 * @brief This contains unit test for TrajOpt YAML conversions
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
    pair_coeff_data:
      [link_a, link_b]: 0.0
      [link_c, link_d]: 1.5
  )";

  {  // decode
    YAML::Node n = YAML::Load(yaml_string);
    auto d = n.as<trajopt_common::CollisionCoeffData>();

    // default for unknown
    EXPECT_NEAR(d.getCollisionCoeff("foo", "bar"), 2.5, 1e-8);
    // specified
    EXPECT_NEAR(d.getCollisionCoeff("link_c", "link_d"), 1.5, 1e-8);
    EXPECT_NEAR(d.getCollisionCoeff("link_a", "link_b"), 0.0, 1e-8);
  }

  {  // encode
    trajopt_common::CollisionCoeffData data_original(3.0);
    data_original.setCollisionCoeff("link_a", "link_b", 0.0);
    data_original.setCollisionCoeff("link_c", "link_d", 1.5);

    YAML::Node n(data_original);
    auto data = n.as<trajopt_common::CollisionCoeffData>();
    EXPECT_NEAR(data.getDefaultCollisionCoeff(), 3.0, 1e-8);
    EXPECT_NEAR(data.getCollisionCoeff("link_a", "link_b"), 0.0, 1e-8);
    EXPECT_NEAR(data.getCollisionCoeff("link_c", "link_d"), 1.5, 1e-8);
    EXPECT_EQ(data.getPairsWithZeroCoeff().size(), 1U);
    EXPECT_EQ(data.getCollisionCoeffPairData().size(), 2U);
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
      pair_coeff_data:
        [l0, l1]: 0.0
    collision_margin_buffer: 0.05
    max_num_cnt: 10
    scale_margins: 0.5
  )";

  {  // decode
    YAML::Node n = YAML::Load(yaml_string);
    auto c = n.as<trajopt_common::TrajOptCollisionConfig>();

    EXPECT_FALSE(c.enabled);
    // default_margin scaled by 0.5
    EXPECT_TRUE(c.contact_manager_config.default_margin.has_value());
    // NOLINTNEXTLINE
    EXPECT_NEAR(c.contact_manager_config.default_margin.value(), 0.01, 1e-8);
    EXPECT_NEAR(c.collision_check_config.longest_valid_segment_length, 0.01, 1e-8);
    EXPECT_NEAR(c.collision_margin_buffer, 0.05, 1e-8);
    EXPECT_EQ(c.max_num_cnt, 10);

    // coeff data
    EXPECT_NEAR(c.collision_coeff_data.getCollisionCoeff("foo", "bar"), 3.0, 1e-8);
    EXPECT_NEAR(c.collision_coeff_data.getCollisionCoeff("l0", "l1"), 0.0, 1e-8);
  }

  {  // encode
    trajopt_common::TrajOptCollisionConfig data_original;
    data_original.enabled = false;
    data_original.contact_manager_config.default_margin = 0.01;
    data_original.collision_check_config.longest_valid_segment_length = 0.02;
    data_original.collision_coeff_data = trajopt_common::CollisionCoeffData(2.0);
    data_original.collision_coeff_data.setCollisionCoeff("l0", "l1", 0.0);
    data_original.collision_coeff_data.setCollisionCoeff("l1", "l2", 1.7);
    data_original.collision_margin_buffer = 0.1;
    data_original.max_num_cnt = 5;

    YAML::Node n(data_original);
    auto data = n.as<trajopt_common::TrajOptCollisionConfig>();

    EXPECT_FALSE(data.enabled);
    EXPECT_TRUE(data.contact_manager_config.default_margin.has_value());
    // NOLINTNEXTLINE
    EXPECT_NEAR(data.contact_manager_config.default_margin.value(), 0.01, 1e-8);
    EXPECT_NEAR(data.collision_check_config.longest_valid_segment_length, 0.02, 1e-8);
    EXPECT_NEAR(data.collision_margin_buffer, 0.1, 1e-8);
    EXPECT_EQ(data.max_num_cnt, 5);

    EXPECT_NEAR(data.collision_coeff_data.getDefaultCollisionCoeff(), 2.0, 1e-8);
    EXPECT_NEAR(data.collision_coeff_data.getCollisionCoeff("l0", "l1"), 0.0, 1e-8);
    EXPECT_NEAR(data.collision_coeff_data.getCollisionCoeff("l1", "l2"), 1.7, 1e-8);
    EXPECT_EQ(data.collision_coeff_data.getPairsWithZeroCoeff().size(), 1U);
    EXPECT_EQ(data.collision_coeff_data.getCollisionCoeffPairData().size(), 2U);
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

  YAML::Node n(d_in);
  auto d_out = n.as<trajopt_common::CollisionCoeffData>();
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

  YAML::Node n(c_in);
  auto c_out = n.as<trajopt_common::TrajOptCollisionConfig>();

  EXPECT_EQ(c_out.enabled, c_in.enabled);
  ASSERT_TRUE(c_out.contact_manager_config.default_margin.has_value());
  ASSERT_TRUE(c_in.contact_manager_config.default_margin.has_value());

  // NOLINTNEXTLINE
  EXPECT_NEAR(
      c_out.contact_manager_config.default_margin.value(), c_in.contact_manager_config.default_margin.value(), 1e-12);

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
