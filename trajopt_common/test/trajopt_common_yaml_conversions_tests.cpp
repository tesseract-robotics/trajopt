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

  {  // encode (only zero coeff pairs encoded)
    trajopt_common::CollisionCoeffData d(3.0);
    d.setCollisionCoeff("link_a", "link_b", 0.0);
    d.setCollisionCoeff("link_c", "link_d", 1.5);

    YAML::Node out = YAML::convert<trajopt_common::CollisionCoeffData>::encode(d);
    EXPECT_NEAR(out["default_coeff"].as<double>(), 3.0, 1e-8);
    if (out["unique_pairs"])  // optional
    {
      auto up = out["unique_pairs"];
      ASSERT_TRUE(up.IsSequence());
      ASSERT_EQ(up.size(), 1u);
      EXPECT_EQ(up[0]["pair"][0].as<std::string>(), "link_a");
      EXPECT_EQ(up[0]["pair"][1].as<std::string>(), "link_b");
      EXPECT_NEAR(up[0]["coeff"].as<double>(), 0.0, 1e-8);
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
    EXPECT_NEAR(c.contact_manager_config.default_margin.value(), 0.01, 1e-8);
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
    ASSERT_EQ(up.size(), 1u);
    EXPECT_EQ(up[0]["pair"][0].as<std::string>(), "l0");
    EXPECT_EQ(up[0]["pair"][1].as<std::string>(), "l1");
    EXPECT_NEAR(up[0]["coeff"].as<double>(), 0.0, 1e-8);
  }
}

TEST(TrajoptCommonYAMLTestFixture, CollisionCoeffDataRoundTripUnit)  // NOLINT
{
  trajopt_common::CollisionCoeffData d_in(2.5);
  d_in.setCollisionCoeff("a", "b", 0.0);
  d_in.setCollisionCoeff("c", "d", 0.0);

  YAML::Node n = YAML::convert<trajopt_common::CollisionCoeffData>::encode(d_in);
  trajopt_common::CollisionCoeffData d_out;
  ASSERT_TRUE(YAML::convert<trajopt_common::CollisionCoeffData>::decode(n, d_out));

  EXPECT_NEAR(d_out.getCollisionCoeff("x", "y"), 2.5, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("a", "b"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("b", "a"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("c", "d"), 0.0, 1e-12);
  EXPECT_NEAR(d_out.getCollisionCoeff("d", "c"), 0.0, 1e-12);
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