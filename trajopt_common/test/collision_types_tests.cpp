#include <gtest/gtest.h>
#include <limits>
#include <stdexcept>
#include <string>
#include <trajopt_common/cereal_serialization.h>
#include <trajopt_common/collision_types.h>
#include <tesseract/common/serialization.h>
#include <tesseract/common/types.h>
#include <tesseract/common/unit_test_utils.h>
#include <tesseract/common/test_suite/name_id_testing.h>

TEST(CollisionCoeffDataUnit, CollidingPairsCoexist)  // NOLINT
{
  const auto a = tesseract::common::NameIdTestAccess::create<tesseract::common::LinkId>(42, "collide_a");
  const auto b = tesseract::common::NameIdTestAccess::create<tesseract::common::LinkId>(42, "collide_b");
  const tesseract::common::LinkId x("link_x");
  trajopt_common::CollisionCoeffData data(/*default_collision_coeff=*/1.0);
  data.setCollisionCoeff(a, x, 5.0);
  data.setCollisionCoeff(b, x, 0.0);  // 0 also lands in zero_coeff_
  EXPECT_DOUBLE_EQ(data.getCollisionCoeff({ a, x }), 5.0);
  EXPECT_DOUBLE_EQ(data.getCollisionCoeff({ b, x }), 0.0);
  EXPECT_FALSE(data.hasZeroCoeff({ a, x }));
  EXPECT_TRUE(data.hasZeroCoeff({ b, x }));
}

TEST(CollisionCoeffDataUnit, RejectsInvalidCoefficients)  // NOLINT
{
  const tesseract::common::LinkId x("link_x");
  const tesseract::common::LinkId y("link_y");
  trajopt_common::CollisionCoeffData data(/*default_collision_coeff=*/1.0);
  data.setCollisionCoeff(x, y, 5.0);

  for (const double bad : { -1.0, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN() })
  {
    EXPECT_THROW(trajopt_common::CollisionCoeffData{ bad }, std::runtime_error);
    EXPECT_THROW(data.setDefaultCollisionCoeff(bad), std::runtime_error);
    EXPECT_THROW(data.setCollisionCoeff(x, y, bad), std::runtime_error);
    EXPECT_THROW(data.setCollisionCoeff({ x, y }, bad), std::runtime_error);
  }

  // A rejected value leaves the stored coefficients unchanged.
  EXPECT_DOUBLE_EQ(data.getDefaultCollisionCoeff(), 1.0);
  EXPECT_DOUBLE_EQ(data.getCollisionCoeff({ x, y }), 5.0);
  EXPECT_FALSE(data.hasZeroCoeff({ x, y }));
}

TEST(CollisionCoeffDataUnit, SerializationRejectsInvalidCoefficients)  // NOLINT
{
  const tesseract::common::LinkId x("link_x");
  const tesseract::common::LinkId y("link_y");
  const tesseract::common::LinkId z("link_z");
  trajopt_common::CollisionCoeffData data(/*default_collision_coeff=*/1.25);
  data.setCollisionCoeff(x, y, 5.5);
  data.setCollisionCoeff(x, z, 0.0);
  tesseract::common::testSerialization<trajopt_common::CollisionCoeffData>(data, "CollisionCoeffData");

  using tesseract::common::Serialization;
  const std::string archive = Serialization::toArchiveStringJSON<trajopt_common::CollisionCoeffData>(data);
  for (const std::string value : { "1.25", "5.5" })
  {
    std::string bad_archive = archive;
    const std::size_t pos = bad_archive.find(value);
    ASSERT_NE(pos, std::string::npos) << archive;
    bad_archive.insert(pos, "-");
    EXPECT_THROW(Serialization::fromArchiveStringJSON<trajopt_common::CollisionCoeffData>(bad_archive),
                 std::runtime_error);
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
