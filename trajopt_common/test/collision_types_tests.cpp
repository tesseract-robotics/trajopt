#include <gtest/gtest.h>
#include <trajopt_common/collision_types.h>
#include <tesseract/common/types.h>

TEST(CollisionCoeffDataUnit, CollidingPairsCoexist)  // NOLINT
{
  const auto a = tesseract::common::LinkId::createWithValueForTesting(42, "collide_a");
  const auto b = tesseract::common::LinkId::createWithValueForTesting(42, "collide_b");
  const tesseract::common::LinkId x("link_x");
  trajopt_common::CollisionCoeffData data(/*default_collision_coeff=*/1.0);
  data.setCollisionCoeff(a, x, 5.0);
  data.setCollisionCoeff(b, x, 0.0);  // 0 → also lands in zero_coeff_; previously threw
  EXPECT_DOUBLE_EQ(data.getCollisionCoeff(tesseract::common::LinkIdPair(a, x)), 5.0);
  EXPECT_DOUBLE_EQ(data.getCollisionCoeff(tesseract::common::LinkIdPair(b, x)), 0.0);
  EXPECT_FALSE(data.hasZeroCoeff(tesseract::common::LinkIdPair(a, x)));
  EXPECT_TRUE(data.hasZeroCoeff(tesseract::common::LinkIdPair(b, x)));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
