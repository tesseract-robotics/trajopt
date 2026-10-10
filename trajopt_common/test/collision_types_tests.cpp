#include <gtest/gtest.h>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#include <trajopt_common/cereal_serialization.h>
#include <trajopt_common/collision_types.h>
#include <trajopt_common/collision_utils.h>
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

namespace
{
/** @brief A contact between two links, optionally reported from the other link */
tesseract::collision::ContactResult
makeContact(int shape_id0, int subshape_id0, int shape_id1, int subshape_id1, bool reversed = false)
{
  tesseract::collision::ContactResult contact;
  contact.link_ids = { tesseract::common::LinkId("link_a"), tesseract::common::LinkId("link_b") };
  contact.shape_id = { shape_id0, shape_id1 };
  contact.subshape_id = { subshape_id0, subshape_id1 };
  if (reversed)
  {
    std::swap(contact.link_ids[0], contact.link_ids[1]);
    std::swap(contact.shape_id[0], contact.shape_id[1]);
    std::swap(contact.subshape_id[0], contact.subshape_id[1]);
  }
  return contact;
}

trajopt_common::ShapePairKey
shapePairKey(int shape_id0, int subshape_id0, int shape_id1, int subshape_id1, bool reversed = false)
{
  return trajopt_common::getShapePairKey(makeContact(shape_id0, subshape_id0, shape_id1, subshape_id1, reversed));
}
}  // namespace

TEST(ShapePairKeyUnit, DistinguishesShapes)  // NOLINT
{
  // A shape without subshapes against a subshape of another shape, on either link
  EXPECT_NE(shapePairKey(0, 1, 0, -1), shapePairKey(2, -1, 0, -1));
  EXPECT_NE(shapePairKey(0, -1, 0, 1), shapePairKey(0, -1, 2, -1));

  // Neighbouring shapes with large subshape ids
  EXPECT_NE(shapePairKey(1, 135000000, 0, -1), shapePairKey(0, 135000001, 0, -1));

  // Every id takes part in the key
  const trajopt_common::ShapePairKey key = shapePairKey(1, 2, 3, 4);
  EXPECT_NE(key, shapePairKey(0, 2, 3, 4));
  EXPECT_NE(key, shapePairKey(1, 0, 3, 4));
  EXPECT_NE(key, shapePairKey(1, 2, 0, 4));
  EXPECT_NE(key, shapePairKey(1, 2, 3, 0));
}

TEST(ShapePairKeyUnit, IndependentOfReportedLinkOrder)  // NOLINT
{
  // The same shape pair, reported from either link
  EXPECT_EQ(shapePairKey(1, 2, 3, 4), shapePairKey(1, 2, 3, 4, true));

  // Mirrored shape pairs, each reported from a different link
  EXPECT_NE(shapePairKey(1, -1, 0, -1), shapePairKey(0, -1, 1, -1, true));
}

TEST(GradientResultsSetsUnit, GroupsContactsByShapePair)  // NOLINT
{
  const tesseract::common::LinkIdPair link_pair(tesseract::common::LinkId("link_a"),
                                                tesseract::common::LinkId("link_b"));

  tesseract::collision::ContactResultVector contacts;
  // One shape pair, reported from either link
  contacts.push_back(makeContact(1, 2, 3, 4));
  contacts.push_back(makeContact(1, 2, 3, 4, true));
  // Its mirror, reported from the other link
  contacts.push_back(makeContact(3, 4, 1, 2, true));
  // A subshape of one shape and another shape without subshapes
  contacts.push_back(makeContact(0, 1, 0, -1));
  contacts.push_back(makeContact(2, -1, 0, -1));

  // Tag each gradient result with the index of its contact
  for (std::size_t i = 0; i < contacts.size(); ++i)
    contacts[i].distance = static_cast<double>(i);

  std::vector<trajopt_common::GradientResultsSet> sets;
  trajopt_common::appendGradientResultsSets(
      sets,
      link_pair,
      contacts,
      /*coeff=*/5.0,
      /*is_continuous=*/true,
      [](trajopt_common::GradientResults& grad, const tesseract::collision::ContactResult& contact) {
        grad.error = contact.distance;
      });

  std::map<trajopt_common::ShapePairKey, std::vector<double>> members;
  for (const auto& set : sets)
  {
    EXPECT_EQ(set.key, link_pair);
    EXPECT_DOUBLE_EQ(set.coeff, 5.0);
    EXPECT_TRUE(set.is_continuous);
    EXPECT_TRUE(members.find(set.shape_key) == members.end());
    for (const auto& result : set.results)
      members[set.shape_key].push_back(result.error);
  }

  ASSERT_EQ(sets.size(), 4);
  EXPECT_EQ(members.at(trajopt_common::getShapePairKey(contacts[0])), (std::vector<double>{ 0, 1 }));
  EXPECT_EQ(members.at(trajopt_common::getShapePairKey(contacts[2])), (std::vector<double>{ 2 }));
  EXPECT_EQ(members.at(trajopt_common::getShapePairKey(contacts[3])), (std::vector<double>{ 3 }));
  EXPECT_EQ(members.at(trajopt_common::getShapePairKey(contacts[4])), (std::vector<double>{ 4 }));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
