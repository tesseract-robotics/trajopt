#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <filesystem>
#include <string>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/utils.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>

#include <trajopt_common/collision_types.h>
#include <trajopt_common/logging.hpp>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>

using namespace trajopt_ifopt;
using namespace tesseract::collision;
using namespace tesseract::environment;

namespace
{
/** @brief How many link entries came back under each continuous collision type */
struct CcTypeCounts
{
  int none{ 0 };
  int time0{ 0 };
  int time1{ 0 };
  int between{ 0 };

  std::string str() const
  {
    return "none=" + std::to_string(none) + " time0=" + std::to_string(time0) + " time1=" + std::to_string(time1) +
           " between=" + std::to_string(between);
  }
};

/**
 * @brief Assert the guarantees the cc_type/cc_time pair carries, and count the segment ends.
 *
 * The type fixes the time, not the reverse. A contact typed Time0 sits at the start of the segment
 * and carries cc_time 0; one typed Time1 sits at the end and carries cc_time 1; a link that took no
 * part in the cast is typed None and keeps a negative time. The converse must not be asserted: the
 * backend chooses Time0 or Time1 over Between from a support-function comparison with its own
 * tolerance, so a Between contact may legitimately carry a time arbitrarily close to 0 or 1.
 *
 * @return A count of the link entries under each type, so a failure says what did come back
 */
CcTypeCounts checkCcTypeInvariants(const ContactResultMap& contacts)
{
  CcTypeCounts counts;
  for (const auto& pair : contacts)
  {
    for (const auto& r : pair.second)
    {
      for (std::size_t j = 0; j < 2; ++j)
      {
        switch (r.cc_type[j])
        {
          case ContinuousCollisionType::CCType_Time0:
            EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(r.cc_time[j], 0.0));
            ++counts.time0;
            break;
          case ContinuousCollisionType::CCType_Time1:
            EXPECT_TRUE(tesseract::common::almostEqualRelativeAndAbs(r.cc_time[j], 1.0));
            ++counts.time1;
            break;
          case ContinuousCollisionType::CCType_Between:
            EXPECT_GE(r.cc_time[j], 0.0);
            EXPECT_LE(r.cc_time[j], 1.0);
            ++counts.between;
            break;
          case ContinuousCollisionType::CCType_None:
            EXPECT_LT(r.cc_time[j], 0.0);
            ++counts.none;
            break;
        }
      }
    }
  }
  return counts;
}
}  // namespace

class ContinuousCollisionEvaluatorTest : public testing::Test
{
public:
  Environment::Ptr env = std::make_shared<Environment>();

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");

    const tesseract::common::ResourceLocator::Ptr locator =
        std::make_shared<tesseract::common::GeneralResourceLocator>();
    ASSERT_TRUE(env->init(urdf_file, srdf_file, locator));

    trajopt_common::gLogLevel = trajopt_common::LevelError;
  }

  static trajopt_common::TrajOptCollisionConfig makeConfig(CollisionEvaluatorType type)
  {
    trajopt_common::TrajOptCollisionConfig config(0.02, 1);
    config.collision_check_config.type = type;
    config.collision_check_config.longest_valid_segment_length = 0.05;
    config.collision_margin_buffer = 0.05;
    return config;
  }
};

// A segment subdivided into casts ends at the last cast, so a contact there is at cc_time 1 and
// must be typed Time1.
//
// The end joint values need real margin on both sides: the manager types a contact Time1 only when
// its support-function comparison places the deepest approach at the far end of the cast. Measured
// at longest_valid_segment_length 0.05, a Time1 comes back for a (d, -d) end value with d in
// [-0.74, -0.27]; -0.5 is central, ~0.24 from either edge, and holds for every
// longest_valid_segment_length from 0.03 to 0.08.
TEST_F(ContinuousCollisionEvaluatorTest, LVSContinuousSegmentEndIsTime1)  // NOLINT
{
  auto manip = env->getJointGroup("manipulator");
  LVSContinuousCollisionEvaluator evaluator(manip, env, makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS));

  Eigen::VectorXd dof_vals0(2);
  dof_vals0 << -0.75, 0.75;
  Eigen::VectorXd dof_vals1(2);
  dof_vals1 << -0.5, 0.5;

  trajopt_common::CollisionCacheData data;
  evaluator.calcCollisionData(data, dof_vals0, dof_vals1, false, false, 1000);

  ASSERT_FALSE(data.contact_results_map.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(data.contact_results_map);
  EXPECT_GT(counts.time1, 0) << "Nothing was typed Time1, so the arm marking the end of a subdivided segment never "
                                "fired. Types seen: "
                             << counts.str();
}

// The discrete evaluator enumerates sub-states rather than the casts between them, so its last
// index is one higher. It is the control for the test above: both must hold the same invariant.
TEST_F(ContinuousCollisionEvaluatorTest, LVSDiscreteSegmentEndIsTime1)  // NOLINT
{
  auto manip = env->getJointGroup("manipulator");
  LVSDiscreteCollisionEvaluator evaluator(manip, env, makeConfig(CollisionEvaluatorType::LVS_DISCRETE));

  Eigen::VectorXd dof_vals0(2);
  dof_vals0 << -0.75, 0.75;
  Eigen::VectorXd dof_vals1(2);
  dof_vals1 << -0.5, 0.5;

  trajopt_common::CollisionCacheData data;
  evaluator.calcCollisionData(data, dof_vals0, dof_vals1, false, false, 1000);

  ASSERT_FALSE(data.contact_results_map.empty());
  const CcTypeCounts counts = checkCcTypeInvariants(data.contact_results_map);
  EXPECT_GT(counts.time1, 0) << "Nothing was typed Time1, so the arm marking the end of a subdivided segment never "
                                "fired. Types seen: "
                             << counts.str();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
