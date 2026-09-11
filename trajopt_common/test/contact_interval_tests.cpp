#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <limits>
#include <tesseract/collision/types.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_common/collision_utils.h>

using tesseract::collision::CollisionCheckConfig;
using tesseract::collision::CollisionEvaluatorType;
using trajopt_common::castCount;
using trajopt_common::ContactInterval;
using trajopt_common::contactInterval;
using trajopt_common::IntervalWeights;
using trajopt_common::intervalWeights;

namespace
{
CollisionCheckConfig makeConfig(CollisionEvaluatorType type, double longest_valid_segment_length)
{
  CollisionCheckConfig config;
  config.type = type;
  config.longest_valid_segment_length = longest_valid_segment_length;
  return config;
}

void expectWeights(const IntervalWeights& actual, double start_a, double start_b, double end_a, double end_b)
{
  EXPECT_NEAR(actual.start_a, start_a, 1e-14);
  EXPECT_NEAR(actual.start_b, start_b, 1e-14);
  EXPECT_NEAR(actual.end_a, end_a, 1e-14);
  EXPECT_NEAR(actual.end_b, end_b, 1e-14);
}
}  // namespace

// A segment no longer than the longest valid segment length is one cast; a longer one is split into
// the fewest equal casts that are each no longer than it. The lengths avoid exact multiples, whose
// quotient may round to either side of an integer.
TEST(ContactIntervalTest, CastCountSplitsLongSegmentsUnderLvsContinuous)  // NOLINT
{
  const CollisionCheckConfig config = makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS, 0.2);
  EXPECT_EQ(castCount(config, 0.0), 1);
  EXPECT_EQ(castCount(config, 0.1), 1);
  EXPECT_EQ(castCount(config, 0.2), 1);
  EXPECT_EQ(castCount(config, 0.3), 2);
  EXPECT_EQ(castCount(config, 0.81), 5);
}

// CONTINUOUS casts a segment once whatever the longest valid segment length says
TEST(ContactIntervalTest, CastCountIsOneUnderContinuous)  // NOLINT
{
  EXPECT_EQ(castCount(makeConfig(CollisionEvaluatorType::CONTINUOUS, 0.2), 0.81), 1);
}

// A length that is not a number is not longer than the limit, so the segment is cast once
TEST(ContactIntervalTest, CastCountCastsALengthThatIsNotANumberOnce)  // NOLINT
{
  const CollisionCheckConfig config = makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS, 0.2);
  EXPECT_EQ(castCount(config, std::numeric_limits<double>::quiet_NaN()), 1);
}

// A longest valid segment length that is not positive cannot subdivide a segment, so the check casts
// once instead of reaching the float-to-integer conversion, which is undefined for a length of zero
TEST(ContactIntervalTest, CastCountCastsOnceWithoutAPositiveLongestValidSegmentLength)  // NOLINT
{
  EXPECT_EQ(castCount(makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS, 0.0), 0.81), 1);
  EXPECT_EQ(castCount(makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS, -0.2), 0.81), 1);
}

TEST(ContactIntervalTest, ContactIntervalIsTheCastHoldingTheTime)  // NOLINT
{
  const ContactInterval interval = contactInterval(0.45, 5);
  EXPECT_NEAR(interval.start, 0.4, 1e-15);
  EXPECT_NEAR(interval.end, 0.6, 1e-15);

  for (const double t : { 0.0, 0.13, 0.2, 0.4, 0.6, 0.8, 0.99, 1.0 })
  {
    const ContactInterval cast = contactInterval(t, 5);
    EXPECT_NEAR(cast.end - cast.start, 0.2, 1e-12) << "t = " << t;
    EXPECT_LE(cast.start, t + 1e-12) << "t = " << t;
    EXPECT_GE(cast.end, t - 1e-12) << "t = " << t;
    EXPECT_GE(cast.start, 0.0) << "t = " << t;
    EXPECT_LE(cast.end, 1.0) << "t = " << t;
  }
}

// A single cast is the whole segment
TEST(ContactIntervalTest, ContactIntervalOfOneCastIsTheSegment)  // NOLINT
{
  const ContactInterval interval = contactInterval(0.3, 1);
  EXPECT_DOUBLE_EQ(interval.start, 0.0);
  EXPECT_DOUBLE_EQ(interval.end, 1.0);
}

// A check at interpolated states finds each contact at a point in time
TEST(ContactIntervalTest, ContactIntervalWithoutCastsIsAPointInTime)  // NOLINT
{
  const ContactInterval interval = contactInterval(0.3, 0);
  EXPECT_DOUBLE_EQ(interval.start, 0.3);
  EXPECT_DOUBLE_EQ(interval.end, 0.3);
}

// On a single cast the segment start's gradient takes all its weight at the segment start and the
// segment end's all its weight at the segment end: Schulman et al. 2014, Eq. (20)
TEST(ContactIntervalTest, WeightsOnOneCastTakeEachTimestepAtItsOwnState)  // NOLINT
{
  expectWeights(intervalWeights(0.3, { 0.0, 1.0 }), 0.7, 0.0, 0.0, 0.3);
}

// A point in time weights the one state it is at by 1 - cc_time and cc_time
TEST(ContactIntervalTest, WeightsAtAPointInTime)  // NOLINT
{
  expectWeights(intervalWeights(0.3, { 0.3, 0.3 }), 0.7, 0.0, 0.3, 0.0);
}

// Inside a sub-cast both timesteps blend both of its ends: a contact a quarter of the way into
// [0.4, 0.6] puts three quarters of each timestep's weight at 0.4
TEST(ContactIntervalTest, WeightsInsideASubCast)  // NOLINT
{
  expectWeights(intervalWeights(0.45, { 0.4, 0.6 }), 0.45, 0.1, 0.3, 0.15);
}

// Whatever the cast count, the start weights sum to 1 - cc_time and the end weights to cc_time,
// the weights the two timesteps carry for an unsplit gradient
TEST(ContactIntervalTest, WeightsSumToTheTimeWeights)  // NOLINT
{
  for (const long cast_count : { 0L, 1L, 3L, 7L })
  {
    for (const double t : { 0.0, 0.05, 0.37, 0.5, 0.91, 1.0 })
    {
      const IntervalWeights w = intervalWeights(t, contactInterval(t, cast_count));
      EXPECT_NEAR(w.start_a + w.start_b, 1.0 - t, 1e-14) << "t = " << t << ", casts = " << cast_count;
      EXPECT_NEAR(w.end_a + w.end_b, t, 1e-14) << "t = " << t << ", casts = " << cast_count;
    }
  }
}

// A time outside its interval is placed at the interval's nearest end rather than extrapolated
TEST(ContactIntervalTest, WeightsClampATimeOutsideItsInterval)  // NOLINT
{
  expectWeights(intervalWeights(0.35, { 0.4, 0.6 }), 0.6, 0.0, 0.4, 0.0);
  expectWeights(intervalWeights(0.65, { 0.4, 0.6 }), 0.0, 0.4, 0.0, 0.6);
}
