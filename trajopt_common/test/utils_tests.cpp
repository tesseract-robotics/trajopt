#include <gtest/gtest.h>
#include <Eigen/Geometry>
#include <trajopt_common/utils.hpp>
#include <tesseract/common/utils.h>

/** @brief Check the map from an angular velocity to the rate of an angle axis vector against a central difference */
TEST(UtilsUnit, calcAngleAxisRateMap)  // NOLINT
{
  // No turn, turns either side of the angle below which the map drops its last term, and nearly half a turn
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 2.0) / 3.0;
  for (const double angle : { 0.0, 1e-9, 1e-6, 1e-3, 0.7, 3.0 })
  {
    const Eigen::Matrix3d rotation = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
    const Eigen::Matrix3d map = trajopt_common::calcAngleAxisRateMap(angle * axis);
    ASSERT_TRUE(map.allFinite()) << "angle " << angle;

    constexpr double delta{ 1e-5 };
    for (int i = 0; i < 3; ++i)
    {
      const Eigen::Vector3d turned =
          tesseract::common::calcRotationalError(Eigen::AngleAxisd(delta, Eigen::Vector3d::Unit(i)) * rotation);
      const Eigen::Vector3d turned_back =
          tesseract::common::calcRotationalError(Eigen::AngleAxisd(-delta, Eigen::Vector3d::Unit(i)) * rotation);
      const Eigen::Vector3d rate = (turned - turned_back) / (2 * delta);
      ASSERT_TRUE(rate.allFinite()) << "angle " << angle;
      EXPECT_LT((map.col(i) - rate).cwiseAbs().maxCoeff(), 1e-9) << "angle " << angle << ", axis " << i;
    }
  }
}
