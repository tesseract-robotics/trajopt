#include <trajopt_common/utils.hpp>

#include <cmath>

namespace trajopt_common
{
Eigen::Isometry3d addTwist(const Eigen::Isometry3d& t1,
                           const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist,
                           double dt)
{
  Eigen::Isometry3d t2;
  t2.setIdentity();
  const Eigen::Vector3d angle_axis = (t1.rotation().inverse() * twist.tail(3)) * dt;
  t2.linear() = t1.rotation() * Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
  t2.translation() = t1.translation() + twist.head(3) * dt;
  return t2;
}

Eigen::Matrix3d calcAngleAxisRateMap(const Eigen::Ref<const Eigen::Vector3d>& angle_axis)
{
  Eigen::Matrix3d cross;
  cross << 0.0, -angle_axis.z(), angle_axis.y(), angle_axis.z(), 0.0, -angle_axis.x(), -angle_axis.y(), angle_axis.x(),
      0.0;

  // Below this angle the squared term is smaller than the rounding of the others
  const double angle = angle_axis.norm();
  const double factor =
      (angle < 1e-8) ? 0.0 : ((1.0 / (angle * angle)) - (1.0 / (2.0 * angle * std::tan(angle / 2.0))));

  return Eigen::Matrix3d::Identity() - 0.5 * cross + factor * cross * cross;
}
}  // namespace trajopt_common
