#include <trajopt_common/utils.hpp>

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
}  // namespace trajopt_common
