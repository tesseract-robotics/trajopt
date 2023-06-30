#include <trajopt_common/utils.hpp>
#include <tesseract_common/utils.h>

namespace trajopt_common
{
SafetyMarginData::SafetyMarginData(double default_safety_margin, double default_safety_margin_coeff)
  : default_safety_margin_data_({ default_safety_margin, default_safety_margin_coeff })
  , max_safety_margin_(default_safety_margin)
{
}

void SafetyMarginData::setPairSafetyMarginData(const std::string& obj1,
                                               const std::string& obj2,
                                               double safety_margin,
                                               double safety_margin_coeff)
{
  std::array<double, 2> data({ safety_margin, safety_margin_coeff });
  auto key = tesseract_common::makeOrderedLinkPair(obj1, obj2);
  pair_lookup_table_[key] = data;

  if (safety_margin > max_safety_margin_)
    max_safety_margin_ = safety_margin;

  if (tesseract_common::almostEqualRelativeAndAbs(safety_margin_coeff, 0.0))
    zero_coeff_.insert(key);
  else
    zero_coeff_.erase(key);
}

const std::array<double, 2>& SafetyMarginData::getPairSafetyMarginData(const std::string& obj1,
                                                                       const std::string& obj2) const
{
  /** @brief Making this thread_local reduce memory allocations by 1,556,329 */
  thread_local tesseract_common::LinkNamesPair key;
  tesseract_common::makeOrderedLinkPair(key, obj1, obj2);

  auto it = pair_lookup_table_.find(key);
  if (it != pair_lookup_table_.end())
    return it->second;

  return default_safety_margin_data_;
}

double SafetyMarginData::getMaxSafetyMargin() const { return max_safety_margin_; }

const std::set<tesseract_common::LinkNamesPair>& SafetyMarginData::getPairsWithZeroCoeff() const { return zero_coeff_; }

std::vector<SafetyMarginData::Ptr> createSafetyMarginDataVector(int num_elements,
                                                                double default_safety_margin,
                                                                double default_safety_margin_coeff)
{
  std::vector<SafetyMarginData::Ptr> info;
  info.reserve(static_cast<size_t>(num_elements));
  for (auto i = 0; i < num_elements; ++i)
    info.push_back(std::make_shared<SafetyMarginData>(default_safety_margin, default_safety_margin_coeff));

  return info;
}

Eigen::Isometry3d addTwist(const Eigen::Isometry3d& t1,
                           const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist,
                           double dt)
{
  Eigen::Isometry3d t2;
  t2.setIdentity();
  Eigen::Vector3d angle_axis = (t1.rotation().inverse() * twist.tail(3)) * dt;
  t2.linear() = t1.rotation() * Eigen::AngleAxisd(angle_axis.norm(), angle_axis.normalized());
  t2.translation() = t1.translation() + twist.head(3) * dt;
  return t2;
}
}  // namespace trajopt_common
