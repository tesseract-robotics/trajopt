#ifndef TRAJOPT_COMMON_UTILS_HPP
#define TRAJOPT_COMMON_UTILS_HPP

#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <unordered_map>
#include <memory>
#include <vector>
#include <array>
#include <tesseract_common/types.h>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
/** @brief Store Safety Margin Data for a given timestep */
struct SafetyMarginData
{
  using Ptr = std::shared_ptr<SafetyMarginData>;
  using ConstPtr = std::shared_ptr<const SafetyMarginData>;

  SafetyMarginData(double default_safety_margin, double default_safety_margin_coeff);

  /**
   * @brief Set the safety margin for a given contact pair
   *
   * The order of the object names does not matter, that is handled internal to
   * the class.
   *
   * @param obj1 The first object name
   * @param obj2 The Second object name
   * @param safety_margins contacts with distance < safety_margin are penalized
   * @param safety_margin_coeffs A safety margin coefficient vector where each
   * element corresponds to a given timestep.
   */
  void setPairSafetyMarginData(const std::string& obj1,
                               const std::string& obj2,
                               double safety_margin,
                               double safety_margin_coeff);

  /**
   * @brief Get the pairs safety margin data
   *
   * If a safety margin for the request pair does not exist it returns the default safety margin data.
   *
   * @param obj1 The first object name
   * @param obj2 The second object name
   * @return A Vector2d[Contact Distance Threshold, Coefficient]
   */
  const std::array<double, 2>& getPairSafetyMarginData(const std::string& obj1, const std::string& obj2) const;

  /**
   * @brief Get the max safety margin data
   *
   * This used when setting the contact distance in the contact manager.
   *
   * @return Max contact distance threshold
   */
  double getMaxSafetyMargin() const;

  /**
   * @brief Get the pairs with zero coeff
   * @return A vector of pairs with zero coeff
   */
  const std::set<tesseract_common::LinkNamesPair>& getPairsWithZeroCoeff() const;

private:
  /// The coeff used during optimization
  /// safety margin: contacts with distance < dist_pen are penalized
  /// Stores [dist_pen, coeff]
  std::array<double, 2> default_safety_margin_data_;

  /// This use when requesting collision data because you can only provide a
  /// single contact distance threshold.
  double max_safety_margin_;

  /// A map of link pair to contact distance setting [dist_pen, coeff]
  std::unordered_map<tesseract_common::LinkNamesPair, std::array<double, 2>, tesseract_common::PairHash>
      pair_lookup_table_;

  /// Pairs containing zero coeff
  std::set<tesseract_common::LinkNamesPair> zero_coeff_;
};

/**
 * @brief This is a utility function for creating the Safety Margin data vector
 * @param num_elements The number of objects to create
 * @param default_safety_margin Default safety margin
 * @param default_safety_margin_coeff Default safety margin coeff
 * @return A vector of Safety Margin Data
 */
std::vector<SafetyMarginData::Ptr> createSafetyMarginDataVector(int num_elements,
                                                                double default_safety_margin,
                                                                double default_safety_margin_coeff);

template <typename T>
std::vector<T> concat(const std::vector<T>& a, const std::vector<T>& b)
{
  std::vector<T> out;
  out.reserve(a.size() + b.size());
  out.insert(out.end(), a.begin(), a.end());
  out.insert(out.end(), b.begin(), b.end());
  return out;
}

/**
 * @brief Apply a twist for dt to a given transform
 * @param t1 The transform to apply twist.
 * @param twist The twist to apply.
 * @param dt The delta time for which the twist is applied
 * @return Transform result of applying a twist for dt.
 */
Eigen::Isometry3d addTwist(const Eigen::Isometry3d& t1,
                           const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist,
                           double dt);
}  // namespace trajopt_common

#endif  // TRAJOPT_COMMON_UTILS_HPP
