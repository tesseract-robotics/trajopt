#ifndef TRAJOPT_COMMON_CEREAL_SERIALIZATION_H
#define TRAJOPT_COMMON_CEREAL_SERIALIZATION_H

#include <trajopt_common/collision_types.h>

#include <tesseract/collision/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/unordered_set.hpp>
#include <cereal/types/utility.hpp>

#include <unordered_set>
#include <utility>

namespace trajopt_common
{
template <class Archive>
void save(Archive& ar, const CollisionCoeffData& obj)
{
  ar(cereal::make_nvp("default_collision_coeff", obj.getDefaultCollisionCoeff()));
  ar(cereal::make_nvp("lookup_table", obj.getCollisionCoeffPairData()));
  ar(cereal::make_nvp("zero_coeff", obj.getPairsWithZeroCoeff()));
}

/**
 * @brief Load through the setters, so every coefficient is validated and the zero-coefficient pairs are derived from
 * the lookup table. The archived zero-coefficient set is read only to keep the archive layout.
 * @throws std::runtime_error if a coefficient is negative or not finite; @p obj is left unchanged
 */
template <class Archive>
void load(Archive& ar, CollisionCoeffData& obj)
{
  double default_collision_coeff{ 0 };
  PairsCollisionCoeffData lookup_table;
  std::unordered_set<tesseract::common::LinkIdPair> zero_coeff;
  ar(cereal::make_nvp("default_collision_coeff", default_collision_coeff));
  ar(cereal::make_nvp("lookup_table", lookup_table));
  ar(cereal::make_nvp("zero_coeff", zero_coeff));

  CollisionCoeffData data(default_collision_coeff);
  for (const auto& [pair, coeff] : lookup_table)
    data.setCollisionCoeff(pair, coeff);
  obj = std::move(data);
}

template <class Archive>
void serialize(Archive& ar, TrajOptCollisionConfig& obj)
{
  ar(cereal::make_nvp("enabled", obj.enabled));
  ar(cereal::make_nvp("contact_manager_config", obj.contact_manager_config));
  ar(cereal::make_nvp("collision_check_config", obj.collision_check_config));
  ar(cereal::make_nvp("collision_coeff_data", obj.collision_coeff_data));
  ar(cereal::make_nvp("collision_margin_buffer", obj.collision_margin_buffer));
  ar(cereal::make_nvp("max_num_cnt", obj.max_num_cnt));
}

}  // namespace trajopt_common
#endif  // TRAJOPT_COMMON_CEREAL_SERIALIZATION_H
