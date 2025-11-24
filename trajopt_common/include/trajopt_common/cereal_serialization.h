#ifndef TRAJOPT_COMMON_CEREAL_SERIALIZATION_H
#define TRAJOPT_COMMON_CEREAL_SERIALIZATION_H

#include <trajopt_common/collision_types.h>

#include <tesseract_collision/core/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/utility.hpp>

namespace trajopt_common
{
template <class Archive>
void serialize(Archive& ar, CollisionCoeffData& obj)
{
  ar(cereal::make_nvp("default_collision_coeff", obj.default_collision_coeff_));
  ar(cereal::make_nvp("lookup_table", obj.lookup_table_));
  ar(cereal::make_nvp("zero_coeff", obj.zero_coeff_));
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
