#ifndef TRAJOPT_COMMON_CEREAL_SERIALIZATION_H
#define TRAJOPT_COMMON_CEREAL_SERIALIZATION_H

#include <trajopt_common/collision_types.h>

#include <tesseract/collision/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/unordered_map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/utility.hpp>

namespace trajopt_common
{
template <class Archive>
void save(Archive& ar, const CollisionCoeffData& obj)
{
  ar(cereal::make_nvp("default_collision_coeff", obj.default_collision_coeff_));

  // Serialize as string-based format for backwards compatibility
  std::unordered_map<tesseract::common::LinkNamesPair, double> compat;
  for (const auto& [key, entry] : obj.lookup_table_)
    compat[tesseract::common::LinkNamesPair{ entry.name1, entry.name2 }] = entry.coeff;
  ar(cereal::make_nvp("lookup_table", compat));

  std::set<tesseract::common::LinkNamesPair> zero_compat;
  for (const auto& id_pair : obj.zero_coeff_)
  {
    auto it = obj.lookup_table_.find(id_pair);
    if (it != obj.lookup_table_.end())
      zero_compat.insert(tesseract::common::LinkNamesPair{ it->second.name1, it->second.name2 });
  }
  ar(cereal::make_nvp("zero_coeff", zero_compat));
}

template <class Archive>
void load(Archive& ar, CollisionCoeffData& obj)
{
  ar(cereal::make_nvp("default_collision_coeff", obj.default_collision_coeff_));

  std::unordered_map<tesseract::common::LinkNamesPair, double> compat;
  ar(cereal::make_nvp("lookup_table", compat));
  for (const auto& [names, coeff] : compat)
    obj.setCollisionCoeff(names.first, names.second, coeff);

  // zero_coeff_ is rebuilt by setCollisionCoeff, so just consume the archive field
  std::set<tesseract::common::LinkNamesPair> zero_compat;
  ar(cereal::make_nvp("zero_coeff", zero_compat));
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
