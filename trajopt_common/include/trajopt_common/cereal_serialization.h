#ifndef TRAJOPT_COMMON_CEREAL_SERIALIZATION_H
#define TRAJOPT_COMMON_CEREAL_SERIALIZATION_H

#include <trajopt_common/collision_types.h>

#include <tesseract/collision/cereal_serialization.h>

#include <cereal/cereal.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/set.hpp>
#include <cereal/types/utility.hpp>

namespace trajopt_common
{
template <class Archive>
void save(Archive& ar, const CollisionCoeffData& obj)
{
  ar(cereal::make_nvp("default_collision_coeff", obj.default_collision_coeff_));

  // Serialize as string-based format for backwards compatibility
  std::map<std::pair<std::string, std::string>, double> compat;
  for (const auto& [key, coeff] : obj.lookup_table_)
    compat[{ key.first.name(), key.second.name() }] = coeff;
  ar(cereal::make_nvp("lookup_table", compat));

  std::set<std::pair<std::string, std::string>> zero_compat;
  for (const auto& id_pair : obj.zero_coeff_)
  {
    if (obj.lookup_table_.count(id_pair) > 0)
      zero_compat.insert({ id_pair.first.name(), id_pair.second.name() });
  }
  ar(cereal::make_nvp("zero_coeff", zero_compat));
}

template <class Archive>
void load(Archive& ar, CollisionCoeffData& obj)
{
  ar(cereal::make_nvp("default_collision_coeff", obj.default_collision_coeff_));

  std::map<std::pair<std::string, std::string>, double> compat;
  ar(cereal::make_nvp("lookup_table", compat));
  for (const auto& [names, coeff] : compat)
    obj.setCollisionCoeff(names.first, names.second, coeff);

  // zero_coeff_ is rebuilt by setCollisionCoeff, so just consume the archive field
  std::set<std::pair<std::string, std::string>> zero_compat;
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
