/**
 * @file name_id_testing.h
 * @brief Test-only access to NameId internals
 *
 * @author Roelof Oomen
 * @date July 10, 2026
 *
 * @copyright Copyright (c) 2026, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TRAJOPT_COMMON_TEST_NAME_ID_TESTING_H
#define TRAJOPT_COMMON_TEST_NAME_ID_TESTING_H

#include <tesseract/common/types.h>

namespace tesseract::common
{
struct NameIdTestAccess
{
  /** @brief Construct an id with an explicit value/name combination, e.g. a manufactured hash collision. */
  template <typename IdT>
  [[nodiscard]] static IdT create(NameIdValue value, std::string name)
  {
    IdT id;
    id.value_ = value;
    id.name_ = std::move(name);
    return id;
  }
};
}  // namespace tesseract::common

#endif  // TRAJOPT_COMMON_TEST_NAME_ID_TESTING_H
