#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
TRAJOPT_IGNORE_WARNINGS_POP

namespace trajopt_common
{
inline float randf() { return (float)rand() / (float)RAND_MAX; }
}  // namespace trajopt_common
