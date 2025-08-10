
#include <trajopt/typedefs.hpp>

namespace trajopt
{
thread_local tesseract_common::TransformMap TrajOptVectorOfVector::transforms_cache;  // NOLINT
thread_local tesseract_common::TransformMap TrajOptMatrixOfVector::transforms_cache;  // NOLINT
}  // namespace trajopt
