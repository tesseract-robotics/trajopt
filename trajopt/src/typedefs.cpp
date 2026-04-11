
#include <trajopt/typedefs.hpp>

namespace trajopt
{
thread_local tesseract::common::LinkIdTransformMap TrajOptVectorOfVector::transforms_cache;  // NOLINT
thread_local tesseract::common::LinkIdTransformMap TrajOptMatrixOfVector::transforms_cache;  // NOLINT
}  // namespace trajopt
