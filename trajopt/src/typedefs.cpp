
#include <trajopt/typedefs.hpp>

namespace trajopt
{
thread_local tesseract_common::TransformMap TrajOptVectorOfVector::transforms_cache;
thread_local tesseract_common::TransformMap TrajOptMatrixOfVector::transforms_cache;
}  // namespace trajopt
