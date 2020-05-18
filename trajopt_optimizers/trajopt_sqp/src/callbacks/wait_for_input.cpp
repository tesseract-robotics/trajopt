#include <trajopt_sqp/callbacks/wait_for_input.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

WaitForInputCallback::WaitForInputCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

bool WaitForInputCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults&)
{
  plotter_->waitForInput();
  return true;
}
