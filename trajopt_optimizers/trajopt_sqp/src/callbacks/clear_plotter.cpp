#include <trajopt_sqp/callbacks/clear_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

ClearPlotterCallback::ClearPlotterCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

bool ClearPlotterCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults&)
{
  plotter_->clear();
  return true;
}
