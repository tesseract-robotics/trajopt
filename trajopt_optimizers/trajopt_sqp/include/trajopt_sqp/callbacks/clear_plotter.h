#ifndef TRAJOPT_SQP_INCLUDE_CLEAR_PLOTTER_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_CLEAR_PLOTTER_CALLBACK_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <tesseract_visualization/visualization.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
/**
 * @brief Clears the plotter passed in. Add this callback before plotting callbacks to clear the plotter before plotting
 * new results
 */
class ClearPlotterCallback : public trajopt_sqp::SQPCallback
{
public:
  using Ptr = std::shared_ptr<ClearPlotterCallback>;
  using ConstPtr = std::shared_ptr<const ClearPlotterCallback>;

  ClearPlotterCallback(tesseract_visualization::Visualization::Ptr plotter);

  bool execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults& sqp_results) override;

protected:
  tesseract_visualization::Visualization::Ptr plotter_;
};
}  // namespace trajopt_sqp

#endif
