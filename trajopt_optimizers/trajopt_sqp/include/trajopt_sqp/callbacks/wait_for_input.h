#ifndef TRAJOPT_SQP_INCLUDE_WAIT_FOR_INPUT_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_WAIT_FOR_INPUT_CALLBACK_H_

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
 * @brief Calls the waitForInput function in the plotter. Use to pause the optimization while displaying intermediate
 * results
 */
class WaitForInputCallback : public trajopt_sqp::SQPCallback
{
public:
  using Ptr = std::shared_ptr<WaitForInputCallback>;
  using ConstPtr = std::shared_ptr<const WaitForInputCallback>;

  WaitForInputCallback(tesseract_visualization::Visualization::Ptr plotter);

  bool execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults& sqp_results) override;

protected:
  tesseract_visualization::Visualization::Ptr plotter_;
};
}  // namespace trajopt_sqp

#endif
