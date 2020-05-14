#ifndef TRAJOPT_SQP_INCLUDE_SQP_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_SQP_CALLBACK_H_

#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
/**
 * @brief Base class for callbacks called during the SQP routine
 */
class SQPCallback
{
public:
  using Ptr = std::shared_ptr<SQPCallback>;
  using ConstPtr = std::shared_ptr<const SQPCallback>;

  /**
   * @brief This is the function called during the SQP
   * @param nlp The ifopt::Problem being optimized
   * @param sqp_results The latest SQPResults
   * @return Returning false will stop the optimization
   */
  virtual bool execute(const ifopt::Problem& nlp, const SQPResults& sqp_results) = 0;
};
}  // namespace trajopt_sqp

#endif
