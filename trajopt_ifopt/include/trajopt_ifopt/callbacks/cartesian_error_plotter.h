#ifndef TRAJOPT_SQP_INCLUDE_CARTESIAN_ERROR_PLOTTING_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_CARTESIAN_ERROR_PLOTTING_CALLBACK_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <tesseract_visualization/visualization.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/types.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_ifopt/constraints/cartesian_position_constraint.h>

namespace trajopt_sqp
{
/**
 * @brief SQPCallback the error of a CartPosConstraint. It plots an axis at the target and the current position with an
 * arrow between them
 */
class CartesianErrorPlottingCallback : public trajopt_sqp::SQPCallback
{
public:
  using Ptr = std::shared_ptr<CartesianErrorPlottingCallback>;
  using ConstPtr = std::shared_ptr<const CartesianErrorPlottingCallback>;

  /**
   * @brief Callback constructor
   * @param plotter Plotter used to plot the error
   */
  CartesianErrorPlottingCallback(tesseract_visualization::Visualization::Ptr plotter);

  /**
   * @brief Plots the error
   * @param nlp Unused
   */
  void plot(const ifopt::Problem& nlp);

  /**
   * @brief addConstraintSet Adds a constraint set to be plotted
   * @param cart_position_cnt Constraint to be plotted
   */
  void addConstraintSet(const trajopt::CartPosConstraint::ConstPtr& cart_position_cnt);

  /**
   * @brief addConstraintSet Adds a vector of constraint sets to be plotted
   * @param cart_position_cnts Vector of constraints to be plotted
   */
  void addConstraintSet(const std::vector<trajopt::CartPosConstraint::ConstPtr>& cart_position_cnts);

  bool execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults& /*sqp_results*/) override;

protected:
  std::vector<trajopt::CartPosConstraint::ConstPtr> cart_position_cnts_;
  tesseract_visualization::Visualization::Ptr plotter_;
};
}  // namespace trajopt_sqp

#endif
