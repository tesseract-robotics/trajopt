#ifndef TRAJOPT_SQP_INCLUDE_COLLISION_PLOTTING_CALLBACK_H_
#define TRAJOPT_SQP_INCLUDE_COLLISION_PLOTTING_CALLBACK_H_

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ifopt/problem.h>
#include <tesseract_visualization/visualization.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ifopt/constraints/collision_constraint.h>
#include <trajopt_sqp/sqp_callback.h>
#include <trajopt_sqp/trust_region_sqp_solver.h>
#include <trajopt_sqp/types.h>

namespace trajopt_sqp
{
class CollisionPlottingCallback : public trajopt_sqp::SQPCallback
{
public:
  using Ptr = std::shared_ptr<CollisionPlottingCallback>;
  using ConstPtr = std::shared_ptr<const CollisionPlottingCallback>;

  CollisionPlottingCallback(tesseract_visualization::Visualization::Ptr plotter);

  void plot(const ifopt::Problem& nlp);

  void addConstraintSet(const trajopt::CollisionConstraintIfopt::ConstPtr& collision_constraint);

  void addConstraintSet(const std::vector<trajopt::CollisionConstraintIfopt::ConstPtr>& collision_constraints);

  bool execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults& sqp_results) override;

protected:
  std::vector<trajopt::CollisionConstraintIfopt::ConstPtr> collision_constraints_;
  tesseract_visualization::Visualization::Ptr plotter_;
};
}  // namespace trajopt_sqp

#endif
