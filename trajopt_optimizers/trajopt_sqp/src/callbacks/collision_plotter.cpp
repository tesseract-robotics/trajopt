#include <trajopt_sqp/callbacks/collision_plotter.h>
#include <trajopt_ifopt/utils/trajopt_utils.h>
#include <trajopt/typedefs.hpp>

using namespace trajopt_sqp;

CollisionPlottingCallback::CollisionPlottingCallback(tesseract_visualization::Visualization::Ptr plotter)
  : plotter_(std::move(plotter))
{
}

void CollisionPlottingCallback::plot(const ifopt::Problem& /*nlp*/)
{
  std::cout << "Collision plotting has not been implemented. PRs welcome" << std::endl;
}

void CollisionPlottingCallback::addConstraintSet(
    const trajopt::CollisionConstraintIfopt::ConstPtr& collision_constraint)
{
  collision_constraints_.push_back(collision_constraint);
}

void CollisionPlottingCallback::addConstraintSet(
    const std::vector<trajopt::CollisionConstraintIfopt::ConstPtr>& collision_constraints)
{
  for (const auto& cnt : collision_constraints)
    collision_constraints_.push_back(cnt);
}

bool CollisionPlottingCallback::execute(const ifopt::Problem& nlp, const trajopt_sqp::SQPResults&)
{
  plot(nlp);
  return true;
}
