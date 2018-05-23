#include "tesseract_planning/ompl/conversions.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

tesseract::TrajArray tesseract::tesseract_planning::toTrajArray(const ompl::geometric::PathGeometric &path)
{
  const auto n_points = path.getStateCount();
  const auto dof = path.getSpaceInformation()->getStateDimension();

  tesseract::TrajArray result (n_points, dof);
  for (std::size_t i = 0; i < n_points; ++i)
  {
    const auto& state = path.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>();
    for (std::size_t j = 0; j < dof; ++j)
    {
      result(i, j) = state->values[j];
    }
  }
  return result;
}
