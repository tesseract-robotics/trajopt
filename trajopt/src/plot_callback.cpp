#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <functional>
#include <set>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_utils/eigen_conversions.hpp>

namespace trajopt
{
void PlotCosts(const tesseract_visualization::Visualization::Ptr& plotter,
               const tesseract_environment::StateSolver::Ptr& state_solver,
               const std::vector<std::string>& joint_names,
               const std::vector<sco::Cost::Ptr>& costs,
               const std::vector<sco::Constraint::Ptr>& cnts,
               const VarArray& vars,
               const sco::OptResults& results)
{
  plotter->clear();

  for (const sco::Cost::Ptr& cost : costs)
  {
    if (auto* plt = dynamic_cast<Plotter*>(cost.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  for (const sco::Constraint::Ptr& cnt : cnts)
  {
    if (auto* plt = dynamic_cast<Plotter*>(cnt.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  auto traj = getTraj(results.x, vars);
  tesseract_common::JointTrajectory joint_trajectory;
  for (long i = 0; i < traj.rows(); ++i)
    joint_trajectory.emplace_back(joint_names, traj.row(i));

  plotter->plotTrajectory(joint_trajectory, state_solver);
  plotter->waitForInput();
}

sco::Optimizer::Callback PlotCallback(TrajOptProb& prob, const tesseract_visualization::Visualization::Ptr& plotter)
{
  std::vector<sco::Constraint::Ptr> cnts = prob.getConstraints();
  return std::bind(&PlotCosts,
                   plotter,
                   prob.GetEnv()->getStateSolver(),
                   prob.GetKin()->getJointNames(),
                   std::ref(prob.getCosts()),
                   cnts,
                   std::ref(prob.GetVars()),
                   std::placeholders::_2);
}

void PlotProb(const tesseract_visualization::Visualization::Ptr& plotter,
              const tesseract_environment::StateSolver::Ptr& state_solver,
              const std::vector<std::string>& joint_names,
              sco::OptProb* prob,
              const sco::OptResults& results)
{
  plotter->clear();

  for (const sco::Cost::Ptr& cost : prob->getCosts())
  {
    if (auto* plt = dynamic_cast<Plotter*>(cost.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }

  for (const sco::Constraint::Ptr& cnt : prob->getConstraints())
  {
    if (auto* plt = dynamic_cast<Plotter*>(cnt.get()))
    {
      plt->Plot(plotter, results.x);
    }
  }
  auto var_vec = prob->getVars();
  // This probably is/should be a utility somewhere
  VarArray var_array;
  var_array.m_data = var_vec;
  var_array.m_nCol = static_cast<int>(joint_names.size());
  var_array.m_nRow = static_cast<int>(var_vec.size()) / var_array.cols();

  auto traj = getTraj(results.x, var_array);
  tesseract_common::JointTrajectory joint_trajectory;
  for (long i = 0; i < traj.rows(); ++i)
    joint_trajectory.emplace_back(joint_names, traj.row(i));

  plotter->plotTrajectory(joint_trajectory, state_solver);
  plotter->waitForInput();
}

sco::Optimizer::Callback PlotProbCallback(const tesseract_visualization::Visualization::Ptr& plotter,
                                          const tesseract_environment::StateSolver::Ptr& state_solver,
                                          const std::vector<std::string>& joint_names)
{
  return std::bind(&PlotProb, plotter, state_solver, joint_names, std::placeholders::_1, std::placeholders::_2);
}

}  // namespace trajopt
