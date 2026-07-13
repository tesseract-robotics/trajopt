#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/Geometry>
#include <boost/format.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/utils.hpp>
#include <trajopt_sco/solver_interface.hpp>

namespace trajopt
{
TrajArray getTraj(const DblVec& x, const VarArray& vars)
{
  TrajArray out(vars.rows(), vars.cols());
  for (int i = 0; i < vars.rows(); ++i)
  {
    for (int j = 0; j < vars.cols(); ++j)
    {
      out(i, j) = vars(i, j).value(x);
    }
  }
  return out;
}

TrajArray getTraj(const DblVec& x, const AffArray& arr)
{
  Eigen::MatrixXd out(arr.rows(), arr.cols());
  for (int i = 0; i < arr.rows(); ++i)
  {
    for (int j = 0; j < arr.cols(); ++j)
    {
      out(i, j) = arr(i, j).value(x);
    }
  }
  return out;
}

void AddVarArrays(sco::OptProb& prob,
                  int rows,
                  const IntVec& cols,
                  const std::vector<std::string>& name_prefix,
                  const std::vector<VarArray*>& newvars)
{
  const std::size_t n_arr = name_prefix.size();
  assert(static_cast<unsigned>(n_arr) == newvars.size());

  std::vector<Eigen::MatrixXi> index(n_arr);
  for (std::size_t i = 0; i < n_arr; ++i)
  {
    newvars[i]->resize(rows, cols[i]);
    index[i].resize(rows, cols[i]);
  }

  std::vector<std::string> names;
  int var_idx = prob.getNumVars();
  for (int i = 0; i < rows; ++i)
  {
    for (std::size_t k = 0; k < n_arr; ++k)
    {
      for (int j = 0; j < cols[k]; ++j)
      {
        index[k](i, j) = var_idx;
        names.push_back((boost::format("%s_%i_%i") % name_prefix[k] % i % j).str());
        ++var_idx;
      }
    }
  }
  prob.createVariables(names);  // note that w,r, are both unbounded

  const std::vector<sco::Var>& vars = prob.getVars();
  for (std::size_t k = 0; k < n_arr; ++k)
  {
    for (int i = 0; i < rows; ++i)
    {
      for (int j = 0; j < cols[k]; ++j)
      {
        (*newvars[k])(i, j) = vars[static_cast<std::size_t>(index[k](i, j))];
      }
    }
  }
}

void AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars)
{
  const std::vector<VarArray*> arrs(1, &newvars);
  const std::vector<std::string> prefixes(1, name_prefix);
  const std::vector<int> colss(1, cols);
  AddVarArrays(prob, rows, colss, prefixes, arrs);
}

bool updateFromSubset(const std::vector<tesseract::common::JointId>& superset_ids,
                      const Eigen::VectorXd& superset_vals,
                      const std::vector<tesseract::common::JointId>& subset_ids,
                      const Eigen::VectorXd& subset_vals,
                      Eigen::Ref<Eigen::VectorXd> new_superset_vals)
{
  new_superset_vals = superset_vals;
  for (std::size_t i = 0; i < subset_ids.size(); ++i)
  {
    const tesseract::common::JointId& joint = subset_ids[i];
    auto it = std::find(superset_ids.begin(), superset_ids.end(), joint);
    if (it == superset_ids.end())
    {
      std::cout << "Failed to find joint '" << joint << "' in superset joint ids";
      return false;
    }
    auto idx = std::distance(superset_ids.begin(), it);
    new_superset_vals[idx] = subset_vals[static_cast<Eigen::Index>(i)];
  }

  return true;
}

bool getSubset(const std::vector<tesseract::common::JointId>& superset_ids,
               const Eigen::VectorXd& superset_vals,
               const std::vector<tesseract::common::JointId>& subset_ids,
               Eigen::Ref<Eigen::VectorXd> subset_vals)
{
  Eigen::VectorXd out(subset_ids.size());
  for (Eigen::Index i = 0; i < static_cast<Eigen::Index>(subset_ids.size()); ++i)
  {
    const tesseract::common::JointId& id = subset_ids[static_cast<std::size_t>(i)];

    auto it = std::find(superset_ids.begin(), superset_ids.end(), id);
    if (it != superset_ids.end())
    {
      auto idx = std::distance(superset_ids.begin(), it);
      out[i] = superset_vals[idx];
    }
    else
    {
      return false;
    }
  }

  subset_vals = out;
  return true;
}

}  // namespace trajopt
