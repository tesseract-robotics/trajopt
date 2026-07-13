#pragma once
#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <Eigen/Geometry>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/typedefs.hpp>

namespace trajopt
{
/**
Extract trajectory array from solution vector x using indices in array vars
*/
TrajArray getTraj(const DblVec& x, const VarArray& vars);
TrajArray getTraj(const DblVec& x, const AffArray& arr);

inline DblVec trajToDblVec(const TrajArray& x) { return { x.data(), x.data() + (x.rows() * x.cols()) }; }

template <typename T>
std::vector<T> singleton(const T& x)
{
  return std::vector<T>(1, x);
}

void AddVarArrays(sco::OptProb& prob,
                  int rows,
                  const std::vector<int>& cols,
                  const std::vector<std::string>& name_prefix,
                  const std::vector<VarArray*>& newvars);

void AddVarArray(sco::OptProb& prob, int rows, int cols, const std::string& name_prefix, VarArray& newvars);

/**
 * @brief Determines if the input superset includes the input subset
 * @param subset
 * @param superset
 * @return
 */
template <typename T>
bool isSuperset(const std::vector<T>& subset, const std::vector<T>& superset)
{
  for (const T& s : subset)
  {
    if (std::find(superset.begin(), superset.end(), s) == superset.end())
      return false;
  }

  // All subset elements were found in the superset.
  // Check that the superset is bigger than the subset.
  return superset.size() > subset.size();
}

/**
 * @brief Updates a superset of joint values with those from a subset of joint values
 * @param superset_ids: Vector of ids of the superset variables
 * @param superset_vals: Values of the superset
 * @param subset_ids: Vector of ids of the subset variables
 * @param subset_vals: Values of the subset
 * @param new_superset_vals: [output] the updated superset values
 * @return True on success, false on failure
 */
bool updateFromSubset(const std::vector<tesseract::common::JointId>& superset_ids,
                      const Eigen::VectorXd& superset_vals,
                      const std::vector<tesseract::common::JointId>& subset_ids,
                      const Eigen::VectorXd& subset_vals,
                      Eigen::Ref<Eigen::VectorXd> new_superset_vals);

/**
 * @brief Gets the subset of joint values from a superset of joint values
 * @param superset_ids: Vector of superset variable ids
 * @param superset_vals: Vector of superset variable values
 * @param subset_ids: Vector of subset variable ids
 * @param subset_vals [output]: Subset variable values
 * @return True on success, false on failure
 */
bool getSubset(const std::vector<tesseract::common::JointId>& superset_ids,
               const Eigen::VectorXd& superset_vals,
               const std::vector<tesseract::common::JointId>& subset_ids,
               Eigen::Ref<Eigen::VectorXd> subset_vals);

}  // namespace trajopt
