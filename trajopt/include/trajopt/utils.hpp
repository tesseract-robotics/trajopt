#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <unordered_map>
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

inline DblVec trajToDblVec(const TrajArray& x) { return DblVec(x.data(), x.data() + x.rows() * x.cols()); }

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
bool isSuperset(const std::vector<std::string>& subset, const std::vector<std::string>& superset);

/**
 * @brief Updates a superset of joint values with those from a subset of joint values
 * @param superset_names: Vector of names of the superset variables
 * @param superset_vals: Values of the superset
 * @param subset_names: Vector of names of the subset variables
 * @param subset_vals: Values of the subset
 * @param new_superset_vals: [output] the updated superset values
 * @return True on success, false on failure
 */
bool updateFromSubset(const std::vector<std::string>& superset_names,
                      const Eigen::VectorXd& superset_vals,
                      const std::vector<std::string>& subset_names,
                      const Eigen::VectorXd& subset_vals,
                      Eigen::Ref<Eigen::VectorXd> new_superset_vals);

/**
 * @brief Gets the subset of joint values from a superset of joint values
 * @param superset_names: Vector of superset variable names
 * @param superset_vals: Vector of superset variable values
 * @param subset_names: Vector of subset variable names
 * @param subset_vals [output]: Subset variable values
 * @return True on success, false on failure
 */
bool getSubset(const std::vector<std::string>& superset_names,
               const Eigen::VectorXd& superset_vals,
               const std::vector<std::string>& subset_names,
               Eigen::Ref<Eigen::VectorXd> subset_vals);

}  // namespace trajopt
