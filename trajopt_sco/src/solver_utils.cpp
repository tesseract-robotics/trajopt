#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <Eigen/SparseCore>
#include <sstream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_utils/logging.hpp>

namespace sco
{
void exprToEigen(const AffExpr& expr, Eigen::SparseVector<double>& sparse_vector, const int& n_vars)
{
  sparse_vector.resize(n_vars);
  sparse_vector.reserve(static_cast<long int>(expr.size()));
  for (size_t i = 0; i < expr.size(); ++i)
  {
    auto i_var_index = static_cast<int>(expr.vars[i].var_rep->index);
    if (i_var_index >= n_vars)
    {
      std::stringstream msg;
      msg << "Coefficient " << i << "has index " << i_var_index << " but n_vars is " << n_vars;
      throw std::runtime_error(msg.str());
    }
    if (expr.coeffs[i] != 0.)
      sparse_vector.coeffRef(i_var_index) += expr.coeffs[i];
  }
}

void exprToEigen(const QuadExpr& expr,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const int& n_vars,
                 const bool& matrix_is_halved,
                 const bool& force_diagonal)
{
  SizeTVec ind1;
  vars2inds(expr.vars1, ind1);
  SizeTVec ind2;
  vars2inds(expr.vars2, ind2);
  sparse_matrix.resize(n_vars, n_vars);
  sparse_matrix.reserve(static_cast<long int>(2 * expr.size()));

  Eigen::SparseVector<double> vector_sparse;
  exprToEigen(expr.affexpr, vector_sparse, n_vars);
  vector = vector_sparse;

  for (size_t i = 0; i < expr.coeffs.size(); ++i)
  {
    if (expr.coeffs[i] != 0.0)
    {
      if (ind1[i] == ind2[i])
        sparse_matrix.coeffRef(static_cast<Eigen::Index>(ind1[i]), static_cast<Eigen::Index>(ind2[i])) +=
            expr.coeffs[i];
      else
      {
        Eigen::Index c, r;
        if (ind1[i] < ind2[i])
        {
          r = static_cast<Eigen::Index>(ind1[i]);
          c = static_cast<Eigen::Index>(ind2[i]);
        }
        else
        {
          r = static_cast<Eigen::Index>(ind2[i]);
          c = static_cast<Eigen::Index>(ind1[i]);
        }
        sparse_matrix.coeffRef(r, c) += expr.coeffs[i];
      }
    }
  }

  auto sparse_matrix_T = Eigen::SparseMatrix<double>(sparse_matrix.transpose());
  sparse_matrix = sparse_matrix + sparse_matrix_T;

  if (!matrix_is_halved)
    sparse_matrix = 0.5 * sparse_matrix;

  if (force_diagonal)
    for (int k = 0; k < n_vars; ++k)
      sparse_matrix.coeffRef(k, k) += 0.0;
}

void exprToEigen(const AffExprVector& expr_vec,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const int& n_vars)
{
  vector.resize(static_cast<long int>(expr_vec.size()));
  vector.setZero();
  sparse_matrix.resize(static_cast<long int>(expr_vec.size()), n_vars);
  sparse_matrix.reserve(static_cast<long int>(expr_vec.size()) * n_vars);

  using T = Eigen::Triplet<double>;
  std::vector<T, Eigen::aligned_allocator<T>> triplets;

  for (int i = 0; i < static_cast<int>(expr_vec.size()); ++i)
  {
    const AffExpr& expr = expr_vec[static_cast<size_t>(i)];
    vector[i] = -expr.constant;

    for (size_t j = 0; j < expr.size(); ++j)
    {
      auto i_var_index = static_cast<int>(expr.vars[j].var_rep->index);
      if (i_var_index >= n_vars)
      {
        std::stringstream msg;
        msg << "Coefficient " << i << "has index " << i_var_index << " but n_vars is " << n_vars;
        throw std::runtime_error(msg.str());
      }
      if (expr.coeffs[j] != 0.)
      {
        triplets.emplace_back(i, i_var_index, expr.coeffs[j]);
      }
    }
  }
  sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());
}

void tripletsToEigen(const IntVec& rows_i,
                     const IntVec& cols_j,
                     const DblVec& values_ij,
                     Eigen::SparseMatrix<double>& sparse_matrix)
{
  using T = Eigen::Triplet<double>;
  std::vector<T, Eigen::aligned_allocator<T>> triplets;
  for (unsigned int i = 0; i < values_ij.size(); ++i)
  {
    triplets.emplace_back(rows_i[i], cols_j[i], values_ij[i]);
  }
  sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());
}

void eigenToTriplets(const Eigen::SparseMatrix<double>& sparse_matrix,
                     IntVec& rows_i,
                     IntVec& cols_j,
                     DblVec& values_ij)
{
  auto& sm = sparse_matrix;
  rows_i.reserve(rows_i.size() + static_cast<size_t>(sm.nonZeros()));
  cols_j.reserve(cols_j.size() + static_cast<size_t>(sm.nonZeros()));
  values_ij.reserve(values_ij.size() + static_cast<size_t>(sm.nonZeros()));
  for (int k = 0; k < sm.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sm, k); it; ++it)
    {
      rows_i.push_back(static_cast<int>(it.row()));
      cols_j.push_back(static_cast<int>(it.col()));
      values_ij.push_back(it.value());
    }
  }
}
}  // namespace sco
