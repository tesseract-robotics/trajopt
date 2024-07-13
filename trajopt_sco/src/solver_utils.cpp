#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/SparseCore>
#include <sstream>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_common/logging.hpp>

namespace sco
{
void exprToEigen(const AffExpr& expr, Eigen::SparseVector<double>& sparse_vector, const Eigen::Index& n_vars)
{
  sparse_vector.resize(n_vars);
  sparse_vector.reserve(static_cast<long int>(expr.size()));

  std::vector<std::pair<int, double>> doublets;
  doublets.reserve(expr.size());

  for (std::size_t i = 0; i < expr.size(); ++i)
  {
    auto i_var_index = static_cast<int>(expr.vars[i].var_rep->index);
    if (i_var_index >= n_vars)
    {
      std::stringstream msg;
      msg << "Coefficient " << i << "has index " << i_var_index << " but n_vars is " << n_vars;
      throw std::runtime_error(msg.str());
    }
    if (expr.coeffs[i] != 0.)
      doublets.emplace_back(i_var_index, expr.coeffs[i]);
  }

  std::sort(doublets.begin(), doublets.end(), [](const auto& lhs, const auto& rhs) { return lhs.first < rhs.first; });
  int prev_index{ -1 };
  for (const auto& doublet : doublets)
  {
    if (doublet.first == prev_index)
    {
      sparse_vector.coeffRef(doublet.first) += doublet.second;
    }
    else
    {
      sparse_vector.insertBack(doublet.first) = doublet.second;
    }
    prev_index = doublet.first;
  }
}

void exprToEigen(const QuadExpr& expr,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 const Eigen::Index& n_vars,
                 const bool& matrix_is_halved,
                 const bool& force_diagonal)
{
  SizeTVec ind1;
  vars2inds(expr.vars1, ind1);
  SizeTVec ind2;
  vars2inds(expr.vars2, ind2);
  assert((ind2.size() == ind1.size()) && (ind1.size() == expr.coeffs.size()));  // NOLINT
  sparse_matrix.resize(n_vars, n_vars);

  Eigen::SparseVector<double> vector_sparse;
  exprToEigen(expr.affexpr, vector_sparse, n_vars);
  vector = vector_sparse;

  using T = Eigen::Triplet<double>;
  thread_local std::vector<T, Eigen::aligned_allocator<T>> triplets;
  triplets.clear();

  for (std::size_t i = 0; i < expr.coeffs.size(); ++i)
  {
    if (expr.coeffs[i] != 0.0)
    {
      // NOLINTNEXTLINE
      if (ind1[i] == ind2[i])
      {
        triplets.emplace_back(static_cast<Eigen::Index>(ind1[i]), static_cast<Eigen::Index>(ind2[i]), expr.coeffs[i]);
      }
      else
      {
        Eigen::Index c{ 0 }, r{ 0 };
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
        triplets.emplace_back(r, c, expr.coeffs[i]);
      }
    }
  }

  if (force_diagonal)
    for (int k = 0; k < n_vars; ++k)
      triplets.emplace_back(k, k, 0.0);

  sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());

  auto sparse_matrix_T = Eigen::SparseMatrix<double>(sparse_matrix.transpose());
  sparse_matrix = sparse_matrix + sparse_matrix_T;

  if (!matrix_is_halved)
    sparse_matrix = 0.5 * sparse_matrix;
}

void exprToEigen(const AffExprVector& expr_vec,
                 Eigen::SparseMatrix<double>& sparse_matrix,
                 Eigen::VectorXd& vector,
                 Eigen::Index n_vars)
{
  vector.resize(static_cast<long int>(expr_vec.size()));
  vector.setZero();
  sparse_matrix.resize(static_cast<long int>(expr_vec.size()), n_vars);

  using T = Eigen::Triplet<double>;
  thread_local std::vector<T, Eigen::aligned_allocator<T>> triplets;
  triplets.clear();

  for (int i = 0; i < static_cast<int>(expr_vec.size()); ++i)
  {
    const AffExpr& expr = expr_vec[static_cast<std::size_t>(i)];
    vector[i] = -expr.constant;

    for (std::size_t j = 0; j < expr.size(); ++j)
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
  sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());  // NOLINT
}

void tripletsToEigen(const IntVec& rows_i,
                     const IntVec& cols_j,
                     const DblVec& values_ij,
                     Eigen::SparseMatrix<double>& sparse_matrix)
{
  assert((rows_i.size() == cols_j.size()) && (rows_i.size() == values_ij.size()));  // NOLINT
  sparse_matrix.reserve(static_cast<Eigen::Index>(values_ij.size()));

  using T = Eigen::Triplet<double>;
  thread_local std::vector<T, Eigen::aligned_allocator<T>> triplets;
  triplets.clear();
  triplets.reserve(values_ij.size());

  // NOLINTNEXTLINE
  for (unsigned int i = 0; i < values_ij.size(); ++i)
    triplets.emplace_back(rows_i[i], cols_j[i], values_ij[i]);

  sparse_matrix.setFromTriplets(triplets.begin(), triplets.end());
}

void eigenToTriplets(const Eigen::SparseMatrix<double>& sparse_matrix,
                     IntVec& rows_i,
                     IntVec& cols_j,
                     DblVec& values_ij)
{
  const auto& sm = sparse_matrix;
  rows_i.reserve(rows_i.size() + static_cast<std::size_t>(sm.nonZeros()));
  cols_j.reserve(cols_j.size() + static_cast<std::size_t>(sm.nonZeros()));
  values_ij.reserve(values_ij.size() + static_cast<std::size_t>(sm.nonZeros()));
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
