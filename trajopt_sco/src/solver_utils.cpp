#include <trajopt_sco/solver_utils.hpp>
#include <Eigen/SparseCore>

namespace sco
{
extern IntVec vars2inds(const VarVector& vars);

void exprToTriplets(const AffExprVector& expr_vec,
                    IntVec& rows_i,
                    IntVec& cols_j,
                    DblVec& values_ij,
                    DblVec& constants)
{
  constants.clear();
  constants.resize(expr_vec.size(), 0.0);
  
  for (size_t i = 0; i < expr_vec.size(); ++i)
  {
    const AffExpr& expr = expr_vec[i];

    rows_i.reserve(rows_i.size() + expr.size());
    cols_j.reserve(cols_j.size() + expr.size());
    values_ij.reserve(values_ij.size() + expr.size());

    IntVec data_j = vars2inds(expr.vars);

    for (size_t j = 0; j < expr.size(); ++j)
    {
      if(expr.coeffs[j] != 0.0)
      {
        rows_i.push_back(i);
        cols_j.push_back(data_j[j]);
        values_ij.push_back(expr.coeffs[j]);
      }
    }
    constants[i] = -expr.constant;
  }
}

void exprToTriplets(const QuadExpr& expr,
                    IntVec& rows_i,
                    IntVec& cols_j,
                    DblVec& values_ij,
                    DblVec& affine)
{
  affine.clear();
  affine.resize(expr.affexpr.size(), 0.0);
  for (size_t i = 0; i < expr.affexpr.size(); ++i)
  {
    affine[expr.affexpr.vars[i].var_rep->index] += expr.affexpr.coeffs[i];
  }

  IntVec ind1 = vars2inds(expr.vars1);
  IntVec ind2 = vars2inds(expr.vars2);
  Eigen::SparseMatrix<double> sm(expr.size(), expr.size());
  sm.reserve(expr.size() * expr.size());

  for (size_t i = 0; i < expr.coeffs.size(); ++i)
  {
    if (expr.coeffs[i] != 0.0)
    {
      if (ind1[i] == ind2[i])
        sm.coeffRef(ind1[i], ind2[i]) += expr.coeffs[i];
      else
      {
        int c, r;
        if (ind1[i] < ind2[i])
        {
          r = ind1[i];
          c = ind2[i];
        }
        else
        {
          r = ind2[i];
          c = ind1[i];
        }
        sm.coeffRef(r, c) += expr.coeffs[i];
      }
    }
  }

  sm = sm + Eigen::SparseMatrix<double>(sm.transpose());
  sm.makeCompressed();

  rows_i.reserve(rows_i.size() + expr.size());
  cols_j.reserve(cols_j.size() + expr.size());
  values_ij.reserve(values_ij.size() + expr.size() * expr.size());
  for (int k = 0; k < sm.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sm, k); it; ++it)
    {
      rows_i.push_back(it.row());  // row index
      cols_j.push_back(it.col());  // col index
      values_ij.push_back(0.5 * it.value());
    }
  }
}

void triplets_to_full(const IntVec& data_i,
                      const IntVec& data_j,
                      const DblVec& data_vij,
                      Eigen::Ref<Eigen::MatrixXd> out)
{
  out.setZero();

  for(unsigned int k = 0; k < data_vij.size(); ++k)
  {
      if(data_vij[k] != 0.0)
      {
          out(data_i[k],data_j[k]) += data_vij[k];
      }
  }
}

void triplets_to_CSC(IntVec& row_indices,
                     IntVec& column_pointers,
                     DblVec& values,
                     const int m_size,
                     const int n_size,
                     const int n_nonzero,
                     const IntVec& data_i,
                     const IntVec& data_j,
                     const DblVec& data_vij,
                     bool only_upper_triangular)
{
  Eigen::SparseMatrix<double> sm(m_size, n_size);
  sm.reserve(n_nonzero);
  
  for(unsigned int k = 0; k < data_vij.size(); ++k)
  {
      if(data_vij[k] != 0.0)
      {   // TODO since we are not using simplify2, apparently there are cases
          // when we try to add the same data twice. We should troubleshoot 
          // why this is the case in the first place - using coeffRef instead
          // of insert for now
          sm.coeffRef(data_i[k], data_j[k]) += data_vij[k];
      }
  }
  
  Eigen::SparseMatrix<double> sm_t;
  Eigen::SparseMatrix<double>* sm_view;
  
  if(only_upper_triangular)
  {
      sm_t = sm.triangularView<Eigen::Upper>();
      sm_t.makeCompressed();
      sm_view = &sm_t;
  }
  else
  {
      sm.makeCompressed();
      sm_view = &sm;
  }
  
  Eigen::SparseMatrix<double>::StorageIndex* si_p;
  double* csc_v;

  si_p = sm_view->innerIndexPtr();
  row_indices.assign(si_p, si_p + sm_view->nonZeros());
  
  si_p = sm_view->outerIndexPtr();
  column_pointers.assign(si_p, si_p + sm_view->outerSize());
  
  // while Eigen does not enforce this, CSC format requires that column
  // pointers ends with the number of non-zero elements
  column_pointers.push_back(sm_view->nonZeros());
  
  csc_v = sm_view->valuePtr();
  values.assign(csc_v, csc_v + sm_view->nonZeros());
}

}