#ifndef __SOLVER_UTILS_HPP__
#define __SOLVER_UTILS_HPP__

#include <Eigen/Core>
#include <trajopt_sco/solver_interface.hpp>

namespace sco
{
/**
 * @brief converts a vector of AffExpr (affine expressions) to triplets 
 * representation
 * 
 * @param [in] rows_i row indices for input values
 * @param [in] cols_j column indices for input values
 * @param [in] values_ij input values
 * @param [out] constants
 */
void exprToTriplets(const AffExprVector& expr_vec,
                    IntVec& rows_i,
                    IntVec& cols_j,
                    DblVec& values_ij,
                    DblVec& constants);

/**
 * @brief converts a vector of QuadExpr (quadratic expressions) to triplets 
 * representation
 *
 * @param [in] rows_i row indices for input values
 * @param [in] cols_j column indices for input values
 * @param [in] values_ij input values
 * @param [out] constants
 */
void exprToTriplets(const QuadExpr& expr,
                    IntVec& rows_i,
                    IntVec& cols_j,
                    DblVec& values_ij,
                    DblVec& affine);
/**
 * @brief converts a triplet representation of a sparse matrix into compressed
 *        sparse column representation (CSC).
 * 
 * @param [in,out] out dense matrix
 * @param [in] data_i row indices for input values
 * @param [in] data_j column indices for input values
 * @param [in] data_vij input values

 */
void triplets_to_full(const IntVec& data_i,
                      const IntVec& data_j,
                      const DblVec& data_vij,
                      Eigen::Ref<Eigen::MatrixXd> out);

/**
 * @brief converts a triplet representation of a sparse matrix into compressed
 *        sparse column representation (CSC).
 * 
 * @param [out] row_indices row indices for a CSC matrix
 * @param [out] column_pointers column pointer for a CSC matrix
 * @param [out] values pointer to non-zero elements in CSC representation
 * @param [in] m_size number of rows of the sparse matrix
 * @param [in] n_size number of columns of the sparse matrix
 * @param [in] n_nonzero number of non zero elements in the matrix. It is used
 *                       to reserve elements in the internal representation of
 *                       the SparseMatrix
 * @param [in] data_i row indices for input values
 * @param [in] data_j column indices for input values
 * @param [in] data_vij input values
 */
void triplets_to_CSC(IntVec& row_indices,
                     IntVec& column_pointers,
                     DblVec& values,
                     const int m_size,
                     const int n_size,
                     const int n_nonzero,
                     const IntVec& data_i,
                     const IntVec& data_j,
                     const DblVec& data_vij,
                     bool only_upper_triangular = false);

}
#endif // __SOLVER_UTILS_HPP__