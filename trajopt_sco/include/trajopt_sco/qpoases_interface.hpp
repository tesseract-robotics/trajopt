#pragma once
#include <qpOASES.hpp>
#include <trajopt_sco/solver_interface.hpp>
#include <trajopt_utils/macros.h>

namespace sco
{
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
void tripletsToCSC(IntVec& row_indices,
                   IntVec& column_pointers,
                   DblVec& values,
                   const int m_size,
                   const int n_size,
                   const int n_nonzero,
                   const IntVec& data_i,
                   const IntVec& data_j,
                   const DblVec& data_vij,
                   bool only_upper_triangular = false);

class qpOASESModel : public Model
{
  std::shared_ptr<qpOASES::SQProblem> qpoases_problem_;
  std::shared_ptr<qpOASES::Constraints> qpoases_constraints_;
  std::shared_ptr<qpOASES::Bounds> qpoases_bounds_;
  qpOASES::Options qpoases_options_;

  qpOASES::SymSparseMat H_; /**< Quadratic cost matrix */
  qpOASES::SparseMatrix A_; /**< Constraints matrix */

  void updateObjective();
  void updateConstraints();
  /**
   * Instantiates a new qpOASES problem if it has not been instantiated yet
   * or if the size of the problem has changed.
   *
   * @returns true if a new qpOASES problem has been instantiated
   */
  bool updateSolver();

  /**
   * Instantiates a new qpOASES problem
   */
  void createSolver();

public:
  VarVector vars_;
  CntVector cnts_;
  DblVec lb_, ub_;
  AffExprVector cnt_exprs_;
  ConstraintTypeVector cnt_types_;
  DblVec solution_;

  IntVec H_row_indices_;
  IntVec H_column_pointers_;
  DblVec H_csc_data_;
  DblVec g_;

  IntVec A_row_indices_;
  IntVec A_column_pointers_;
  DblVec A_csc_data_;
  DblVec lbA_, ubA_;

  QuadExpr objective_;

  qpOASESModel();
  virtual ~qpOASESModel();

  Var addVar(const std::string& name);
  Cnt addEqCnt(const AffExpr&, const std::string& name);
  Cnt addIneqCnt(const AffExpr&, const std::string& name);
  Cnt addIneqCnt(const QuadExpr&, const std::string& name);
  void removeVars(const VarVector& vars);
  void removeCnts(const CntVector& cnts);

  void update();
  void setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper);
  DblVec getVarValues(const VarVector& vars) const;
  virtual CvxOptStatus optimize();
  virtual void setObjective(const AffExpr&);
  virtual void setObjective(const QuadExpr&);
  virtual void writeToFile(const std::string& fname);
  virtual VarVector getVars() const;
};
}
