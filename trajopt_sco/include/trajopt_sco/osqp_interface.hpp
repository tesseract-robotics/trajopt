#include <osqp.h>
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
void tripletsToCSC(std::vector<c_int>& row_indices,
                   std::vector<c_int>& column_pointers,
                   DblVec& values,
                   const int m_size,
                   const int n_size,
                   const int n_nonzero,
                   const IntVec& data_i,
                   const IntVec& data_j,
                   const DblVec& data_vij,
                   bool only_upper_triangular = false);

class OSQPModel : public Model
{
  OSQPSettings osqp_settings_;     // OSQP Settings
  OSQPData osqp_data_;             // OSQPData
  OSQPWorkspace* osqp_workspace_;  // OSQP Workspace

  void updateObjective();
  void updateConstraints();
  void createOrUpdateSolver();

public:
  VarVector vars_;
  CntVector cnts_;
  DblVec lbs_, ubs_;
  AffExprVector cnt_exprs_;
  ConstraintTypeVector cnt_types_;
  DblVec solution_;

  std::vector<c_int> P_row_indices_;
  std::vector<c_int> P_column_pointers_;
  DblVec P_csc_data_;
  DblVec q_;

  std::vector<c_int> A_row_indices_;
  std::vector<c_int> A_column_pointers_;
  DblVec A_csc_data_;
  DblVec l_, u_;

  QuadExpr objective_;

  OSQPModel();
  virtual ~OSQPModel();

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
