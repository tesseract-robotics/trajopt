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
void tripletsToCSC(vector<c_int>& row_indices,
                   vector<c_int>& column_pointers,
                   vector<double>& values,
                   const int m_size,
                   const int n_size,
                   const int n_nonzero,
                   const vector<int>& data_i,
                   const vector<int>& data_j,
                   const vector<double>& data_vij,
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
  vector<Var> vars_;
  vector<Cnt> cnts_;
  vector<double> lbs_, ubs_;
  vector<AffExpr> cnt_exprs_;
  vector<ConstraintType> cnt_types_;
  vector<double> solution_;

  vector<c_int> P_row_indices_;
  vector<c_int> P_column_pointers_;
  vector<double> P_csc_data_;
  vector<double> q_;

  vector<c_int> A_row_indices_;
  vector<c_int> A_column_pointers_;
  vector<double> A_csc_data_;
  vector<double> l_, u_;

  QuadExpr objective_;

  OSQPModel();
  virtual ~OSQPModel();

  Var addVar(const string& name);
  Cnt addEqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const AffExpr&, const string& name);
  Cnt addIneqCnt(const QuadExpr&, const string& name);
  void removeVars(const VarVector& vars);
  void removeCnts(const vector<Cnt>& cnts);

  void update();
  void setVarBounds(const vector<Var>& vars, const vector<double>& lower, const vector<double>& upper);
  vector<double> getVarValues(const VarVector& vars) const;
  virtual CvxOptStatus optimize();
  virtual void setObjective(const AffExpr&);
  virtual void setObjective(const QuadExpr&);
  virtual void writeToFile(const string& fname);
  virtual VarVector getVars() const;
};
}
