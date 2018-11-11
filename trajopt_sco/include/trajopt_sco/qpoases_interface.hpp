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
void tripletsToCSC(vector<int>& row_indices,
                   vector<int>& column_pointers,
                   vector<double>& values,
                   const int m_size,
                   const int n_size,
                   const int n_nonzero,
                   const vector<int>& data_i,
                   const vector<int>& data_j,
                   const vector<double>& data_vij,
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
  vector<Var> vars_;
  vector<Cnt> cnts_;
  vector<double> lb_, ub_;
  vector<AffExpr> cnt_exprs_;
  vector<ConstraintType> cnt_types_;
  vector<double> solution_;

  vector<int> H_row_indices_;
  vector<int> H_column_pointers_;
  vector<double> H_csc_data_;
  vector<double> g_;

  vector<int> A_row_indices_;
  vector<int> A_column_pointers_;
  vector<double> A_csc_data_;
  vector<double> lbA_, ubA_;

  QuadExpr objective_;

  qpOASESModel();
  virtual ~qpOASESModel();

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
