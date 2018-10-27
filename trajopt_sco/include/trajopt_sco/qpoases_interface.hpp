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
void triplets_to_CSC(vector<int>& row_indices,
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
  std::shared_ptr<qpOASES::SQProblem> m_problem;
  std::shared_ptr<qpOASES::Constraints> m_constraints;
  std::shared_ptr<qpOASES::Bounds> m_bounds;
  qpOASES::Options m_options;

  qpOASES::SymSparseMat m_H;
  qpOASES::SparseMatrix m_A;

  void update_objective();
  void update_constraints();
  void create_solver();

public:
  vector<Var> m_vars;
  vector<Cnt> m_cnts;
  vector<double> m_lb, m_ub;
  vector<AffExpr> m_cntExprs;
  vector<ConstraintType> m_cntTypes;
  vector<double> m_soln;


  vector<int> m_H_row_indices;
  vector<int> m_H_column_pointers;
  vector<double> m_H_csc_data;
  vector<double> m_g;

  vector<int> m_A_row_indices;
  vector<int> m_A_column_pointers;
  vector<double> m_A_csc_data;
  vector<double> m_lbA, m_ubA;

  QuadExpr m_objective;

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
