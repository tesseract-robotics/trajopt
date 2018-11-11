#include <cmath>
#include <Eigen/Eigen>
#include <fstream>
#include <signal.h>
#include <trajopt_sco/qpoases_interface.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace std;
using namespace qpOASES;

namespace sco
{
double QPOASES_INFTY = qpOASES::INFTY;

extern void simplify2(vector<int>& inds, vector<double>& vals);
extern vector<int> vars2inds(const vector<Var>& vars);
extern vector<int> cnts2inds(const vector<Cnt>& cnts);

void triplets_to_CSC(vector<int>& row_indices,
                     vector<int>& column_pointers,
                     vector<double>& values,
                     const int m_size,
                     const int n_size,
                     const int n_nonzero,
                     const vector<int>& data_i,
                     const vector<int>& data_j,
                     const vector<double>& data_vij,
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
    
    for(unsigned int k = 0; k < std::min(m_size, n_size); ++k)
    {
      sm.coeffRef(k, k) += 0.0;
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

ModelPtr createqpOASESModel()
{
  ModelPtr out(new qpOASESModel());
  return out;
}

qpOASESModel::qpOASESModel()
{
    m_options.setToMPC();
    m_options.printLevel = qpOASES::PL_NONE;
    m_options.enableRegularisation = qpOASES::BT_TRUE;
    m_options.ensureConsistency();
}

qpOASESModel::~qpOASESModel()
{
}

Var qpOASESModel::addVar(const string& name)
{
  m_vars.push_back(new VarRep(m_vars.size(), name, this));
  m_lb.push_back(-QPOASES_INFTY);
  m_ub.push_back(QPOASES_INFTY);
  return m_vars.back();
}
Cnt qpOASESModel::addEqCnt(const AffExpr& expr, const string& /*name*/)
{
  m_cnts.push_back(new CntRep(m_cnts.size(), this));
  m_cntExprs.push_back(expr);
  m_cntTypes.push_back(EQ);
  return m_cnts.back();
}
Cnt qpOASESModel::addIneqCnt(const AffExpr& expr, const string& /*name*/)
{
  m_cnts.push_back(new CntRep(m_cnts.size(), this));
  m_cntExprs.push_back(expr);
  m_cntTypes.push_back(INEQ);
  return m_cnts.back();
}
Cnt qpOASESModel::addIneqCnt(const QuadExpr&, const string& /*name*/)
{
  assert(0 && "NOT IMPLEMENTED");
  return 0;
}
void qpOASESModel::removeVars(const VarVector& vars)
{
  vector<int> inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void qpOASESModel::removeCnts(const vector<Cnt>& cnts)
{
  vector<int> inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void qpOASESModel::update_objective()
{
  int n = m_vars.size();
  
  m_g.clear();
  m_g.resize(n, 0.0);
  for (size_t i = 0; i < m_objective.affexpr.size(); ++i)
  {
    m_g[m_objective.affexpr.vars[i].var_rep->index] += m_objective.affexpr.coeffs[i];
  }

  vector<int> ind1 = vars2inds(m_objective.vars1);
  vector<int> ind2 = vars2inds(m_objective.vars2);
  Eigen::SparseMatrix<double> sm(m_vars.size(), m_vars.size());
  sm.reserve(m_vars.size() * m_vars.size());

  for(size_t i = 0; i < m_objective.coeffs.size(); ++i)
  {
    if(m_objective.coeffs[i] != 0.0)
    {
      if(ind1[i] == ind2[i])
        sm.coeffRef(ind1[i], ind2[i]) += m_objective.coeffs[i];
      else
      {
        int c, r;
        if(ind1[i] < ind2[i])
        {
          r = ind1[i];
          c = ind2[i];
        }
        else
        {
          r = ind2[i];
          c = ind1[i];
        }
        sm.coeffRef(r, c) += m_objective.coeffs[i];
      }
    }
  }
  
  sm = sm + Eigen::SparseMatrix<double>(sm.transpose());
  sm.makeCompressed();
  
  vector<int> data_i;
  vector<int> data_j;
  vector<double> vals_ij;
  for (int k=0; k < sm.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sm, k); it; ++it)
    {
      data_i.push_back(it.row()); // row index
      data_j.push_back(it.col()); // col index
      vals_ij.push_back(it.value());
    }
  }

  vector<int> row_indices;
  vector<int> column_pointers;
  vector<double> csc_data;
  triplets_to_CSC(m_H_row_indices, m_H_column_pointers, m_H_csc_data,
                  m_vars.size(), m_vars.size(), n*n,
                  data_i, data_j, vals_ij);

  m_H = SymSparseMat(m_vars.size(), m_vars.size(),
                     m_H_row_indices.data(), m_H_column_pointers.data(),
                     m_H_csc_data.data());
  m_H.createDiagInfo();
}

void qpOASESModel::update_constraints()
{
  int n = m_vars.size();
  int m = m_cnts.size();

  m_lbA.clear();
  m_lbA.resize(m, -QPOASES_INFTY);
  m_ubA.clear();
  m_ubA.resize(m, QPOASES_INFTY);

  vector<int> data_i;
  vector<int> data_j;
  vector<double> data_ij;
  for (int iCnt = 0; iCnt < m; ++iCnt)
  {
    const AffExpr& aff = m_cntExprs[iCnt];
    vector<int> inds = vars2inds(aff.vars);
    vector<double> vals = aff.coeffs;

    for (unsigned i = 0; i < aff.vars.size(); ++i)
    {
      if(aff.coeffs[i] != 0)
      {
        data_i.push_back(iCnt);
        data_j.push_back(inds[i]);
        data_ij.push_back(aff.coeffs[i]);
      }
    }

    m_lbA[iCnt] = (m_cntTypes[iCnt] == INEQ) ? -QPOASES_INFTY : -aff.constant;
    m_ubA[iCnt] = -aff.constant;
  }

  triplets_to_CSC(m_A_row_indices, m_A_column_pointers, m_A_csc_data,
                  m_cnts.size() + m_vars.size(), m_vars.size(), m*n,
                  data_i, data_j, data_ij);

  m_A = SparseMatrix(m_cnts.size(), m_vars.size(),
                     m_A_row_indices.data(), m_A_column_pointers.data(),
                     m_A_csc_data.data());
}

bool qpOASESModel::updateSolver()
{
  bool solver_updated = false;
  if(!m_problem ||
     m_vars.size() != m_problem->getNV() ||
     m_cnts.size() != m_problem->getNC())
  {
    // Create Problem - this should be called only once
    m_problem.reset(new SQProblem(m_vars.size(), m_cnts.size()));
    m_problem->setOptions(m_options);
    solver_updated = true;
  }
  return solver_updated;
}

void qpOASESModel::createSolver()
{
  qpoases_problem_.reset();
  updateSolver();
}

void qpOASESModel::update()
{
  {
    int inew = 0;
    for (unsigned iold = 0; iold < m_vars.size(); ++iold)
    {
      const Var& var = m_vars[iold];
      if (!var.var_rep->removed)
      {
        m_vars[inew] = var;
        m_lb[inew] = m_lb[iold];
        m_ub[inew] = m_ub[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
        delete var.var_rep;
    }
    m_vars.resize(inew);
    m_lb.resize(inew, QPOASES_INFTY);
    m_ub.resize(inew, -QPOASES_INFTY);
  }
  {
    int inew = 0;
    for (unsigned iold = 0; iold < m_cnts.size(); ++iold)
    {
      const Cnt& cnt = m_cnts[iold];
      if (!cnt.cnt_rep->removed)
      {
        m_cnts[inew] = cnt;
        m_cntExprs[inew] = m_cntExprs[iold];
        m_cntTypes[inew] = m_cntTypes[iold];
        cnt.cnt_rep->index = inew;
        ++inew;
      }
      else
        delete cnt.cnt_rep;
    }
    m_cnts.resize(inew);
    m_cntExprs.resize(inew);
    m_cntTypes.resize(inew);
  }
}

void qpOASESModel::setVarBounds(const vector<Var>& vars, const vector<double>& lower, const vector<double>& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    m_lb[varind] = lower[i];
    m_ub[varind] = upper[i];
  }
}
vector<double> qpOASESModel::getVarValues(const VarVector& vars) const
{
  vector<double> out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    out[i] = m_soln[varind];
  }
  return out;
}


CvxOptStatus qpOASESModel::optimize()
{
  update();
  update_objective();
  update_constraints();
  updateSolver();
  qpOASES::returnValue val = qpOASES::RET_QP_SOLUTION_STARTED;

  // Solve Problem
  int nWSR = 255;
  if(m_problem->isInitialised())
  {
    val = m_problem->hotstart(&m_H, m_g.data(),
                              &m_A, m_lb.data(), m_ub.data(),
                              m_lbA.data(), m_ubA.data(),
                              nWSR, nullptr);
  }
  
  if(val != qpOASES::SUCCESSFUL_RETURN)
  {
    createSolver();
  
    val = m_problem->init(&m_H, m_g.data(),
                        &m_A, m_lb.data(), m_ub.data(),
                        m_lbA.data(), m_ubA.data(),
                        nWSR, nullptr);
  }
  
  if(val == qpOASES::SUCCESSFUL_RETURN)
  {
    // opt += m_objective.affexpr.constant;
    m_soln.resize(m_vars.size(), 0.);
    val = m_problem->getPrimalSolution(m_soln.data());
    return CVX_SOLVED;
  }
  else if (val == qpOASES::RET_INIT_FAILED_INFEASIBILITY)
  {
      return CVX_INFEASIBLE;
  }
  else
  {
    return CVX_FAILED;
  }
}
void qpOASESModel::setObjective(const AffExpr& expr) { m_objective.affexpr = expr; }
void qpOASESModel::setObjective(const QuadExpr& expr) { m_objective = expr; }
void qpOASESModel::writeToFile(const string& /*fname*/)
{
  // assert(0 && "NOT IMPLEMENTED");
}
VarVector qpOASESModel::getVars() const { return m_vars; }
}
