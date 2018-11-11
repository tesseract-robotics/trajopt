#include <cmath>
#include <Eigen/Eigen>
#include <fstream>
#include <constants.h>
#include <signal.h>
#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace std;

namespace sco
{
double OSQP_INFINITY = numeric_limits<double>::infinity();

extern void simplify2(vector<int>& inds, vector<double>& vals);
extern vector<int> vars2inds(const vector<Var>& vars);
extern vector<int> cnts2inds(const vector<Cnt>& cnts);

void triplets_to_CSC(vector<c_int>& row_indices,
                     vector<c_int>& column_pointers,
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

ModelPtr createOSQPModel()
{
  ModelPtr out(new OSQPModel());
  return out;
}

OSQPModel::OSQPModel()
{
  // Problem settings
  _settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));

  // Define Solver settings as default
  osqp_set_default_settings(_settings);
  _settings->eps_abs = 1e-4;
  _settings->eps_rel = 1e-6;
  _settings->max_iter = 8192;
  _settings->polish = 1;

  // Populate data
  _data = (OSQPData *)c_malloc(sizeof(OSQPData));
  osqp_data_.A = nullptr;
  osqp_data_.P = nullptr;
  osqp_workspace_ = nullptr;
}

OSQPModel::~OSQPModel()
{
  // Cleanup
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);
  if (osqp_data_.A != nullptr)
    c_free(osqp_data_.A);
  if (osqp_data_.P != nullptr)
    c_free(osqp_data_.P);
}

Var OSQPModel::addVar(const string& name)
{
  m_vars.push_back(new VarRep(m_vars.size(), name, this));
  m_lbs.push_back(-OSQP_INFINITY);
  m_ubs.push_back(OSQP_INFINITY);
  return m_vars.back();
}
Cnt OSQPModel::addEqCnt(const AffExpr& expr, const string& /*name*/)
{
  m_cnts.push_back(new CntRep(m_cnts.size(), this));
  m_cntExprs.push_back(expr);
  m_cntTypes.push_back(EQ);
  return m_cnts.back();
}
Cnt OSQPModel::addIneqCnt(const AffExpr& expr, const string& /*name*/)
{
  m_cnts.push_back(new CntRep(m_cnts.size(), this));
  m_cntExprs.push_back(expr);
  m_cntTypes.push_back(INEQ);
  return m_cnts.back();
}
Cnt OSQPModel::addIneqCnt(const QuadExpr&, const string& /*name*/)
{
  assert(0 && "NOT IMPLEMENTED");
  return 0;
}
void OSQPModel::removeVars(const VarVector& vars)
{
  vector<int> inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void OSQPModel::removeCnts(const vector<Cnt>& cnts)
{
  vector<int> inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void OSQPModel::update_objective()
{
  int n = m_vars.size();
  
  m_q.clear();
  m_q.resize(n, 0.0);
  for (size_t i = 0; i < m_objective.affexpr.size(); ++i)
  {
    m_q[m_objective.affexpr.vars[i].var_rep->index] += m_objective.affexpr.coeffs[i];
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

  vector<c_int> row_indices;
  vector<c_int> column_pointers;
  vector<double> csc_data;
  triplets_to_CSC(m_P_row_indices, m_P_column_pointers, m_P_csc_data,
                  m_vars.size(), m_vars.size(), n*n,
                  data_i, data_j, vals_ij);

  _data->n = n;
  if (osqp_data_.P != nullptr)
    c_free(osqp_data_.P);
  _data->P = csc_matrix(_data->n, _data->n, m_P_csc_data.size(),
                        m_P_csc_data.data(), m_P_row_indices.data(),
                        m_P_column_pointers.data());
  _data->q = m_q.data();
}

void OSQPModel::update_constraints()
{
  int n = m_vars.size();
  int m = m_cnts.size();
  _data->m = m + n;

  m_l.clear();
  m_l.resize(m + n, -OSQP_INFINITY);
  m_u.clear();
  m_u.resize(m + n, OSQP_INFINITY);

  vector<int> data_i;
  vector<int> data_j;
  vector<double> data_ij;
  for (int iVar = 0; iVar < n; ++iVar)
  {
    m_l[iVar] = fmax(m_lbs[iVar], -OSQP_INFINITY);
    m_u[iVar] = fmin(m_ubs[iVar], OSQP_INFINITY);
    data_i.push_back(iVar);
    data_j.push_back(iVar);
    data_ij.push_back(1.);
  }
  
  for (int iCnt = 0; iCnt < m; ++iCnt)
  {
    const AffExpr& aff = m_cntExprs[iCnt];
    vector<int> inds = vars2inds(aff.vars);
    vector<double> vals = aff.coeffs;

    for (unsigned i = 0; i < aff.vars.size(); ++i)
    {
      if(aff.coeffs[i] != 0.)
      {
        data_i.push_back(iCnt + n);
        data_j.push_back(inds[i]);
        data_ij.push_back(aff.coeffs[i]);
      }
    }

    m_l[iCnt + n] = (m_cntTypes[iCnt] == INEQ) ? -OSQP_INFINITY : -aff.constant;
    m_u[iCnt + n] = -aff.constant;
  }

  triplets_to_CSC(m_A_row_indices, m_A_column_pointers, m_A_csc_data,
                  m_cnts.size() + m_vars.size(), m_vars.size(), m*n,
                  data_i, data_j, data_ij);

  if (osqp_data_.A != nullptr)
    c_free(osqp_data_.A); 
  _data->A = csc_matrix(_data->m, _data->n, m_A_csc_data.size(), 
                        m_A_csc_data.data(), m_A_row_indices.data(),
                        m_A_column_pointers.data());

  _data->l = m_l.data();
  _data->u = m_u.data();

}

void OSQPModel::create_or_update_solver()
{
  if(true)
  {
    update_objective();
    update_constraints();

    // TODO atm we are not updating the workspace, but recreating it each time
    if (osqp_workspace_ != nullptr)
      osqp_cleanup(osqp_workspace_);
    // Setup workspace - this should be called only once
   _work = osqp_setup(_data, _settings);
  }
  else
  {
    // TODO update everything - take care of checking sparsity did not change
  }
}

void OSQPModel::update()
{
  {
    int inew = 0;
    for (unsigned iold = 0; iold < m_vars.size(); ++iold)
    {
      const Var& var = m_vars[iold];
      if (!var.var_rep->removed)
      {
        m_vars[inew] = var;
        m_lbs[inew] = m_lbs[iold];
        m_ubs[inew] = m_ubs[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
        delete var.var_rep;
    }
    m_vars.resize(inew);
    m_lbs.resize(inew);
    m_ubs.resize(inew);
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

void OSQPModel::setVarBounds(const vector<Var>& vars, const vector<double>& lower, const vector<double>& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    m_lbs[varind] = lower[i];
    m_ubs[varind] = upper[i];
  }
}
vector<double> OSQPModel::getVarValues(const VarVector& vars) const
{
  vector<double> out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    out[i] = m_soln[varind];
  }
  return out;
}


CvxOptStatus OSQPModel::optimize()
{
  update();
  create_or_update_solver();
  
  // Solve Problem
  int retcode = osqp_solve(_work);;

  if(retcode == 0)
  {
    // opt += m_objective.affexpr.constant;
    m_soln = vector<double>(_work->solution->x, 
                            _work->solution->x + m_vars.size());
    int status = _work->info->status_val;
    if (status == OSQP_SOLVED || 
        status == OSQP_SOLVED_INACCURATE)
      return CVX_SOLVED;
    else if (status == OSQP_PRIMAL_INFEASIBLE || 
             status == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
             status == OSQP_DUAL_INFEASIBLE ||
             status == OSQP_DUAL_INFEASIBLE_INACCURATE)
      return CVX_INFEASIBLE;
  }
  return CVX_FAILED;
}
void OSQPModel::setObjective(const AffExpr& expr) { m_objective.affexpr = expr; }
void OSQPModel::setObjective(const QuadExpr& expr) { m_objective = expr; }
void OSQPModel::writeToFile(const string& /*fname*/)
{
  // assert(0 && "NOT IMPLEMENTED");
}
VarVector OSQPModel::getVars() const { return m_vars; }
}
