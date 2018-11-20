#include <cmath>
#include <Eigen/Eigen>
#include <fstream>
#include <constants.h>
#include <signal.h>
#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

namespace sco
{
double OSQP_INFINITY = std::numeric_limits<double>::infinity();

extern void simplify2(IntVec& inds, DblVec& vals);
extern IntVec vars2inds(const VarVector& vars);
extern IntVec cnts2inds(const CntVector& cnts);

void tripletsToCSC(std::vector<c_int>& row_indices,
                   std::vector<c_int>& column_pointers,
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

  for (unsigned int k = 0; k < data_vij.size(); ++k)
  {
    if (data_vij[k] != 0.0)
    {  // TODO since we are not using simplify2, apparently there are cases
      // when we try to add the same data twice. We should troubleshoot
      // why this is the case in the first place - using coeffRef instead
      // of insert for now
      sm.coeffRef(data_i[k], data_j[k]) += data_vij[k];
    }
  }

  Eigen::SparseMatrix<double> sm_t;
  auto sm_ref = std::ref(sm);

  if (only_upper_triangular)
  {
    sm_t = sm.triangularView<Eigen::Upper>();
    sm_t.makeCompressed();
    sm_ref = std::ref(sm_t);
  }
  else
  {
    sm.makeCompressed();
  }

  auto si_p = sm_ref.get().innerIndexPtr();
  row_indices.assign(si_p, si_p + sm_ref.get().nonZeros());

  si_p = sm_ref.get().outerIndexPtr();
  column_pointers.assign(si_p, si_p + sm_ref.get().outerSize());

  // while Eigen does not enforce this, CSC format requires that column
  // pointers ends with the number of non-zero elements
  column_pointers.push_back(sm_ref.get().nonZeros());

  auto csc_v = sm_ref.get().valuePtr();
  values.assign(csc_v, csc_v + sm_ref.get().nonZeros());
}

ModelPtr createOSQPModel()
{
  ModelPtr out(new OSQPModel());
  return out;
}

OSQPModel::OSQPModel()
{
  // Define Solver settings as default
  osqp_set_default_settings(&osqp_settings_);
  osqp_settings_.eps_abs = 1e-4;
  osqp_settings_.eps_rel = 1e-6;
  osqp_settings_.max_iter = 8192;
  osqp_settings_.polish = 1;

  // Initialize data
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

Var OSQPModel::addVar(const std::string& name)
{
  vars_.push_back(new VarRep(vars_.size(), name, this));
  lbs_.push_back(-OSQP_INFINITY);
  ubs_.push_back(OSQP_INFINITY);
  return vars_.back();
}
Cnt OSQPModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}
Cnt OSQPModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}
Cnt OSQPModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/)
{
  assert(0 && "NOT IMPLEMENTED");
  return 0;
}
void OSQPModel::removeVars(const VarVector& vars)
{
  IntVec inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void OSQPModel::removeCnts(const CntVector& cnts)
{
  IntVec inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void OSQPModel::updateObjective()
{
  int n = vars_.size();
  osqp_data_.n = n;

  q_.clear();
  q_.resize(n, 0.0);
  for (size_t i = 0; i < objective_.affexpr.size(); ++i)
  {
    q_[objective_.affexpr.vars[i].var_rep->index] += objective_.affexpr.coeffs[i];
  }

  IntVec ind1 = vars2inds(objective_.vars1);
  IntVec ind2 = vars2inds(objective_.vars2);
  Eigen::SparseMatrix<double> sm(vars_.size(), vars_.size());
  sm.reserve(vars_.size() * vars_.size());

  for (size_t i = 0; i < objective_.coeffs.size(); ++i)
  {
    if (objective_.coeffs[i] != 0.0)
    {
      if (ind1[i] == ind2[i])
        sm.coeffRef(ind1[i], ind2[i]) += objective_.coeffs[i];
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
        sm.coeffRef(r, c) += objective_.coeffs[i];
      }
    }
  }

  sm = sm + Eigen::SparseMatrix<double>(sm.transpose());
  sm.makeCompressed();

  IntVec data_i;
  IntVec data_j;
  DblVec vals_ij;
  for (int k = 0; k < sm.outerSize(); ++k)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(sm, k); it; ++it)
    {
      data_i.push_back(it.row());  // row index
      data_j.push_back(it.col());  // col index
      vals_ij.push_back(it.value());
    }
  }

  std::vector<c_int> row_indices;
  std::vector<c_int> column_pointers;
  DblVec csc_data;
  tripletsToCSC(
      P_row_indices_, P_column_pointers_, P_csc_data_, vars_.size(), vars_.size(), n * n, data_i, data_j, vals_ij);

  if (osqp_data_.P != nullptr)
    c_free(osqp_data_.P);
  osqp_data_.P = csc_matrix(osqp_data_.n,
                            osqp_data_.n,
                            P_csc_data_.size(),
                            P_csc_data_.data(),
                            P_row_indices_.data(),
                            P_column_pointers_.data());

  osqp_data_.q = q_.data();
}

void OSQPModel::updateConstraints()
{
  int n = vars_.size();
  int m = cnts_.size();
  osqp_data_.m = m + n;

  l_.clear();
  l_.resize(m + n, -OSQP_INFINITY);
  u_.clear();
  u_.resize(m + n, OSQP_INFINITY);

  IntVec data_i;
  IntVec data_j;
  DblVec data_ij;
  for (int iVar = 0; iVar < n; ++iVar)
  {
    l_[iVar] = fmax(lbs_[iVar], -OSQP_INFINITY);
    u_[iVar] = fmin(ubs_[iVar], OSQP_INFINITY);
    data_i.push_back(iVar);
    data_j.push_back(iVar);
    data_ij.push_back(1.);
  }

  for (int iCnt = 0; iCnt < m; ++iCnt)
  {
    const AffExpr& aff = cnt_exprs_[iCnt];
    IntVec inds = vars2inds(aff.vars);
    DblVec vals = aff.coeffs;

    for (unsigned i = 0; i < aff.vars.size(); ++i)
    {
      if (aff.coeffs[i] != 0.)
      {
        data_i.push_back(iCnt + n);
        data_j.push_back(inds[i]);
        data_ij.push_back(aff.coeffs[i]);
      }
    }

    l_[iCnt + n] = (cnt_types_[iCnt] == INEQ) ? -OSQP_INFINITY : -aff.constant;
    u_[iCnt + n] = -aff.constant;
  }

  tripletsToCSC(A_row_indices_,
                A_column_pointers_,
                A_csc_data_,
                cnts_.size() + vars_.size(),
                vars_.size(),
                m * n,
                data_i,
                data_j,
                data_ij);

  if (osqp_data_.A != nullptr)
    c_free(osqp_data_.A);
  osqp_data_.A = csc_matrix(osqp_data_.m,
                            osqp_data_.n,
                            A_csc_data_.size(),
                            A_csc_data_.data(),
                            A_row_indices_.data(),
                            A_column_pointers_.data());

  osqp_data_.l = l_.data();
  osqp_data_.u = u_.data();
}

void OSQPModel::createOrUpdateSolver()
{
  if (true)
  {
    updateObjective();
    updateConstraints();

    // TODO atm we are not updating the workspace, but recreating it each time
    if (osqp_workspace_ != nullptr)
      osqp_cleanup(osqp_workspace_);
    // Setup workspace - this should be called only once
    osqp_workspace_ = osqp_setup(&osqp_data_, &osqp_settings_);
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
    for (unsigned iold = 0; iold < vars_.size(); ++iold)
    {
      const Var& var = vars_[iold];
      if (!var.var_rep->removed)
      {
        vars_[inew] = var;
        lbs_[inew] = lbs_[iold];
        ubs_[inew] = ubs_[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
        delete var.var_rep;
    }
    vars_.resize(inew);
    lbs_.resize(inew);
    ubs_.resize(inew);
  }
  {
    int inew = 0;
    for (unsigned iold = 0; iold < cnts_.size(); ++iold)
    {
      const Cnt& cnt = cnts_[iold];
      if (!cnt.cnt_rep->removed)
      {
        cnts_[inew] = cnt;
        cnt_exprs_[inew] = cnt_exprs_[iold];
        cnt_types_[inew] = cnt_types_[iold];
        cnt.cnt_rep->index = inew;
        ++inew;
      }
      else
        delete cnt.cnt_rep;
    }
    cnts_.resize(inew);
    cnt_exprs_.resize(inew);
    cnt_types_.resize(inew);
  }
}

void OSQPModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    lbs_[varind] = lower[i];
    ubs_[varind] = upper[i];
  }
}
DblVec OSQPModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus OSQPModel::optimize()
{
  update();
  createOrUpdateSolver();

  // Solve Problem
  int retcode = osqp_solve(osqp_workspace_);
  ;

  if (retcode == 0)
  {
    // opt += m_objective.affexpr.constant;
    solution_ = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + vars_.size());
    int status = osqp_workspace_->info->status_val;
    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE)
      return CVX_SOLVED;
    else if (status == OSQP_PRIMAL_INFEASIBLE || status == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
             status == OSQP_DUAL_INFEASIBLE || status == OSQP_DUAL_INFEASIBLE_INACCURATE)
      return CVX_INFEASIBLE;
  }
  return CVX_FAILED;
}
void OSQPModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void OSQPModel::setObjective(const QuadExpr& expr) { objective_ = expr; }
void OSQPModel::writeToFile(const std::string& /*fname*/)
{
  // assert(0 && "NOT IMPLEMENTED");
}
VarVector OSQPModel::getVars() const { return vars_; }
}
