#include <cmath>
#include <Eigen/Eigen>
#include <fstream>
#include <signal.h>
#include <trajopt_sco/qpoases_interface.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

using namespace qpOASES;

namespace sco
{
double QPOASES_INFTY = qpOASES::INFTY;

extern void simplify2(IntVec& inds, DblVec& vals);
extern IntVec vars2inds(const VarVector& vars);
extern IntVec cnts2inds(const CntVector& cnts);

void tripletsToCSC(IntVec& row_indices,
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

  for (unsigned int k = 0; k < std::min(m_size, n_size); ++k)
  {
    sm.coeffRef(k, k) += 0.0;
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

ModelPtr createqpOASESModel()
{
  ModelPtr out(new qpOASESModel());
  return out;
}

qpOASESModel::qpOASESModel()
{
  qpoases_options_.setToMPC();
  qpoases_options_.printLevel = qpOASES::PL_NONE;
  qpoases_options_.enableRegularisation = qpOASES::BT_TRUE;
  qpoases_options_.ensureConsistency();
}

qpOASESModel::~qpOASESModel() {}

Var qpOASESModel::addVar(const std::string& name)
{
  vars_.push_back(new VarRep(vars_.size(), name, this));
  lb_.push_back(-QPOASES_INFTY);
  ub_.push_back(QPOASES_INFTY);
  return vars_.back();
}
Cnt qpOASESModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}
Cnt qpOASESModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  cnts_.push_back(new CntRep(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}
Cnt qpOASESModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/)
{
  assert(0 && "NOT IMPLEMENTED");
  return 0;
}
void qpOASESModel::removeVars(const VarVector& vars)
{
  IntVec inds = vars2inds(vars);
  for (unsigned i = 0; i < vars.size(); ++i)
    vars[i].var_rep->removed = true;
}

void qpOASESModel::removeCnts(const CntVector& cnts)
{
  IntVec inds = cnts2inds(cnts);
  for (unsigned i = 0; i < cnts.size(); ++i)
    cnts[i].cnt_rep->removed = true;
}

void qpOASESModel::updateObjective()
{
  int n = vars_.size();

  g_.clear();
  g_.resize(n, 0.0);
  for (size_t i = 0; i < objective_.affexpr.size(); ++i)
  {
    g_[objective_.affexpr.vars[i].var_rep->index] += objective_.affexpr.coeffs[i];
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

  IntVec row_indices;
  IntVec column_pointers;
  DblVec csc_data;
  tripletsToCSC(
      H_row_indices_, H_column_pointers_, H_csc_data_, vars_.size(), vars_.size(), n * n, data_i, data_j, vals_ij);

  H_ = SymSparseMat(vars_.size(), vars_.size(), H_row_indices_.data(), H_column_pointers_.data(), H_csc_data_.data());
  H_.createDiagInfo();
}

void qpOASESModel::updateConstraints()
{
  int n = vars_.size();
  int m = cnts_.size();

  lbA_.clear();
  lbA_.resize(m, -QPOASES_INFTY);
  ubA_.clear();
  ubA_.resize(m, QPOASES_INFTY);

  IntVec data_i;
  IntVec data_j;
  DblVec data_ij;
  for (int iCnt = 0; iCnt < m; ++iCnt)
  {
    const AffExpr& aff = cnt_exprs_[iCnt];
    IntVec inds = vars2inds(aff.vars);
    DblVec vals = aff.coeffs;

    for (unsigned i = 0; i < aff.vars.size(); ++i)
    {
      if (aff.coeffs[i] != 0)
      {
        data_i.push_back(iCnt);
        data_j.push_back(inds[i]);
        data_ij.push_back(aff.coeffs[i]);
      }
    }

    lbA_[iCnt] = (cnt_types_[iCnt] == INEQ) ? -QPOASES_INFTY : -aff.constant;
    ubA_[iCnt] = -aff.constant;
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

  A_ = SparseMatrix(cnts_.size(), vars_.size(), A_row_indices_.data(), A_column_pointers_.data(), A_csc_data_.data());
}

bool qpOASESModel::updateSolver()
{
  bool solver_updated = false;
  if (!qpoases_problem_ || vars_.size() != qpoases_problem_->getNV() || cnts_.size() != qpoases_problem_->getNC())
  {
    // Create Problem - this should be called only once
    qpoases_problem_.reset(new SQProblem(vars_.size(), cnts_.size()));
    qpoases_problem_->setOptions(qpoases_options_);
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
    for (unsigned iold = 0; iold < vars_.size(); ++iold)
    {
      const Var& var = vars_[iold];
      if (!var.var_rep->removed)
      {
        vars_[inew] = var;
        lb_[inew] = lb_[iold];
        ub_[inew] = ub_[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
        delete var.var_rep;
    }
    vars_.resize(inew);
    lb_.resize(inew, QPOASES_INFTY);
    ub_.resize(inew, -QPOASES_INFTY);
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

void qpOASESModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    lb_[varind] = lower[i];
    ub_[varind] = upper[i];
  }
}
DblVec qpOASESModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    int varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus qpOASESModel::optimize()
{
  update();
  updateObjective();
  updateConstraints();
  updateSolver();
  qpOASES::returnValue val = qpOASES::RET_QP_SOLUTION_STARTED;

  // Solve Problem
  int nWSR = 255;
  if (qpoases_problem_->isInitialised())
  {
    val = qpoases_problem_->hotstart(
        &H_, g_.data(), &A_, lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), nWSR, nullptr);
  }

  if (val != qpOASES::SUCCESSFUL_RETURN)
  {
    // TODO ATM this means we are creating a new solver even if updateSolver
    //      returned true and the problem is not initialized. Still, it makes
    //      tests pass.
    createSolver();

    val = qpoases_problem_->init(&H_, g_.data(), &A_, lb_.data(), ub_.data(), lbA_.data(), ubA_.data(), nWSR, nullptr);
  }

  if (val == qpOASES::SUCCESSFUL_RETURN)
  {
    // opt += m_objective.affexpr.constant;
    solution_.resize(vars_.size(), 0.);
    val = qpoases_problem_->getPrimalSolution(solution_.data());
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
void qpOASESModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void qpOASESModel::setObjective(const QuadExpr& expr) { objective_ = expr; }
void qpOASESModel::writeToFile(const std::string& /*fname*/)
{
  // assert(0 && "NOT IMPLEMENTED");
}
VarVector qpOASESModel::getVars() const { return vars_; }
}
