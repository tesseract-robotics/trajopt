#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <constants.h>
#include <cmath>
#include <Eigen/SparseCore>
#include <fstream>
#include <csignal>
#include <iomanip>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>

namespace sco
{
const double OSQP_INFINITY = std::numeric_limits<double>::infinity();
const bool OSQP_COMPARE_DEBUG_MODE = false;

OSQPModelConfig::OSQPModelConfig()
{
  // Define Solver settings as default
  // see https://osqp.org/docs/interfaces/solver_settings.html#solver-settings
  osqp_set_default_settings(&settings);
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-6;
  settings.max_iter = 8192;
  settings.polish = 1;
  settings.adaptive_rho = 1;
  settings.verbose = 0;
}

Model::Ptr createOSQPModel(const ModelConfig::ConstPtr& config = nullptr)
{
  return std::make_shared<OSQPModel>(config);
}

OSQPModel::OSQPModel(const ModelConfig::ConstPtr& config) : P_(nullptr), A_(nullptr)
{
  // tuning parameters to be less accurate, but add a polishing step
  if (config != nullptr)
  {
    config_.settings = std::dynamic_pointer_cast<const OSQPModelConfig>(config)->settings;
  }
}

OSQPModel::~OSQPModel()
{
  // The osqp_workspace_ is managed by osqp but its members are not so must clean up.
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);

  // Clean up memory
  for (Var& var : vars_)
    var.var_rep->removed = true;
  for (Cnt& cnt : cnts_)
    cnt.cnt_rep->removed = true;

  OSQPModel::update();
}

Var OSQPModel::addVar(const std::string& name)
{
  std::scoped_lock lock(mutex_);
  vars_.push_back(std::make_shared<VarRep>(vars_.size(), name, this));
  lbs_.push_back(-OSQP_INFINITY);
  ubs_.push_back(OSQP_INFINITY);
  return vars_.back();
}

Cnt OSQPModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  std::scoped_lock lock(mutex_);
  cnts_.push_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  std::scoped_lock lock(mutex_);
  cnts_.push_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/) { throw std::runtime_error("NOT IMPLEMENTED"); }

void OSQPModel::removeVars(const VarVector& vars)
{
  std::scoped_lock lock(mutex_);
  SizeTVec inds;
  vars2inds(vars, inds);
  for (const auto& var : vars)
    var.var_rep->removed = true;
}

void OSQPModel::removeCnts(const CntVector& cnts)
{
  std::scoped_lock lock(mutex_);
  SizeTVec inds;
  cnts2inds(cnts, inds);
  for (const auto& cnt : cnts)
    cnt.cnt_rep->removed = true;
}

void OSQPModel::updateObjective()
{
  const size_t n = vars_.size();
  osqp_data_.n = static_cast<c_int>(n);

  Eigen::SparseMatrix<double> sm;
  exprToEigen(objective_, sm, q_, static_cast<int>(n), true);

  // Copy triangular upper into empty matrix
  Eigen::SparseMatrix<double> triangular_sm;
  triangular_sm = sm.triangularView<Eigen::Upper>();

  eigenToCSC(triangular_sm, P_row_indices_, P_column_pointers_, P_csc_data_);

  P_.reset(csc_matrix(osqp_data_.n,
                      osqp_data_.n,
                      static_cast<c_int>(P_csc_data_.size()),
                      P_csc_data_.data(),
                      P_row_indices_.data(),
                      P_column_pointers_.data()));

  osqp_data_.P = P_.get();
  osqp_data_.q = q_.data();
}

void OSQPModel::updateConstraints()
{
  const size_t n = vars_.size();
  const size_t m = cnts_.size();
  const auto n_int = static_cast<Eigen::Index>(n);
  const auto m_int = static_cast<Eigen::Index>(m);

  osqp_data_.m = static_cast<c_int>(m) + static_cast<c_int>(n);

  Eigen::SparseMatrix<double> sm;
  Eigen::VectorXd v;
  exprToEigen(cnt_exprs_, sm, v, n_int);
  sm.conservativeResize(m_int + n_int,
                        Eigen::NoChange_t(n_int));  // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult)

  l_.clear();
  l_.resize(m + n, -OSQP_INFINITY);
  u_.clear();
  u_.resize(m + n, OSQP_INFINITY);

  for (std::size_t i_cnt = 0; i_cnt < m; ++i_cnt)
  {
    l_[i_cnt] = (cnt_types_[i_cnt] == INEQ) ? -OSQP_INFINITY : v[static_cast<Eigen::Index>(i_cnt)];
    u_[i_cnt] = v[static_cast<Eigen::Index>(i_cnt)];
  }

  sm.reserve(sm.nonZeros() + static_cast<Eigen::Index>(n));
  for (std::size_t i_bnd = 0; i_bnd < n; ++i_bnd)
  {
    l_[i_bnd + m] = fmax(lbs_[i_bnd], -OSQP_INFINITY);
    u_[i_bnd + m] = fmin(ubs_[i_bnd], OSQP_INFINITY);
    sm.insert(static_cast<Eigen::Index>(i_bnd + m), static_cast<Eigen::Index>(i_bnd)) = 1.;
  }

  eigenToCSC(sm, A_row_indices_, A_column_pointers_, A_csc_data_);

  A_.reset(csc_matrix(osqp_data_.m,
                      osqp_data_.n,
                      static_cast<c_int>(A_csc_data_.size()),
                      A_csc_data_.data(),
                      A_row_indices_.data(),
                      A_column_pointers_.data()));

  osqp_data_.A = A_.get();

  osqp_data_.l = l_.data();
  osqp_data_.u = u_.data();
}

void OSQPModel::createOrUpdateSolver()
{
  updateObjective();
  updateConstraints();  // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult)

  // TODO atm we are not updating the workspace, but recreating it each time.
  // In the future, we will checking sparsity did not change and update instead
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);

  // Setup workspace - this should be called only once
  auto ret = osqp_setup(&osqp_workspace_, &osqp_data_, &config_.settings);
  if (ret != 0)
  {
    // In this case, no data got allocated, so don't try to free it.
    if (ret == OSQP_DATA_VALIDATION_ERROR || ret == OSQP_SETTINGS_VALIDATION_ERROR)
      osqp_workspace_ = nullptr;
    throw std::runtime_error("Could not initialize OSQP: error " + std::to_string(ret));
  }
}

void OSQPModel::update()
{
  {
    std::size_t inew = 0;
    for (std::size_t iold = 0; iold < vars_.size(); ++iold)
    {
      Var& var = vars_[iold];
      if (!var.var_rep->removed)
      {
        vars_[inew] = var;
        lbs_[inew] = lbs_[iold];
        ubs_[inew] = ubs_[iold];
        var.var_rep->index = inew;
        ++inew;
      }
      else
      {
        var.var_rep = nullptr;
      }
    }
    vars_.resize(inew);
    lbs_.resize(inew);
    ubs_.resize(inew);
  }
  {
    std::size_t inew = 0;
    for (std::size_t iold = 0; iold < cnts_.size(); ++iold)
    {
      Cnt& cnt = cnts_[iold];
      if (!cnt.cnt_rep->removed)
      {
        cnts_[inew] = cnt;
        cnt_exprs_[inew] = cnt_exprs_[iold];
        cnt_types_[inew] = cnt_types_[iold];
        cnt.cnt_rep->index = inew;
        ++inew;
      }
      else
      {
        cnt.cnt_rep = nullptr;
      }
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
    const std::size_t varind = vars[i].var_rep->index;
    lbs_[varind] = lower[i];
    ubs_[varind] = upper[i];
  }
}
DblVec OSQPModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const std::size_t varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus OSQPModel::optimize()
{
  update();
  try
  {
    createOrUpdateSolver();  // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult)
  }
  catch (std::exception& e)
  {
    std::cout << "OSQP Setup failed with error: " << e.what() << std::endl;
    return CVX_FAILED;
  }

  if (OSQP_COMPARE_DEBUG_MODE)
  {
    Eigen::IOFormat format(5);
    std::cout << "OSQP Number of Variables:" << osqp_data_.n << std::endl;
    std::cout << "OSQP Number of Constraints:" << osqp_data_.m << std::endl;

    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> P_p_vec(osqp_data_.P->p, osqp_data_.P->n + 1);
    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> P_i_vec(osqp_data_.P->i, osqp_data_.P->nzmax);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> P_x_vec(osqp_data_.P->x, osqp_data_.P->nzmax);
    std::cout << "OSQP Hessian:" << std::endl;
    std::cout << "     nzmax:" << osqp_data_.P->nzmax << std::endl;
    std::cout << "        nz:" << osqp_data_.P->nz << std::endl;
    std::cout << "         m:" << osqp_data_.P->m << std::endl;
    std::cout << "         n:" << osqp_data_.P->n << std::endl;
    std::cout << "         p:" << P_p_vec.transpose().format(format) << std::endl;
    std::cout << "         i:" << P_i_vec.transpose().format(format) << std::endl;
    std::cout << "         x:" << P_x_vec.transpose().format(format) << std::endl;

    Eigen::Map<Eigen::VectorXd> q_vec(osqp_data_.q, osqp_data_.n);
    std::cout << "OSQP Gradient: " << q_vec.transpose().format(format) << std::endl;

    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> A_p_vec(osqp_data_.A->p, osqp_data_.A->n + 1);
    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> A_i_vec(osqp_data_.A->i, osqp_data_.A->nzmax);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> A_x_vec(osqp_data_.A->x, osqp_data_.A->nzmax);
    std::cout << "OSQP Constraint Matrix:" << std::endl;
    std::cout << "     nzmax:" << osqp_data_.A->nzmax << std::endl;
    std::cout << "         m:" << osqp_data_.A->m << std::endl;
    std::cout << "         n:" << osqp_data_.A->n << std::endl;
    std::cout << "         p:" << A_p_vec.transpose().format(format) << std::endl;
    std::cout << "         i:" << A_i_vec.transpose().format(format) << std::endl;
    std::cout << "         x:" << A_x_vec.transpose().format(format) << std::endl;

    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l_vec(osqp_data_.l, osqp_data_.m);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u_vec(osqp_data_.u, osqp_data_.m);
    std::cout << "OSQP Lower Bounds: " << l_vec.transpose().format(format) << std::endl;
    std::cout << "OSQP Upper Bounds: " << u_vec.transpose().format(format) << std::endl;

    std::vector<std::string> vars_names(vars_.size());
    for (const auto& var : vars_)
      vars_names[var.var_rep->index] = var.var_rep->name;

    std::cout << "OSQP Variable Names: ";
    for (const auto& var_name : vars_names)
      std::cout << var_name << ",";
    std::cout << std::endl;
  }

  // Solve Problem
  const c_int retcode = osqp_solve(osqp_workspace_);

  if (retcode == 0)
  {
    // opt += m_objective.affexpr.constant;
    solution_ = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + vars_.size());

    if (OSQP_COMPARE_DEBUG_MODE)
    {
      Eigen::IOFormat format(5);
      Eigen::Map<Eigen::VectorXd> solution_vec(solution_.data(), static_cast<Eigen::Index>(solution_.size()));
      std::cout << "OSQP Solution: " << solution_vec.transpose().format(format) << std::endl;
    }

    auto status = static_cast<int>(osqp_workspace_->info->status_val);
    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE)
      return CVX_SOLVED;
    if (status == OSQP_PRIMAL_INFEASIBLE || status == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
        status == OSQP_DUAL_INFEASIBLE || status == OSQP_DUAL_INFEASIBLE_INACCURATE)
      return CVX_INFEASIBLE;
  }
  return CVX_FAILED;
}
void OSQPModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void OSQPModel::setObjective(const QuadExpr& expr) { objective_ = expr; }

VarVector OSQPModel::getVars() const { return vars_; }

void OSQPModel::writeToFile(const std::string& fname) const
{
  std::ofstream outStream(fname);
  outStream << "\\ Generated by trajopt_sco with backend OSQP\n";
  outStream << "Minimize\n";
  outStream << objective_;
  outStream << "Subject To\n";
  for (std::size_t i = 0; i < cnt_exprs_.size(); ++i)
  {
    std::string op = (cnt_types_[i] == INEQ) ? " <= " : " = ";
    outStream << cnt_exprs_[i] << op << 0 << "\n";
  }

  outStream << "Bounds\n";
  for (std::size_t i = 0; i < vars_.size(); ++i)
  {
    outStream << lbs_[i] << " <= " << vars_[i] << " <= " << ubs_[i] << "\n";
  }
  outStream << "End";
}
}  // namespace sco
