#include <osqp.h>
#include <trajopt_common/macros.h>
#include "trajopt_sco/sco_common.hpp"
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <constants.h>
#include <cmath>
#include <Eigen/SparseCore>
#include <fstream>
#include <csignal>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/osqp_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>

namespace sco
{
const double OSQP_INFINITY = OSQP_INFTY;
const bool OSQP_COMPARE_DEBUG_MODE = false;

OSQPModelConfig::OSQPModelConfig() { setDefaultOSQPSettings(settings); }

void OSQPModelConfig::setDefaultOSQPSettings(OSQPSettings& settings)
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

OSQPModel::OSQPModel(const ModelConfig::ConstPtr& config) : P_(nullptr, free), A_(nullptr, free)
{
  // tuning parameters to be less accurate, but add a polishing step
  if (config != nullptr)
  {
    const auto& osqp_config = std::dynamic_pointer_cast<const OSQPModelConfig>(config);
    config_.settings = osqp_config->settings;
    config_.update_workspace = osqp_config->update_workspace;
  }
}

OSQPModel::~OSQPModel()
{
  // The osqp_workspace_ is managed by osqp but its members are not so must clean up.
  if (osqp_workspace_ != nullptr)
    osqp_cleanup(osqp_workspace_);

  // Clean up memory
  for (const Var& var : vars_)
    var.var_rep->removed = true;
  for (const Cnt& cnt : cnts_)
    cnt.cnt_rep->removed = true;

  OSQPModel::update();
}

Var OSQPModel::addVar(const std::string& name)
{
  const std::scoped_lock lock(mutex_);
  vars_.emplace_back(std::make_shared<VarRep>(vars_.size(), name, this));
  lbs_.push_back(-OSQP_INFINITY);
  ubs_.push_back(OSQP_INFINITY);
  return vars_.back();
}

Cnt OSQPModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  const std::scoped_lock lock(mutex_);
  cnts_.emplace_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  const std::scoped_lock lock(mutex_);
  cnts_.emplace_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}

Cnt OSQPModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/) { throw std::runtime_error("NOT IMPLEMENTED"); }

void OSQPModel::removeVars(const VarVector& vars)
{
  const std::scoped_lock lock(mutex_);
  SizeTVec inds;
  vars2inds(vars, inds);
  for (const auto& var : vars)
    var.var_rep->removed = true;
}

void OSQPModel::removeCnts(const CntVector& cnts)
{
  const std::scoped_lock lock(mutex_);
  SizeTVec inds;
  cnts2inds(cnts, inds);
  for (const auto& cnt : cnts)
    cnt.cnt_rep->removed = true;
}

bool OSQPModel::updateObjective(bool check_sparsity)
{
  const std::size_t n = vars_.size();
  osqp_data_.n = static_cast<c_int>(n);

  Eigen::SparseMatrix<double> sm;
  exprToEigen(objective_, sm, q_, static_cast<int>(n), true);

  // Copy triangular upper into empty matrix
  Eigen::SparseMatrix<double> triangular_sm;
  triangular_sm = sm.triangularView<Eigen::Upper>();

  eigenToCSC(triangular_sm, P_row_indices_, P_column_pointers_, P_csc_data_);

  // Check if sparsity has changed
  bool sparsity_equal = false;
  if (check_sparsity && osqp_data_.P != nullptr && osqp_data_.P->n == osqp_data_.n && osqp_data_.P->m == osqp_data_.n &&
      osqp_data_.P->nzmax == P_csc_data_.size())
  {
    sparsity_equal = memcmp(osqp_data_.P->p, P_column_pointers_.data(), static_cast<size_t>(osqp_data_.P->n) + 1) == 0;
    sparsity_equal = sparsity_equal &&
                     (memcmp(osqp_data_.P->i, P_row_indices_.data(), static_cast<size_t>(osqp_data_.P->nzmax)) == 0);
  }

  P_.reset(csc_matrix(osqp_data_.n,
                      osqp_data_.n,
                      static_cast<c_int>(P_csc_data_.size()),
                      P_csc_data_.data(),
                      P_row_indices_.data(),
                      P_column_pointers_.data()));

  osqp_data_.P = P_.get();
  osqp_data_.q = q_.data();

  return sparsity_equal;
}

bool OSQPModel::updateConstraints(bool check_sparsity)
{
  const std::size_t n = vars_.size();
  const std::size_t m = cnts_.size();
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

  std::vector<Eigen::Index> new_inner_sizes(n);
  for (std::size_t k = 0; k < n; ++k)
  {
    new_inner_sizes[k] = sm.innerVector(static_cast<Eigen::Index>(k)).nonZeros() + 1;
  }
  sm.reserve(new_inner_sizes);
  for (std::size_t i_bnd = 0; i_bnd < n; ++i_bnd)
  {
    l_[i_bnd + m] = fmax(lbs_[i_bnd], -OSQP_INFINITY);
    u_[i_bnd + m] = fmin(ubs_[i_bnd], OSQP_INFINITY);
    sm.insert(static_cast<Eigen::Index>(i_bnd + m), static_cast<Eigen::Index>(i_bnd)) = 1.;
  }

  eigenToCSC(sm, A_row_indices_, A_column_pointers_, A_csc_data_);

  // Check if sparsity has changed
  bool sparsity_equal = false;
  if (check_sparsity && osqp_data_.A != nullptr && osqp_data_.A->n == osqp_data_.n && osqp_data_.A->m == osqp_data_.m &&
      osqp_data_.A->nzmax == A_csc_data_.size())
  {
    sparsity_equal = memcmp(osqp_data_.A->p, A_column_pointers_.data(), static_cast<size_t>(osqp_data_.A->n) + 1) == 0;
    sparsity_equal = sparsity_equal &&
                     (memcmp(osqp_data_.A->i, A_row_indices_.data(), static_cast<size_t>(osqp_data_.A->nzmax)) == 0);
  }

  A_.reset(csc_matrix(osqp_data_.m,
                      osqp_data_.n,
                      static_cast<c_int>(A_csc_data_.size()),
                      A_csc_data_.data(),
                      A_row_indices_.data(),
                      A_column_pointers_.data()));

  osqp_data_.A = A_.get();

  osqp_data_.l = l_.data();
  osqp_data_.u = u_.data();

  return sparsity_equal;
}

void OSQPModel::createOrUpdateSolver()
{
  bool allow_update = false;
  bool allow_explicit_warm_start = false;
  if (osqp_workspace_ != nullptr)
  {
    const auto status_val = static_cast<int>(osqp_workspace_->info->status_val);
    // Only warm start or update if the last optimzation was (almost) solved
    if ((status_val == OSQP_SOLVED) || (status_val == OSQP_SOLVED_INACCURATE))
    {
      if (config_.update_workspace)
      {
        // When updating, warm start is implicit (depending on the warm_start setting)
        allow_update = true;
      }
      else if (config_.settings.warm_start != 0)
      {
        // Only allow explicit warm start if not updating
        allow_explicit_warm_start = true;
      }
    }
  }

  // Update P and q (only check sparsity if necessary)
  const bool P_sparsity_equal = updateObjective(allow_update || allow_explicit_warm_start);
  // Update A and l, u
  const bool A_sparsity_equal = updateConstraints(P_sparsity_equal);

  // Only allow update or warm start if the sparsity of P and A did not change
  allow_update = allow_update && P_sparsity_equal && A_sparsity_equal;
  allow_explicit_warm_start = allow_explicit_warm_start && P_sparsity_equal && A_sparsity_equal;

  // If sparsity did not change, update data, otherwise cleanup and setup
  bool need_setup = true;
  if (allow_update)
  {
    LOG_DEBUG("OSQP update (warm start = %lli).", config_.settings.warm_start);
    need_setup = false;

    if (osqp_update_bounds(osqp_workspace_, osqp_data_.l, osqp_data_.u) != 0)
    {
      need_setup = true;
      LOG_WARN("OSQP updating bounds failed.");
    }
    if (!need_setup && (osqp_update_lin_cost(osqp_workspace_, osqp_data_.q) != 0))
    {
      need_setup = true;
      LOG_WARN("OSQP updating linear costs failed.");
    }
    if (!need_setup && (osqp_update_P_A(osqp_workspace_,
                                        osqp_data_.P->x,
                                        OSQP_NULL,
                                        osqp_data_.P->nzmax,
                                        osqp_data_.A->x,
                                        OSQP_NULL,
                                        osqp_data_.A->nzmax) != 0))
    {
      need_setup = true;
      LOG_WARN("OSQP updating P and A matrices failed.");
    }
  }

  // If setup is not required then return
  if (!need_setup)
    return;

  DblVec prev_x;
  DblVec prev_y;
  double prev_rho = 0.0;
  if (osqp_workspace_ != nullptr)
  {
    if (allow_explicit_warm_start)
    {
      LOG_DEBUG("OSQP explicit warm start (warm_start = %lli).", config_.settings.warm_start);
      // Store previous solution
      prev_x = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + osqp_data_.n);
      prev_y = DblVec(osqp_workspace_->solution->y, osqp_workspace_->solution->y + osqp_data_.m);
      prev_rho = osqp_workspace_->settings->rho;
    }
    osqp_cleanup(osqp_workspace_);
  }

  // Setup workspace - this should be called only once
  auto ret = osqp_setup(&osqp_workspace_, &osqp_data_, &config_.settings);
  if (ret != 0)
  {
    // In this case, no data got allocated, so don't try to free it.
    if (ret == OSQP_DATA_VALIDATION_ERROR || ret == OSQP_SETTINGS_VALIDATION_ERROR)
      osqp_workspace_ = nullptr;
    throw std::runtime_error("Could not initialize OSQP: error " + std::to_string(ret));
  }
  if (!prev_x.empty() && !prev_y.empty())
  {
    // Warm start recreated workspace with previous solution
    if (osqp_warm_start(osqp_workspace_, prev_x.data(), prev_y.data()) != 0)
    {
      LOG_WARN("OSQP warm start failed.");
    }
    if (osqp_update_rho(osqp_workspace_, prev_rho) != 0)
    {
      LOG_WARN("OSQP rho update failed.");
    }
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
    createOrUpdateSolver();  // NOLINT(clang-analyzer-core.UndefinedBinaryOperatorResult,clang-analyzer-core.uninitialized.Assign)
  }
  catch (std::exception& e)
  {
    std::cout << "OSQP Setup failed with error: " << e.what() << '\n';
    return CVX_FAILED;
  }

  if (OSQP_COMPARE_DEBUG_MODE)
  {
    const Eigen::IOFormat format(5);
    std::cout << "OSQP Number of Variables:" << osqp_data_.n << '\n';
    std::cout << "OSQP Number of Constraints:" << osqp_data_.m << '\n';

    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> P_p_vec(osqp_data_.P->p, osqp_data_.P->n + 1);
    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> P_i_vec(osqp_data_.P->i, osqp_data_.P->nzmax);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> P_x_vec(osqp_data_.P->x, osqp_data_.P->nzmax);
    std::cout << "OSQP Hessian:" << '\n';
    std::cout << "     nzmax:" << osqp_data_.P->nzmax << '\n';
    std::cout << "        nz:" << osqp_data_.P->nz << '\n';
    std::cout << "         m:" << osqp_data_.P->m << '\n';
    std::cout << "         n:" << osqp_data_.P->n << '\n';
    std::cout << "         p:" << P_p_vec.transpose().format(format) << '\n';
    std::cout << "         i:" << P_i_vec.transpose().format(format) << '\n';
    std::cout << "         x:" << P_x_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::VectorXd> q_vec(osqp_data_.q, osqp_data_.n);
    std::cout << "OSQP Gradient: " << q_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> A_p_vec(osqp_data_.A->p, osqp_data_.A->n + 1);
    Eigen::Map<Eigen::Matrix<c_int, Eigen::Dynamic, 1>> A_i_vec(osqp_data_.A->i, osqp_data_.A->nzmax);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> A_x_vec(osqp_data_.A->x, osqp_data_.A->nzmax);
    std::cout << "OSQP Constraint Matrix:" << '\n';
    std::cout << "     nzmax:" << osqp_data_.A->nzmax << '\n';
    std::cout << "         m:" << osqp_data_.A->m << '\n';
    std::cout << "         n:" << osqp_data_.A->n << '\n';
    std::cout << "         p:" << A_p_vec.transpose().format(format) << '\n';
    std::cout << "         i:" << A_i_vec.transpose().format(format) << '\n';
    std::cout << "         x:" << A_x_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> l_vec(osqp_data_.l, osqp_data_.m);
    Eigen::Map<Eigen::Matrix<c_float, Eigen::Dynamic, 1>> u_vec(osqp_data_.u, osqp_data_.m);
    std::cout << "OSQP Lower Bounds: " << l_vec.transpose().format(format) << '\n';
    std::cout << "OSQP Upper Bounds: " << u_vec.transpose().format(format) << '\n';

    std::vector<std::string> vars_names(vars_.size());
    for (const auto& var : vars_)
      vars_names[var.var_rep->index] = var.var_rep->name;

    std::cout << "OSQP Variable Names: ";
    for (const auto& var_name : vars_names)
      std::cout << var_name << ",";
    std::cout << '\n';
  }

  // Solve Problem
  const c_int retcode = osqp_solve(osqp_workspace_);

  if (retcode == 0)
  {
    // opt += m_objective.affexpr.constant;
    solution_ = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + vars_.size());
    auto status = static_cast<int>(osqp_workspace_->info->status_val);

    if (OSQP_COMPARE_DEBUG_MODE)
    {
      const Eigen::IOFormat format(5);
      Eigen::Map<Eigen::VectorXd> solution_vec(solution_.data(), static_cast<Eigen::Index>(solution_.size()));
      std::cout << "OSQP Status Value: " << status << '\n';
      std::cout << "OSQP Solution: " << solution_vec.transpose().format(format) << '\n';
    }

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
    const std::string op = (cnt_types_[i] == INEQ) ? " <= " : " = ";
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
