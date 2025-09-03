#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <cmath>
#include <Eigen/SparseCore>
#include <fstream>
#include <csignal>
#include <console_bridge/console.h>
#include <osqp.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/sco_common.hpp>
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
  settings.polishing = 1;
  settings.adaptive_rho = 1;
  settings.verbose = 0;
  settings.check_dualgap = 0;
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
  n_ = static_cast<OSQPInt>(n);

  Eigen::SparseMatrix<double> sm;
  exprToEigen(objective_, sm, q_, static_cast<int>(n), true);

  // Copy triangular upper into empty matrix
  Eigen::SparseMatrix<double> triangular_sm;
  triangular_sm = sm.triangularView<Eigen::Upper>();

  std::vector<OSQPInt> prev_row_indices;
  std::vector<OSQPInt> prev_column_pointers;
  // Check if dimensions have changed
  bool sparsity_equal = false;
  if (check_sparsity && P_ != nullptr && P_->n == sm.outerSize() && P_->m == sm.innerSize() &&
      P_->nzmax == triangular_sm.nonZeros())
  {
    sparsity_equal = true;
    // Store previous values for the sparsity check
    prev_row_indices.swap(P_row_indices_);
    prev_column_pointers.swap(P_column_pointers_);
  }

  eigenToCSC(triangular_sm, P_row_indices_, P_column_pointers_, P_csc_data_);

  // Check if sparsity has changed
  sparsity_equal = sparsity_equal &&
                   memcmp(prev_column_pointers.data(), P_column_pointers_.data(), static_cast<size_t>(P_->n) + 1) == 0;
  sparsity_equal =
      sparsity_equal && (memcmp(prev_row_indices.data(), P_row_indices_.data(), static_cast<size_t>(P_->nzmax)) == 0);

  P_.reset(OSQPCscMatrix_new(n_,
                             n_,
                             static_cast<OSQPInt>(P_csc_data_.size()),
                             P_csc_data_.data(),
                             P_row_indices_.data(),
                             P_column_pointers_.data()));

  return sparsity_equal;
}

bool OSQPModel::updateConstraints(bool check_sparsity)
{
  const std::size_t n = vars_.size();
  const std::size_t m = cnts_.size();
  const auto n_int = static_cast<Eigen::Index>(n);
  const auto m_int = static_cast<Eigen::Index>(m);

  m_ = static_cast<OSQPInt>(m) + static_cast<OSQPInt>(n);

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

  std::vector<OSQPInt> prev_row_indices;
  std::vector<OSQPInt> prev_column_pointers;
  // Check if dimensions have changed
  bool sparsity_equal = false;
  if (check_sparsity && A_ != nullptr && A_->n == sm.outerSize() && A_->m == sm.innerSize() &&
      A_->nzmax == sm.nonZeros())
  {
    sparsity_equal = true;
    // Store previous values for the sparsity check
    prev_row_indices.swap(A_row_indices_);
    prev_column_pointers.swap(A_column_pointers_);
  }

  eigenToCSC(sm, A_row_indices_, A_column_pointers_, A_csc_data_);

  // Check if sparsity has changed
  sparsity_equal = sparsity_equal &&
                   memcmp(prev_column_pointers.data(), A_column_pointers_.data(), static_cast<size_t>(A_->n) + 1) == 0;
  sparsity_equal =
      sparsity_equal && (memcmp(prev_row_indices.data(), A_row_indices_.data(), static_cast<size_t>(A_->nzmax)) == 0);

  A_.reset(OSQPCscMatrix_new(m_,
                             n_,
                             static_cast<OSQPInt>(A_csc_data_.size()),
                             A_csc_data_.data(),
                             A_row_indices_.data(),
                             A_column_pointers_.data()));

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
        // When updating, warm start is implicit (depending on the warm_starting setting)
        allow_update = true;
      }
      else if (config_.settings.warm_starting != 0)
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
  if (allow_update && (osqp_update_data_vec(osqp_workspace_, q_.data(), l_.data(), u_.data()) != 0))
  {
    allow_update = false;
    LOG_WARN("OSQP updating bounds and linear costs failed.");
  }
  if (allow_update &&
      (osqp_update_data_mat(osqp_workspace_, P_->x, OSQP_NULL, P_->nzmax, A_->x, OSQP_NULL, A_->nzmax) != 0))
  {
    allow_update = false;
    LOG_WARN("OSQP updating P and A matrices failed.");
  }

  // If setup is not required then return
  if (allow_update)
  {
    LOG_DEBUG("OSQP updated (warm start = %lli).", config_.settings.warm_starting);
    return;
  }

  auto settings = config_.settings;
  DblVec prev_x;
  DblVec prev_y;
  if (osqp_workspace_ != nullptr)
  {
    if (allow_explicit_warm_start)
    {
      LOG_DEBUG("OSQP explicit warm start (warm_starting = %lli).", config_.settings.warm_starting);
      // Store previous solution
      prev_x = DblVec(osqp_workspace_->solution->x, osqp_workspace_->solution->x + n_);
      prev_y = DblVec(osqp_workspace_->solution->y, osqp_workspace_->solution->y + m_);
      settings.rho = osqp_workspace_->settings->rho;
    }
    osqp_cleanup(osqp_workspace_);
  }

  // Setup workspace - this should be called only once
  auto ret = osqp_setup(&osqp_workspace_, P_.get(), q_.data(), A_.get(), l_.data(), u_.data(), m_, n_, &settings);
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
    std::cout << "OSQP Number of Variables:" << n_ << '\n';
    std::cout << "OSQP Number of Constraints:" << m_ << '\n';

    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> P_p_vec(P_->p, P_->n + 1);
    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> P_i_vec(P_->i, P_->nzmax);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> P_x_vec(P_->x, P_->nzmax);
    std::cout << "OSQP Hessian:" << '\n';
    std::cout << "     nzmax:" << P_->nzmax << '\n';
    std::cout << "        nz:" << P_->nz << '\n';
    std::cout << "         m:" << P_->m << '\n';
    std::cout << "         n:" << P_->n << '\n';
    std::cout << "         p:" << P_p_vec.transpose().format(format) << '\n';
    std::cout << "         i:" << P_i_vec.transpose().format(format) << '\n';
    std::cout << "         x:" << P_x_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::VectorXd> q_vec(q_.data(), n_);
    std::cout << "OSQP Gradient: " << q_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> A_p_vec(A_->p, A_->n + 1);
    Eigen::Map<Eigen::Matrix<OSQPInt, Eigen::Dynamic, 1>> A_i_vec(A_->i, A_->nzmax);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> A_x_vec(A_->x, A_->nzmax);
    std::cout << "OSQP Constraint Matrix:" << '\n';
    std::cout << "     nzmax:" << A_->nzmax << '\n';
    std::cout << "         m:" << A_->m << '\n';
    std::cout << "         n:" << A_->n << '\n';
    std::cout << "         p:" << A_p_vec.transpose().format(format) << '\n';
    std::cout << "         i:" << A_i_vec.transpose().format(format) << '\n';
    std::cout << "         x:" << A_x_vec.transpose().format(format) << '\n';

    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> l_vec(l_.data(), m_);
    Eigen::Map<Eigen::Matrix<OSQPFloat, Eigen::Dynamic, 1>> u_vec(u_.data(), m_);
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
  const OSQPInt retcode = osqp_solve(osqp_workspace_);

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

    if (trajopt_common::GetLogLevel() >= trajopt_common::LevelDebug)
    {
      switch (status)
      {
        case OSQP_SOLVED:
          break;
        case OSQP_SOLVED_INACCURATE:
          LOG_WARN("OSQP solved inaccurate");
          break;
        case OSQP_PRIMAL_INFEASIBLE:
          LOG_WARN("OSQP primal infeasible");
          break;
        case OSQP_PRIMAL_INFEASIBLE_INACCURATE:
          LOG_WARN("OSQP primal infeasible inaccurate");
          break;
        case OSQP_DUAL_INFEASIBLE:
          LOG_WARN("OSQP dual infeasible");
          break;
        case OSQP_DUAL_INFEASIBLE_INACCURATE:
          LOG_WARN("OSQP dual infeasible inaccurate");
          break;
        case OSQP_MAX_ITER_REACHED:
          LOG_WARN("OSQP max iterations reached");
          break;
        case OSQP_TIME_LIMIT_REACHED:
          LOG_WARN("OSQP time limit reached");
          break;
        case OSQP_NON_CVX:
          LOG_WARN("OSQP non-convex problem");
          break;
        case OSQP_SIGINT:
          LOG_WARN("OSQP interrupted by signal");
          break;
        case OSQP_UNSOLVED:
          LOG_WARN("OSQP unsolved");
          break;
        default:
          LOG_ERROR("OSQP unknown status: %i", status);
      }
    }

    if (status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE)
      return CVX_SOLVED;
    if (status == OSQP_PRIMAL_INFEASIBLE || status == OSQP_PRIMAL_INFEASIBLE_INACCURATE ||
        status == OSQP_DUAL_INFEASIBLE || status == OSQP_DUAL_INFEASIBLE_INACCURATE)
      return CVX_INFEASIBLE;
  }

  // Log error
  switch (retcode)
  {
    case OSQP_NO_ERROR:
      break;
    case OSQP_DATA_VALIDATION_ERROR:
      LOG_ERROR("OSQP Data Validation Error");
      break;
    case OSQP_SETTINGS_VALIDATION_ERROR:
      LOG_ERROR("OSQP Settings Validation Error");
      break;
    case OSQP_LINSYS_SOLVER_INIT_ERROR:
      LOG_ERROR("OSQP Linear System Solver Initialization Error");
      break;
    case OSQP_NONCVX_ERROR:
      LOG_ERROR("OSQP Non Convex Error");
      break;
    case OSQP_MEM_ALLOC_ERROR:
      LOG_ERROR("OSQP Memory Allocation Error");
      break;
    case OSQP_WORKSPACE_NOT_INIT_ERROR:
      LOG_ERROR("OSQP Workspace Not Initialized Error");
      break;
    case OSQP_ALGEBRA_LOAD_ERROR:
      LOG_ERROR("OSQP Algebra Load Error");
      break;
    case OSQP_FOPEN_ERROR:
      LOG_ERROR("OSQP File Open Error");
      break;
    case OSQP_CODEGEN_DEFINES_ERROR:
      LOG_ERROR("OSQP Codegen Defines Error");
      break;
    case OSQP_DATA_NOT_INITIALIZED:
      LOG_ERROR("OSQP Data Not Initialized Error");
      break;
    case OSQP_FUNC_NOT_IMPLEMENTED:
      LOG_ERROR("OSQP Function Not Implemented Error");
      break;
    default:
      LOG_ERROR("OSQP Unknown Error: %lld", retcode);
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
