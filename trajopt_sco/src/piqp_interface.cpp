#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <Eigen/SparseCore>
#include <fstream>
#include <piqp/piqp.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_sco/piqp_interface.hpp>
#include <trajopt_sco/solver_utils.hpp>
#include <trajopt_common/logging.hpp>

namespace sco
{
PIQPModelConfig::PIQPModelConfig() { setDefaultPIQPSettings(settings); }

void PIQPModelConfig::setDefaultPIQPSettings(piqp::Settings<double>& settings)
{
  settings = piqp::Settings<double>();
  settings.kkt_solver = piqp::KKTSolver::sparse_ldlt;
  settings.eps_abs = 1e-4;
  settings.eps_rel = 1e-6;
  settings.check_duality_gap = true;
  settings.max_iter = 250;
  settings.verbose = false;
  settings.compute_timings = false;
}

Model::Ptr createPIQPModel(const ModelConfig::ConstPtr& config) { return std::make_shared<PIQPModel>(config); }

PIQPModel::PIQPModel(const ModelConfig::ConstPtr& config)
{
  if (config != nullptr)
  {
    const auto piqp_config = std::dynamic_pointer_cast<const PIQPModelConfig>(config);
    if (piqp_config == nullptr)
      throw std::runtime_error("PIQPModel requires a PIQPModelConfig");
    config_.settings = piqp_config->settings;
  }
}

PIQPModel::~PIQPModel()
{
  // Clean up memory
  for (const Var& var : vars_)
    var.var_rep->removed = true;
  for (const Cnt& cnt : cnts_)
    cnt.cnt_rep->removed = true;

  PIQPModel::update();
}

Var PIQPModel::addVar(const std::string& name)
{
  const std::scoped_lock lock(mutex_);
  vars_.emplace_back(std::make_shared<VarRep>(vars_.size(), name, this));
  lbs_.push_back(-PIQP_INF);
  ubs_.push_back(PIQP_INF);
  return vars_.back();
}

Cnt PIQPModel::addEqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  const std::scoped_lock lock(mutex_);
  cnts_.emplace_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(EQ);
  return cnts_.back();
}

Cnt PIQPModel::addIneqCnt(const AffExpr& expr, const std::string& /*name*/)
{
  const std::scoped_lock lock(mutex_);
  cnts_.emplace_back(std::make_shared<CntRep>(cnts_.size(), this));
  cnt_exprs_.push_back(expr);
  cnt_types_.push_back(INEQ);
  return cnts_.back();
}

Cnt PIQPModel::addIneqCnt(const QuadExpr&, const std::string& /*name*/) { throw std::runtime_error("NOT IMPLEMENTED"); }

void PIQPModel::removeVars(const VarVector& vars)
{
  const std::scoped_lock lock(mutex_);
  for (const auto& var : vars)
    var.var_rep->removed = true;
}

void PIQPModel::removeCnts(const CntVector& cnts)
{
  const std::scoped_lock lock(mutex_);
  for (const auto& cnt : cnts)
    cnt.cnt_rep->removed = true;
}

void PIQPModel::update()
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

void PIQPModel::setVarBounds(const VarVector& vars, const DblVec& lower, const DblVec& upper)
{
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const std::size_t varind = vars[i].var_rep->index;
    lbs_[varind] = lower[i];
    ubs_[varind] = upper[i];
  }
}

DblVec PIQPModel::getVarValues(const VarVector& vars) const
{
  DblVec out(vars.size());
  for (unsigned i = 0; i < vars.size(); ++i)
  {
    const std::size_t varind = vars[i].var_rep->index;
    out[i] = solution_[varind];
  }
  return out;
}

CvxOptStatus PIQPModel::optimize()
{
  update();
  const auto n = static_cast<Eigen::Index>(vars_.size());

  // exprToEigen doubles the Hessian, matching PIQP's 1/2*x'Px; PIQP reads only the upper triangle
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  exprToEigen(objective_, hessian, gradient, n, true);
  const Eigen::SparseMatrix<double> P = hessian.triangularView<Eigen::Upper>();

  // Every constraint row reads expr <= 0 or expr = 0, stored as row * x <= v or row * x = v
  Eigen::SparseMatrix<double> cnt_matrix;
  Eigen::VectorXd cnt_values;
  exprToEigen(cnt_exprs_, cnt_matrix, cnt_values, n);

  std::vector<Eigen::Index> split_row(cnt_types_.size());
  Eigen::Index n_eq = 0;
  Eigen::Index n_ineq = 0;
  for (std::size_t i = 0; i < cnt_types_.size(); ++i)
    split_row[i] = (cnt_types_[i] == EQ) ? n_eq++ : n_ineq++;

  std::vector<Eigen::Triplet<double>> eq_triplets;
  std::vector<Eigen::Triplet<double>> ineq_triplets;
  eq_triplets.reserve(static_cast<std::size_t>(cnt_matrix.nonZeros() + n));
  ineq_triplets.reserve(static_cast<std::size_t>(cnt_matrix.nonZeros()));
  for (Eigen::Index col = 0; col < cnt_matrix.outerSize(); ++col)
  {
    for (Eigen::SparseMatrix<double>::InnerIterator it(cnt_matrix, col); it; ++it)
    {
      const auto row = static_cast<std::size_t>(it.row());
      (cnt_types_[row] == EQ ? eq_triplets : ineq_triplets).emplace_back(split_row[row], col, it.value());
    }
  }

  // PIQP meets variable bounds only to its tolerance; pin a variable with equal bounds by an equality row instead
  Eigen::VectorXd x_l = Eigen::Map<const Eigen::VectorXd>(lbs_.data(), n).cwiseMax(-PIQP_INF);
  Eigen::VectorXd x_u = Eigen::Map<const Eigen::VectorXd>(ubs_.data(), n).cwiseMin(PIQP_INF);
  std::vector<double> pinned_values;
  for (Eigen::Index i = 0; i < n; ++i)
  {
    if (x_l[i] != x_u[i])
      continue;
    eq_triplets.emplace_back(n_eq + static_cast<Eigen::Index>(pinned_values.size()), i, 1.0);
    pinned_values.push_back(x_l[i]);
    x_l[i] = -PIQP_INF;
    x_u[i] = PIQP_INF;
  }

  Eigen::SparseMatrix<double> A(n_eq + static_cast<Eigen::Index>(pinned_values.size()), n);
  A.setFromTriplets(eq_triplets.begin(), eq_triplets.end());
  Eigen::SparseMatrix<double> G(n_ineq, n);
  G.setFromTriplets(ineq_triplets.begin(), ineq_triplets.end());
  Eigen::VectorXd b(A.rows());
  Eigen::VectorXd h_u(n_ineq);
  for (std::size_t i = 0; i < cnt_types_.size(); ++i)
    (cnt_types_[i] == EQ ? b : h_u)[split_row[i]] = cnt_values[static_cast<Eigen::Index>(i)];
  b.tail(static_cast<Eigen::Index>(pinned_values.size())) =
      Eigen::Map<const Eigen::VectorXd>(pinned_values.data(), static_cast<Eigen::Index>(pinned_values.size()));
  const Eigen::VectorXd h_l = Eigen::VectorXd::Constant(n_ineq, -PIQP_INF);

  piqp::SparseSolver<double> solver;
  solver.settings() = config_.settings;
  solver.setup(P, gradient, A, b, G, h_l, h_u, x_l, x_u);
  const piqp::Status status = solver.solve();

  // PIQP reports rejected settings, such as a KKT solver the sparse backend lacks, only on stderr
  if (status == piqp::Status::PIQP_UNSOLVED || status == piqp::Status::PIQP_INVALID_SETTINGS)
  {
    LOG_ERROR("PIQP setup failed with status %s (kkt_solver %s)",
              piqp::status_to_string(status),
              piqp::kkt_solver_to_string(config_.settings.kkt_solver));
    return CVX_FAILED;
  }

  solution_ = DblVec(solver.result().x.data(), solver.result().x.data() + n);

  if (status == piqp::Status::PIQP_SOLVED)
    return CVX_SOLVED;

  LOG_DEBUG("PIQP status: %s", piqp::status_to_string(status));
  if (status == piqp::Status::PIQP_PRIMAL_INFEASIBLE || status == piqp::Status::PIQP_DUAL_INFEASIBLE)
    return CVX_INFEASIBLE;
  return CVX_FAILED;
}

void PIQPModel::setObjective(const AffExpr& expr) { objective_.affexpr = expr; }
void PIQPModel::setObjective(const QuadExpr& expr) { objective_ = expr; }

VarVector PIQPModel::getVars() const { return vars_; }

void PIQPModel::writeToFile(const std::string& fname) const
{
  std::ofstream outStream(fname);
  outStream << "\\ Generated by trajopt_sco with backend PIQP\n";
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
