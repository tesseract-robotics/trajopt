#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <tesseract_core/basic_kin.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/kinematic_terms.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt/trajectory_costs.hpp>
#include <trajopt_sco/expr_op_overloads.hpp>
#include <trajopt_sco/expr_ops.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/eigen_slicing.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/vector_ops.hpp>

namespace
{
bool gRegisteredMakers = false;

void ensure_only_members(const Json::Value& v, const char** fields, int nvalid)
{
  for (Json::ValueConstIterator it = v.begin(); it != v.end(); ++it)
  {
    bool valid = false;
    for (int j = 0; j < nvalid; ++j)
    {
      JSONCPP_STRING member_name = it.name();
      if (strcmp(member_name.c_str(), fields[j]) == 0)
      {
        valid = true;
        break;
      }
    }
    if (!valid)
    {
      PRINT_AND_THROW(boost::format("invalid field found: %s") % it.name());
    }
  }
}

void RegisterMakers()
{
  trajopt::TermInfo::RegisterMaker("dynamic_cart_pose", &trajopt::DynamicCartPoseTermInfo::create);
  trajopt::TermInfo::RegisterMaker("cart_pose", &trajopt::CartPoseTermInfo::create);
  trajopt::TermInfo::RegisterMaker("cart_vel", &trajopt::CartVelTermInfo::create);
  trajopt::TermInfo::RegisterMaker("joint_pos", &trajopt::JointPosTermInfo::create);
  trajopt::TermInfo::RegisterMaker("joint_vel", &trajopt::JointVelTermInfo::create);
  trajopt::TermInfo::RegisterMaker("joint_acc", &trajopt::JointAccTermInfo::create);
  trajopt::TermInfo::RegisterMaker("joint_jerk", &trajopt::JointJerkTermInfo::create);
  trajopt::TermInfo::RegisterMaker("collision", &trajopt::CollisionTermInfo::create);
  trajopt::TermInfo::RegisterMaker("total_time", &trajopt::TotalTimeTermInfo::create);

  gRegisteredMakers = true;
}

/**
 * @brief Checks the size of the parameter given and throws if incorrect
 * @param parameter The vector whose size is getting checked
 * @param expected_size The expected size of the vector
 * @param name The name to use when printing an error or warning
 * @param apply_first If true and only one value is given, broadcast value to length of expected_size
 */
void checkParameterSize(trajopt::DblVec& parameter,
                        const unsigned int& expected_size,
                        const std::string& name,
                        const bool& apply_first = true)
{
  if (apply_first == true && parameter.size() == 1)
  {
    parameter = trajopt::DblVec(expected_size, parameter[0]);
    ROS_INFO("1 %s given. Applying to all %i joints", name.c_str(), expected_size);
  }
  else if (parameter.size() != expected_size)
  {
    PRINT_AND_THROW(boost::format("wrong number of %s. expected %i got %i") % name % expected_size % parameter.size());
  }
}

#if 0
BoolVec toMask(const VectorXd& x) {
  BoolVec out(x.size());
  for (int i=0; i < x.size(); ++i) out[i] = (x[i] > 0);
  return out;
}
#endif
}  // namespace

namespace trajopt
{
std::map<std::string, TermInfo::MakerFunc> TermInfo::name2maker;
void TermInfo::RegisterMaker(const std::string& type, MakerFunc f) { name2maker[type] = f; }
TermInfoPtr TermInfo::fromName(const std::string& type)
{
  if (!gRegisteredMakers)
    RegisterMakers();
  if (name2maker.find(type) != name2maker.end())
  {
    return (*name2maker[type])();
  }
  else
  {
    // RAVELOG_ERROR("There is no cost of type %s\n", type.c_str());
    return TermInfoPtr();
  }
}

void ProblemConstructionInfo::readBasicInfo(const Json::Value& v)
{
  json_marshal::childFromJson(v, basic_info.start_fixed, "start_fixed", true);
  json_marshal::childFromJson(v, basic_info.n_steps, "n_steps");
  json_marshal::childFromJson(v, basic_info.manip, "manip");
  json_marshal::childFromJson(v, basic_info.robot, "robot", std::string(""));
  json_marshal::childFromJson(v, basic_info.dofs_fixed, "dofs_fixed", IntVec());
  json_marshal::childFromJson(v, basic_info.convex_solver, "convex_solver", basic_info.convex_solver);
  json_marshal::childFromJson(v, basic_info.dt_lower_lim, "dt_lower_lim", 1.0);
  json_marshal::childFromJson(v, basic_info.dt_upper_lim, "dt_upper_lim", 1.0);

  if (basic_info.dt_lower_lim <= 0 || basic_info.dt_upper_lim < basic_info.dt_lower_lim)
  {
    PRINT_AND_THROW("dt limits (Basic Info) invalid. The lower limit must be positive, "
                    "and the minimum upper limit is equal to the lower limit.");
  }
}

void ProblemConstructionInfo::readOptInfo(const Json::Value& v)
{
  json_marshal::childFromJson(
      v, opt_info.improve_ratio_threshold, "improve_ratio_threshold", opt_info.improve_ratio_threshold);
  json_marshal::childFromJson(v, opt_info.min_trust_box_size, "min_trust_box_size", opt_info.min_trust_box_size);
  json_marshal::childFromJson(v, opt_info.min_approx_improve, "min_approx_improve", opt_info.min_approx_improve);
  json_marshal::childFromJson(
      v, opt_info.min_approx_improve_frac, "min_approx_improve_frac", opt_info.min_approx_improve_frac);
  json_marshal::childFromJson(v, opt_info.max_iter, "max_iter", opt_info.max_iter);
  json_marshal::childFromJson(v, opt_info.trust_shrink_ratio, "trust_shrink_ratio", opt_info.trust_shrink_ratio);
  json_marshal::childFromJson(v, opt_info.trust_expand_ratio, "trust_expand_ratio", opt_info.trust_expand_ratio);
  json_marshal::childFromJson(v, opt_info.cnt_tolerance, "cnt_tolerance", opt_info.cnt_tolerance);
  json_marshal::childFromJson(
      v, opt_info.max_merit_coeff_increases, "max_merit_coeff_increases", opt_info.max_merit_coeff_increases);
  json_marshal::childFromJson(
      v, opt_info.merit_coeff_increase_ratio, "merit_coeff_increase_ratio", opt_info.merit_coeff_increase_ratio);
  json_marshal::childFromJson(v, opt_info.max_time, "max_time", opt_info.max_time);
  json_marshal::childFromJson(v, opt_info.merit_error_coeff, "merit_error_coeff", opt_info.merit_error_coeff);
  json_marshal::childFromJson(v, opt_info.trust_box_size, "trust_box_size", opt_info.trust_box_size);
}

void ProblemConstructionInfo::readCosts(const Json::Value& v)
{
  cost_infos.clear();
  cost_infos.reserve(v.size());
  for (Json::Value::const_iterator it = v.begin(); it != v.end(); ++it)
  {
    std::string type, use_time_str;
    json_marshal::childFromJson(*it, type, "type");
    json_marshal::childFromJson(*it, use_time_str, "use_time", static_cast<std::string>("false"));
    LOG_DEBUG("reading term: %s", type.c_str());
    TermInfoPtr term = TermInfo::fromName(type);

    if (!term)
      PRINT_AND_THROW(boost::format("failed to construct cost named %s") % type);

    if (boost::iequals(use_time_str, "true"))
    {
      term->term_type = TT_COST | TT_USE_TIME;
      basic_info.use_time = true;
    }
    else
      term->term_type = TT_COST;
    term->fromJson(*this, *it);
    json_marshal::childFromJson(*it, term->name, "name", type);

    cost_infos.push_back(term);
  }
}

void ProblemConstructionInfo::readConstraints(const Json::Value& v)
{
  cnt_infos.clear();
  cnt_infos.reserve(v.size());
  for (Json::Value::const_iterator it = v.begin(); it != v.end(); ++it)
  {
    std::string type, use_time_str;
    json_marshal::childFromJson(*it, type, "type");
    json_marshal::childFromJson(*it, use_time_str, "use_time", static_cast<std::string>("false"));
    LOG_DEBUG("reading term: %s", type.c_str());
    TermInfoPtr term = TermInfo::fromName(type);

    if (!term)
      PRINT_AND_THROW(boost::format("failed to construct constraint named %s") % type);

    if (boost::iequals(use_time_str, "true"))
    {
      term->term_type = (TT_CNT | TT_USE_TIME);
      basic_info.use_time = true;
    }
    else
      term->term_type = (TT_CNT);
    term->fromJson(*this, *it);
    json_marshal::childFromJson(*it, term->name, "name", type);

    cnt_infos.push_back(term);
  }
}

void ProblemConstructionInfo::readInitInfo(const Json::Value& v)
{
  std::string type_str;
  json_marshal::childFromJson(v, type_str, "type");
  json_marshal::childFromJson(v, init_info.dt, "dt", 1.0);
  int n_steps = basic_info.n_steps;
  int n_dof = static_cast<int>(kin->numJoints());

  if (boost::iequals(type_str, "stationary"))
  {
    init_info.type = InitInfo::STATIONARY;
  }
  else if (boost::iequals(type_str, "given_traj"))
  {
    init_info.type = InitInfo::GIVEN_TRAJ;
    FAIL_IF_FALSE(v.isMember("data"));
    const Json::Value& vdata = v["data"];
    if (static_cast<int>(vdata.size()) != n_steps)
    {
      PRINT_AND_THROW("given initialization traj has wrong length");
    }
    init_info.data.resize(n_steps, n_dof);
    for (int i = 0; i < n_steps; ++i)
    {
      DblVec row;
      json_marshal::fromJsonArray(vdata[i], row, static_cast<int>(n_dof));
      init_info.data.row(i) = util::toVectorXd(row);
    }
  }
  else if (boost::iequals(type_str, "joint_interpolated"))
  {
    init_info.type = InitInfo::JOINT_INTERPOLATED;
    FAIL_IF_FALSE(v.isMember("endpoint"));
    DblVec endpoint;
    json_marshal::childFromJson(v, endpoint, "endpoint");
    if (endpoint.size() != static_cast<unsigned>(n_dof))
    {
      PRINT_AND_THROW(boost::format("wrong number of dof values in "
                                    "initialization. expected %i got %i") %
                      n_dof % endpoint.size());
    }
    init_info.data = util::toVectorXd(endpoint);
  }
  else
  {
    PRINT_AND_THROW("init_info did not have a valid type from Json. Valid types are "
                    "stationary, joint_interpolated, or given_traj");
  }
}

void ProblemConstructionInfo::fromJson(const Json::Value& v)
{
  if (v.isMember("basic_info"))
  {
    readBasicInfo(v["basic_info"]);
  }
  else
  {
    PRINT_AND_THROW("Json missing required section basic_info!");
  }

  if (v.isMember("opt_info"))
  {
    readOptInfo(v["opt_info"]);
  }

  if (!env->hasManipulator(basic_info.manip))
  {
    PRINT_AND_THROW(boost::format("Manipulator does not exist: %s") % basic_info.manip.c_str());
  }
  kin = env->getManipulator(basic_info.manip);

  if (v.isMember("costs"))
    readCosts(v["costs"]);
  if (v.isMember("constraints"))
    readConstraints(v["constraints"]);

  if (v.isMember("init_info"))
  {
    readInitInfo(v["init_info"]);
  }
  else
  {
    PRINT_AND_THROW("Json missing required section init_info!");
  }
}

void generateInitTraj(TrajArray& init_traj, const ProblemConstructionInfo& pci)
{
  // TODO: Change this so that it can intelligently handle when time is enabled and dt values are/are not given
  InitInfo init_info = pci.init_info;

  // initialize based on type specified
  if (init_info.type == InitInfo::STATIONARY)
  {
    Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
    init_traj = start_pos.transpose().replicate(pci.basic_info.n_steps, 1);
  }
  else if (init_info.type == InitInfo::JOINT_INTERPOLATED)
  {
    Eigen::VectorXd start_pos = pci.env->getCurrentJointValues(pci.kin->getName());
    Eigen::VectorXd end_pos = init_info.data;
    init_traj.resize(pci.basic_info.n_steps, end_pos.rows());
    for (int idof = 0; idof < start_pos.rows(); ++idof)
    {
      init_traj.col(idof) = Eigen::VectorXd::LinSpaced(pci.basic_info.n_steps, start_pos(idof), end_pos(idof));
    }
  }
  else if (init_info.type == InitInfo::GIVEN_TRAJ)
  {
    init_traj = init_info.data;
  }
  else
  {
    PRINT_AND_THROW("Init Info did not have a valid type. Valid types are "
                    "STATIONARY, JOINT_INTERPOLATED, or GIVEN_TRAJ");
  }

  // Currently all trajectories are generated without time then appended here
  if (pci.basic_info.use_time)
  {
    // add on time (default to 1 sec)
    init_traj.conservativeResize(Eigen::NoChange_t(), init_traj.cols() + 1);

    init_traj.block(0, init_traj.cols() - 1, init_traj.rows(), 1) =
        Eigen::VectorXd::Constant(init_traj.rows(), init_info.dt);
  }
}

TrajOptResult::TrajOptResult(sco::OptResults& opt, TrajOptProb& prob)
  : cost_vals(opt.cost_vals), cnt_viols(opt.cnt_viols)
{
  for (const sco::CostPtr& cost : prob.getCosts())
  {
    cost_names.push_back(cost->name());
  }

  for (const sco::ConstraintPtr& cnt : prob.getConstraints())
  {
    cnt_names.push_back(cnt->name());
  }

  traj = getTraj(opt.x, prob.GetVars());
}

TrajOptResultPtr OptimizeProblem(TrajOptProbPtr prob, const tesseract::BasicPlottingPtr plotter)
{
  sco::BasicTrustRegionSQP opt(prob);
  sco::BasicTrustRegionSQPParameters& param = opt.getParameters();
  param.max_iter = 40;
  param.min_approx_improve_frac = .001;
  param.improve_ratio_threshold = .2;
  param.merit_error_coeff = 20;
  if (plotter)
    opt.addCallback(PlotCallback(*prob, plotter));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();
  return TrajOptResultPtr(new TrajOptResult(opt.results(), *prob));
}

TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo& pci)
{
  const BasicInfo& bi = pci.basic_info;
  int n_steps = bi.n_steps;

  bool use_time = false;
  // Check that all costs and constraints support the types that are specified in pci
  for (TermInfoPtr cost : pci.cost_infos)
  {
    if (cost->term_type & TT_CNT)
      ROS_WARN("%s is listed as a type TT_CNT but was added to cost_infos", (cost->name).c_str());
    if (!(cost->getSupportedTypes() & TT_COST))
      PRINT_AND_THROW(boost::format("%s is only a constraint, but you listed it as a cost") % cost->name);
    if (cost->term_type & TT_USE_TIME)
    {
      use_time = true;
      if (!(cost->getSupportedTypes() & TT_USE_TIME))
        PRINT_AND_THROW(boost::format("%s does not support time, but you listed it as a using time") % cost->name);
    }
  }
  for (TermInfoPtr cnt : pci.cnt_infos)
  {
    if (cnt->term_type & TT_COST)
      ROS_WARN("%s is listed as a type TT_COST but was added to cnt_infos", (cnt->name).c_str());
    if (!(cnt->getSupportedTypes() & TT_CNT))
      PRINT_AND_THROW(boost::format("%s is only a cost, but you listed it as a constraint") % cnt->name);
    if (cnt->term_type & TT_USE_TIME)
    {
      use_time = true;
      if (!(cnt->getSupportedTypes() & TT_USE_TIME))
        PRINT_AND_THROW(boost::format("%s does not support time, but you listed it as a using time") % cnt->name);
    }
  }

  // Check that if a cost or constraint uses time, basic_info is set accordingly
  if ((use_time == true) && (pci.basic_info.use_time == false))
    PRINT_AND_THROW("A term is using time and basic_info is not set correctly. Try basic_info.use_time = true");

  // This could be removed in the future once we are sure that all costs are
  if ((use_time == false) && (pci.basic_info.use_time == true))
    PRINT_AND_THROW("No terms use time and basic_info is not set correctly. Try basic_info.use_time = false");

  TrajOptProbPtr prob(new TrajOptProb(n_steps, pci));
  unsigned n_dof = prob->GetKin()->numJoints();

  // Generate initial trajectory and check its size
  TrajArray init_traj;
  generateInitTraj(init_traj, pci);
  if (pci.basic_info.use_time == true)
  {
    prob->SetHasTime(true);
    if (init_traj.rows() != n_steps || init_traj.cols() != n_dof + 1)
    {
      PRINT_AND_THROW(boost::format("Initial trajectory is not the right size matrix\n"
                                    "Expected %i rows (time steps) x %i columns (%i dof + 1 time column)\n"
                                    "Got %i rows and %i columns") %
                      n_steps % (n_dof + 1) % n_dof % init_traj.rows() % init_traj.cols());
    }
  }
  else
  {
    prob->SetHasTime(false);
    if (init_traj.rows() != n_steps || init_traj.cols() != n_dof)
    {
      PRINT_AND_THROW(boost::format("Initial trajectory is not the right size matrix\n"
                                    "Expected %i rows (time steps) x %i columns\n"
                                    "Got %i rows and %i columns") %
                      n_steps % n_dof % init_traj.rows() % init_traj.cols());
    }
  }
  prob->SetInitTraj(init_traj);

  // If start_fixed, constrain the joint values for the first time step to be their initialized values
  if (bi.start_fixed)
  {
    if (init_traj.rows() < 1)
    {
      PRINT_AND_THROW("Initial trajectory must contain at least the start state.");
    }

    if (init_traj.cols() != (n_dof + (use_time ? 1 : 0)))
    {
      PRINT_AND_THROW("robot dof values don't match initialization. I don't "
                      "know what you want me to use for the dof values");
    }

    for (int j = 0; j < static_cast<int>(n_dof); ++j)
    {
      prob->addLinearConstraint(sco::exprSub(sco::AffExpr(prob->m_traj_vars(0, j)), init_traj(0, j)), sco::EQ);
    }
  }

  // Apply constraint to each fixed dof to its initial value for all timesteps (freeze that column)
  if (!bi.dofs_fixed.empty())
  {
    for (const int& dof_ind : bi.dofs_fixed)
    {
      for (int i = 1; i < prob->GetNumSteps(); ++i)
      {
        prob->addLinearConstraint(
            sco::exprSub(sco::AffExpr(prob->m_traj_vars(i, dof_ind)), sco::AffExpr(init_traj(0, dof_ind))), sco::EQ);
      }
    }
  }

  for (const TermInfoPtr& ci : pci.cost_infos)
  {
    ci->hatch(*prob);
  }

  for (const TermInfoPtr& ci : pci.cnt_infos)
  {
    ci->hatch(*prob);
  }
  return prob;
}

TrajOptProbPtr ConstructProblem(const Json::Value& root, tesseract::BasicEnvConstPtr env)
{
  ProblemConstructionInfo pci(env);
  pci.fromJson(root);
  return ConstructProblem(pci);
}

TrajOptProb::TrajOptProb(int n_steps, const ProblemConstructionInfo& pci)
  : OptProb(pci.basic_info.convex_solver), m_kin(pci.kin), m_env(pci.env)
{
  const Eigen::MatrixX2d& limits = m_kin->getLimits();
  int n_dof = static_cast<int>(m_kin->numJoints());
  Eigen::VectorXd lower, upper;
  lower = limits.col(0);
  upper = limits.col(1);

  DblVec vlower, vupper;
  std::vector<std::string> names;
  for (int i = 0; i < n_steps; ++i)
  {
    for (int j = 0; j < n_dof; ++j)
    {
      names.push_back((boost::format("j_%i_%i") % i % j).str());
    }
    vlower.insert(vlower.end(), lower.data(), lower.data() + lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data() + upper.size());

    if (pci.basic_info.use_time == true)
    {
      vlower.insert(vlower.end(), pci.basic_info.dt_lower_lim);
      vupper.insert(vupper.end(), pci.basic_info.dt_upper_lim);
      names.push_back((boost::format("dt_%i") % i).str());
    }
  }
  sco::VarVector trajvarvec = createVariables(names, vlower, vupper);
  m_traj_vars = VarArray(n_steps, n_dof + (pci.basic_info.use_time ? 1 : 0), trajvarvec.data());
}

TrajOptProb::TrajOptProb() {}
DynamicCartPoseTermInfo::DynamicCartPoseTermInfo() : TermInfo(TT_COST | TT_CNT)
{
  pos_coeffs = Eigen::Vector3d::Ones();
  rot_coeffs = Eigen::Vector3d::Ones();
  tcp.setIdentity();
}

void DynamicCartPoseTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  Eigen::Vector3d tcp_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector4d tcp_wxyz = Eigen::Vector4d(1, 0, 0, 0);

  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  json_marshal::childFromJson(params, target, "target");
  json_marshal::childFromJson(params, pos_coeffs, "pos_coeffs", Eigen::Vector3d(1, 1, 1));
  json_marshal::childFromJson(params, rot_coeffs, "rot_coeffs", Eigen::Vector3d(1, 1, 1));
  json_marshal::childFromJson(params, link, "link");
  json_marshal::childFromJson(params, tcp_xyz, "tcp_xyz", Eigen::Vector3d(0, 0, 0));
  json_marshal::childFromJson(params, tcp_wxyz, "tcp_wxyz", Eigen::Vector4d(1, 0, 0, 0));

  Eigen::Quaterniond q(tcp_wxyz(0), tcp_wxyz(1), tcp_wxyz(2), tcp_wxyz(3));
  tcp.linear() = q.matrix();
  tcp.translation() = tcp_xyz;

  const std::vector<std::string>& link_names = pci.kin->getLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  const char* all_fields[] = { "timestep", "target", "pos_coeffs", "rot_coeffs", "link", "tcp_xyz", "tcp_wxyz" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void DynamicCartPoseTermInfo::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  if (term_type & TT_USE_TIME)
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else
  {
    sco::VectorOfVectorPtr f(new DynamicCartPoseErrCalculator(target, prob.GetKin(), prob.GetEnv(), link, tcp));
    // Apply error calculator as either cost or constraint
    if (term_type & TT_COST)
    {
      prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(
          f, prob.GetVarRow(timestep, 0, n_dof), concat(rot_coeffs, pos_coeffs), sco::ABS, name)));
    }
    else if (term_type & TT_CNT)
    {
      prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(
          f, prob.GetVarRow(timestep, 0, n_dof), concat(rot_coeffs, pos_coeffs), sco::EQ, name)));
    }
    else
    {
      ROS_WARN("DynamicCartPoseTermInfo does not have a valid term_type defined. No cost/constraint applied");
    }
  }
}

CartPoseTermInfo::CartPoseTermInfo() : TermInfo(TT_COST | TT_CNT)
{
  pos_coeffs = Eigen::Vector3d::Ones();
  rot_coeffs = Eigen::Vector3d::Ones();
  tcp.setIdentity();
}

void CartPoseTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  Eigen::Vector3d tcp_xyz = Eigen::Vector3d::Zero();
  Eigen::Vector4d tcp_wxyz = Eigen::Vector4d(1, 0, 0, 0);

  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  json_marshal::childFromJson(params, xyz, "xyz");
  json_marshal::childFromJson(params, wxyz, "wxyz");
  json_marshal::childFromJson(params, pos_coeffs, "pos_coeffs", Eigen::Vector3d(1, 1, 1));
  json_marshal::childFromJson(params, rot_coeffs, "rot_coeffs", Eigen::Vector3d(1, 1, 1));
  json_marshal::childFromJson(params, link, "link");
  json_marshal::childFromJson(params, tcp_xyz, "tcp_xyz", Eigen::Vector3d(0, 0, 0));
  json_marshal::childFromJson(params, tcp_wxyz, "tcp_wxyz", Eigen::Vector4d(1, 0, 0, 0));

  Eigen::Quaterniond q(tcp_wxyz(0), tcp_wxyz(1), tcp_wxyz(2), tcp_wxyz(3));
  tcp.linear() = q.matrix();
  tcp.translation() = tcp_xyz;

  const std::vector<std::string>& link_names = pci.kin->getLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  const char* all_fields[] = { "timestep", "xyz", "wxyz", "pos_coeffs", "rot_coeffs", "link", "tcp_xyz", "tcp_wxyz" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CartPoseTermInfo::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  Eigen::Isometry3d input_pose;
  Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  input_pose.linear() = q.matrix();
  input_pose.translation() = xyz;

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    sco::VectorOfVectorPtr f(new CartPoseErrCalculator(input_pose, prob.GetKin(), prob.GetEnv(), link, tcp));
    prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(
        f, prob.GetVarRow(timestep, 0, n_dof), concat(rot_coeffs, pos_coeffs), sco::ABS, name)));
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    sco::VectorOfVectorPtr f(new CartPoseErrCalculator(input_pose, prob.GetKin(), prob.GetEnv(), link, tcp));
    prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(
        f, prob.GetVarRow(timestep, 0, n_dof), concat(rot_coeffs, pos_coeffs), sco::EQ, name)));
  }
  else
  {
    ROS_WARN("CartPoseTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void CartVelTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, first_step, "first_step");
  json_marshal::childFromJson(params, last_step, "last_step");
  json_marshal::childFromJson(params, max_displacement, "max_displacement");

  FAIL_IF_FALSE((first_step >= 0) && (first_step <= pci.basic_info.n_steps - 1) && (first_step < last_step));
  FAIL_IF_FALSE((last_step > 0) && (last_step <= pci.basic_info.n_steps - 1));

  json_marshal::childFromJson(params, link, "link");
  const std::vector<std::string>& link_names = pci.kin->getLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  const char* all_fields[] = { "first_step", "last_step", "max_displacement", "link" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CartVelTermInfo::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    for (int iStep = first_step; iStep < last_step; ++iStep)
    {
      prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(
          sco::VectorOfVectorPtr(new CartVelErrCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          sco::MatrixOfVectorPtr(new CartVelJacCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          concat(prob.GetVarRow(iStep, 0, n_dof), prob.GetVarRow(iStep + 1, 0, n_dof)),
          Eigen::VectorXd::Ones(0),
          sco::ABS,
          name)));
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    for (int iStep = first_step; iStep < last_step; ++iStep)
    {
      prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(
          sco::VectorOfVectorPtr(new CartVelErrCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          sco::MatrixOfVectorPtr(new CartVelJacCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          concat(prob.GetVarRow(iStep, 0, n_dof), prob.GetVarRow(iStep + 1, 0, n_dof)),
          Eigen::VectorXd::Ones(0),
          sco::INEQ,
          "CartVel")));
    }
  }
  else
  {
    ROS_WARN("CartVelTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void JointPosTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  unsigned int n_dof = pci.kin->numJoints();
  json_marshal::childFromJson(params, targets, "targets");

  // Optional Parameters
  json_marshal::childFromJson(params, coeffs, "coeffs", DblVec(n_dof, 1));
  json_marshal::childFromJson(params, upper_tols, "upper_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, lower_tols, "lower_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, first_step, "first_step", 0);
  json_marshal::childFromJson(params, last_step, "last_step", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "coeffs", "first_step", "last_step", "targets", "lower_tols", "upper_tols" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointPosTermInfo::hatch(TrajOptProb& prob)
{
  unsigned int n_dof = prob.GetKin()->numJoints();

  // If optional parameter not given, set to default
  if (coeffs.empty())
    coeffs = DblVec(n_dof, 1);
  if (upper_tols.empty())
    upper_tols = DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = DblVec(n_dof, 0);
  if (last_step <= -1)
    last_step = prob.GetNumSteps() - 1;

  // Check time step is valid
  if ((prob.GetNumSteps() - 1) <= first_step)
    first_step = prob.GetNumSteps() - 1;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  //  if (last_step == first_step)
  //    last_step += 1;
  if (last_step < first_step)
  {
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointPosTerm comes before first step. Reversing them.");
  }
  if (last_step == -1)  // last_step not set
    last_step = first_step;

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointPosTermInfo coeffs", true);
  checkParameterSize(targets, n_dof, "JointPosTermInfo upper_tols", true);
  checkParameterSize(upper_tols, n_dof, "JointPosTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointPosTermInfo lower_tols", true);

  // Check if tolerances are all zeros
  bool is_upper_zeros =
      std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });
  bool is_lower_zeros =
      std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });

  // Get vars associated with joints
  trajopt::VarArray vars = prob.GetVars();
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(n_dof));
  if (prob.GetHasTime())
    ROS_INFO("JointPosTermInfo does not differ based on setting of TT_USE_TIME");

  if (term_type & TT_COST)
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(sco::CostPtr(
          new JointPosEqCost(joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(sco::CostPtr(new JointPosIneqCost(joint_vars,
                                                     util::toVectorXd(coeffs),
                                                     util::toVectorXd(targets),
                                                     util::toVectorXd(upper_tols),
                                                     util::toVectorXd(lower_tols),
                                                     first_step,
                                                     last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if (term_type & TT_CNT)
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(sco::ConstraintPtr(new JointPosEqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(sco::ConstraintPtr(new JointPosIneqConstraint(joint_vars,
                                                                       util::toVectorXd(coeffs),
                                                                       util::toVectorXd(targets),
                                                                       util::toVectorXd(upper_tols),
                                                                       util::toVectorXd(lower_tols),
                                                                       first_step,
                                                                       last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
  else
  {
    ROS_WARN("JointPosTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void JointVelTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  unsigned int n_dof = pci.kin->numJoints();
  json_marshal::childFromJson(params, targets, "targets");

  // Optional Parameters
  json_marshal::childFromJson(params, coeffs, "coeffs", DblVec(n_dof, 1));
  json_marshal::childFromJson(params, upper_tols, "upper_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, lower_tols, "lower_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, first_step, "first_step", 0);
  json_marshal::childFromJson(params, last_step, "last_step", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "coeffs", "first_step", "last_step", "targets", "lower_tols", "upper_tols" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointVelTermInfo::hatch(TrajOptProb& prob)
{
  unsigned int n_dof = prob.GetKin()->numJoints();

  // If optional parameter not given, set to default
  if (coeffs.empty())
    coeffs = DblVec(n_dof, 1);
  if (upper_tols.empty())
    upper_tols = DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = DblVec(n_dof, 0);
  if (last_step <= -1)
    last_step = prob.GetNumSteps() - 1;

  // If only one time step is desired, calculate velocity with next step (2 steps are needed for 1 velocity calculation)
  if ((prob.GetNumSteps() - 2) <= first_step)
    first_step = prob.GetNumSteps() - 2;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  if (last_step == first_step)
    last_step += 1;
  if (last_step < first_step)
  {
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointVelTerm comes before first step. Reversing them.");
  }

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointVelTermInfo coeffs", true);
  checkParameterSize(targets, n_dof, "JointVelTermInfo targets", true);
  checkParameterSize(upper_tols, n_dof, "JointVelTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointVelTermInfo lower_tols", true);
  assert(last_step > first_step);
  assert(first_step >= 0);

  // Check if tolerances are all zeros
  bool is_upper_zeros =
      std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });
  bool is_lower_zeros =
      std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });

  // Get vars associated with joints
  trajopt::VarArray vars = prob.GetVars();
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(n_dof));

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    unsigned num_vels = last_step - first_step;

    // Apply seperate cost to each joint b/c that is how the error function is currently written
    for (size_t j = 0; j < n_dof; j++)
    {
      // Get a vector of a single column of vars
      sco::VarVector joint_vars_vec = joint_vars.cblock(first_step, j, last_step - first_step + 1);
      sco::VarVector time_vars_vec = vars.cblock(first_step, vars.cols() - 1, last_step - first_step + 1);

      // If the tolerances are 0, an equality cost is set
      if (is_upper_zeros && is_lower_zeros)
      {
        DblVec single_jnt_coeffs = DblVec(num_vels * 2, coeffs[j]);
        prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(
            sco::VectorOfVectorPtr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVectorPtr(new JointVelJacCalculator()),
            concat(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs),
            sco::SQUARED,
            name + "_j" + std::to_string(j))));
      }
      // Otherwise it's a hinged "inequality" cost
      else
      {
        DblVec single_jnt_coeffs = DblVec(num_vels * 2, coeffs[j]);
        prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(
            sco::VectorOfVectorPtr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVectorPtr(new JointVelJacCalculator()),
            concat(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs),
            sco::HINGE,
            name + "_j" + std::to_string(j))));
      }
    }
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    unsigned num_vels = last_step - first_step;

    // Apply seperate cnt to each joint b/c that is how the error function is currently written
    for (size_t j = 0; j < n_dof; j++)
    {
      // Get a vector of a single column of vars
      sco::VarVector joint_vars_vec = joint_vars.cblock(first_step, j, last_step - first_step + 1);
      sco::VarVector time_vars_vec = vars.cblock(first_step, vars.cols() - 1, last_step - first_step + 1);

      // If the tolerances are 0, an equality cnt is set
      if (is_upper_zeros && is_lower_zeros)
      {
        DblVec single_jnt_coeffs = DblVec(num_vels * 2, coeffs[j]);
        prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(
            sco::VectorOfVectorPtr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVectorPtr(new JointVelJacCalculator()),
            concat(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs),
            sco::EQ,
            name + "_j" + std::to_string(j))));
      }
      // Otherwise it's a hinged "inequality" constraint
      else
      {
        DblVec single_jnt_coeffs = DblVec(num_vels * 2, coeffs[j]);
        prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(
            sco::VectorOfVectorPtr(new JointVelErrCalculator(targets[j], upper_tols[j], lower_tols[j])),
            sco::MatrixOfVectorPtr(new JointVelJacCalculator()),
            concat(joint_vars_vec, time_vars_vec),
            util::toVectorXd(single_jnt_coeffs),
            sco::INEQ,
            name + "_j" + std::to_string(j))));
      }
    }
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(sco::CostPtr(
          new JointVelEqCost(joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(sco::CostPtr(new JointVelIneqCost(joint_vars,
                                                     util::toVectorXd(coeffs),
                                                     util::toVectorXd(targets),
                                                     util::toVectorXd(upper_tols),
                                                     util::toVectorXd(lower_tols),
                                                     first_step,
                                                     last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(sco::ConstraintPtr(new JointVelEqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(sco::ConstraintPtr(new JointVelIneqConstraint(joint_vars,
                                                                       util::toVectorXd(coeffs),
                                                                       util::toVectorXd(targets),
                                                                       util::toVectorXd(upper_tols),
                                                                       util::toVectorXd(lower_tols),
                                                                       first_step,
                                                                       last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
  else
  {
    ROS_WARN("JointVelTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void JointAccTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  unsigned int n_dof = pci.kin->numJoints();
  json_marshal::childFromJson(params, targets, "targets");

  // Optional Parameters
  json_marshal::childFromJson(params, coeffs, "coeffs", DblVec(n_dof, 1));
  json_marshal::childFromJson(params, upper_tols, "upper_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, lower_tols, "lower_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, first_step, "first_step", 0);
  json_marshal::childFromJson(params, last_step, "last_step", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "coeffs", "first_step", "last_step", "targets", "lower_tols", "upper_tols" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointAccTermInfo::hatch(TrajOptProb& prob)
{
  unsigned int n_dof = prob.GetKin()->numJoints();

  // If optional parameter not given, set to default
  if (coeffs.empty())
    coeffs = DblVec(n_dof, 1);
  if (upper_tols.empty())
    upper_tols = DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = DblVec(n_dof, 0);
  if (last_step <= -1)
    last_step = prob.GetNumSteps() - 1;

  // Adjust final timesteps if calculating near the end of a trajectory
  if ((prob.GetNumSteps() - 3) <= first_step)
    first_step = prob.GetNumSteps() - 3;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  // If only one time step is desired, calculate velocity with next step (3 steps are needed for 1 accel calculation)
  if (last_step == first_step)
    last_step += 2;
  if (last_step < first_step)
  {
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointAccTerm comes before first step. Reversing them.");
  }

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointAccTermInfo coeffs", true);
  checkParameterSize(targets, n_dof, "JointAccTermInfo targets", true);
  checkParameterSize(upper_tols, n_dof, "JointAccTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointAccTermInfo lower_tols", true);

  // Check if tolerances are all zeros
  bool is_upper_zeros =
      std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });
  bool is_lower_zeros =
      std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });

  // Get vars associated with joints
  trajopt::VarArray vars = prob.GetVars();
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(n_dof));

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(sco::CostPtr(
          new JointAccEqCost(joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(sco::CostPtr(new JointAccIneqCost(joint_vars,
                                                     util::toVectorXd(coeffs),
                                                     util::toVectorXd(targets),
                                                     util::toVectorXd(upper_tols),
                                                     util::toVectorXd(lower_tols),
                                                     first_step,
                                                     last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(sco::ConstraintPtr(new JointAccEqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(sco::ConstraintPtr(new JointAccIneqConstraint(joint_vars,
                                                                       util::toVectorXd(coeffs),
                                                                       util::toVectorXd(targets),
                                                                       util::toVectorXd(upper_tols),
                                                                       util::toVectorXd(lower_tols),
                                                                       first_step,
                                                                       last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
  else
  {
    ROS_WARN("JointAccTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void JointJerkTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  unsigned int n_dof = pci.kin->numJoints();

  json_marshal::childFromJson(params, targets, "targets");

  // Optional Parameters
  json_marshal::childFromJson(params, coeffs, "coeffs", DblVec(n_dof, 1));
  json_marshal::childFromJson(params, upper_tols, "upper_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, lower_tols, "lower_tols", DblVec(n_dof, 0));
  json_marshal::childFromJson(params, first_step, "first_step", 0);
  json_marshal::childFromJson(params, last_step, "last_step", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "coeffs", "first_step", "last_step", "targets", "lower_tols", "upper_tols" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointJerkTermInfo::hatch(TrajOptProb& prob)
{
  unsigned int n_dof = prob.GetKin()->numJoints();

  // If optional parameter not given, set to default
  if (coeffs.empty())
    coeffs = DblVec(n_dof, 1);
  if (upper_tols.empty())
    upper_tols = DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = DblVec(n_dof, 0);
  if (last_step <= -1)
    last_step = prob.GetNumSteps() - 1;

  // Adjust final timesteps if calculating near the end of a trajectory
  if ((prob.GetNumSteps() - 4) <= first_step)
    first_step = prob.GetNumSteps() - 4;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  // If only one time step is desired, calculate velocity with next step (5 steps are needed for 1 jerk calculation)
  if (last_step == first_step)
    last_step += 4;
  if (last_step < first_step)
  {
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointJerkTerm comes before first step. Reversing them.");
  }

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointJerkTermInfo coeffs", true);
  checkParameterSize(targets, n_dof, "JointJerkTermInfo targets", true);
  checkParameterSize(upper_tols, n_dof, "JointJerkTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointJerkTermInfo lower_tols", true);

  // Check if tolerances are all zeros
  bool is_upper_zeros =
      std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });
  bool is_lower_zeros =
      std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return util::doubleEquals(i, 0.); });

  // Get vars associated with joints
  trajopt::VarArray vars = prob.GetVars();
  trajopt::VarArray joint_vars = vars.block(0, 0, vars.rows(), static_cast<int>(n_dof));

  if (term_type == (TT_COST | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if (term_type == (TT_CNT | TT_USE_TIME))
  {
    ROS_ERROR("Use time version of this term has not been defined.");
  }
  else if ((term_type & TT_COST) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(sco::CostPtr(
          new JointJerkEqCost(joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(sco::CostPtr(new JointJerkIneqCost(joint_vars,
                                                      util::toVectorXd(coeffs),
                                                      util::toVectorXd(targets),
                                                      util::toVectorXd(upper_tols),
                                                      util::toVectorXd(lower_tols),
                                                      first_step,
                                                      last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if ((term_type & TT_CNT) && ~(term_type | ~TT_USE_TIME))
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(sco::ConstraintPtr(new JointJerkEqConstraint(
          joint_vars, util::toVectorXd(coeffs), util::toVectorXd(targets), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(sco::ConstraintPtr(new JointJerkIneqConstraint(joint_vars,
                                                                        util::toVectorXd(coeffs),
                                                                        util::toVectorXd(targets),
                                                                        util::toVectorXd(upper_tols),
                                                                        util::toVectorXd(lower_tols),
                                                                        first_step,
                                                                        last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
  else
  {
    ROS_WARN("JointJerkTermInfo does not have a valid term_type defined. No cost/constraint applied");
  }
}

void CollisionTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  int n_steps = pci.basic_info.n_steps;
  json_marshal::childFromJson(params, continuous, "continuous", true);
  json_marshal::childFromJson(params, first_step, "first_step", 0);
  json_marshal::childFromJson(params, last_step, "last_step", n_steps - 1);
  json_marshal::childFromJson(params, gap, "gap", 1);
  FAIL_IF_FALSE(gap >= 0);
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));

  DblVec coeffs, dist_pen;
  json_marshal::childFromJson(params, coeffs, "coeffs");
  int n_terms = last_step - first_step + 1;
  if (coeffs.size() == 1)
    coeffs = DblVec(static_cast<size_t>(n_terms), coeffs[0]);
  else if (static_cast<int>(coeffs.size()) != n_terms)
  {
    PRINT_AND_THROW(boost::format("wrong size: coeffs. expected %i got %i") % n_terms % coeffs.size());
  }
  json_marshal::childFromJson(params, dist_pen, "dist_pen");
  if (dist_pen.size() == 1)
    dist_pen = DblVec(static_cast<size_t>(n_terms), dist_pen[0]);
  else if (static_cast<int>(dist_pen.size()) != n_terms)
  {
    PRINT_AND_THROW(boost::format("wrong size: dist_pen. expected %i got %i") % n_terms % dist_pen.size());
  }

  // Create Contact Distance Data for each timestep
  info.reserve(static_cast<size_t>(n_terms));
  for (int i = first_step; i <= last_step; ++i)
  {
    size_t index = static_cast<size_t>(i - first_step);
    SafetyMarginDataPtr data(new SafetyMarginData(dist_pen[index], coeffs[index]));
    info.push_back(data);
  }

  // Check if data was set for individual pairs
  if (params.isMember("pairs"))
  {
    const Json::Value& pairs = params["pairs"];
    for (Json::Value::const_iterator it = pairs.begin(); it != pairs.end(); ++it)
    {
      FAIL_IF_FALSE(it->isMember("link"));
      std::string link;
      json_marshal::childFromJson(*it, link, "link");

      FAIL_IF_FALSE(it->isMember("pair"));
      std::vector<std::string> pair;
      json_marshal::childFromJson(*it, pair, "pair");

      if (pair.size() == 0)
      {
        PRINT_AND_THROW(boost::format("wrong size: pair. expected > 0 got %i") % pair.size());
      }

      DblVec pair_coeffs;
      json_marshal::childFromJson(*it, pair_coeffs, "coeffs");
      if (pair_coeffs.size() == 1)
      {
        pair_coeffs = DblVec(static_cast<size_t>(n_terms), pair_coeffs[0]);
      }
      else if (static_cast<int>(pair_coeffs.size()) != n_terms)
      {
        PRINT_AND_THROW(boost::format("wrong size: coeffs. expected %i got %i") % n_terms % pair_coeffs.size());
      }

      DblVec pair_dist_pen;
      json_marshal::childFromJson(*it, pair_dist_pen, "dist_pen");
      if (pair_dist_pen.size() == 1)
      {
        pair_dist_pen = DblVec(static_cast<size_t>(n_terms), pair_dist_pen[0]);
      }
      else if (static_cast<int>(pair_dist_pen.size()) != n_terms)
      {
        PRINT_AND_THROW(boost::format("wrong size: dist_pen. expected %i got %i") % n_terms % pair_dist_pen.size());
      }

      for (auto i = first_step; i <= last_step; ++i)
      {
        size_t index = static_cast<size_t>(i - first_step);
        SafetyMarginDataPtr& data = info[index];
        for (auto j = 0u; j < pair.size(); ++j)
        {
          data->SetPairSafetyMarginData(link, pair[j], pair_dist_pen[index], pair_coeffs[index]);
        }
      }
    }
  }

  const char* all_fields[] = { "continuous", "first_step", "last_step", "gap", "coeffs", "dist_pen", "pairs" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CollisionTermInfo::hatch(TrajOptProb& prob)
{
  int n_dof = static_cast<int>(prob.GetKin()->numJoints());

  if (term_type == TT_COST)
  {
    if (continuous)
    {
      for (int i = first_step; i <= last_step - gap; ++i)
      {
        prob.addCost(sco::CostPtr(new CollisionCost(prob.GetKin(),
                                                    prob.GetEnv(),
                                                    info[static_cast<size_t>(i - first_step)],
                                                    prob.GetVarRow(i, 0, n_dof),
                                                    prob.GetVarRow(i + gap, 0, n_dof))));
        prob.getCosts().back()->setName((boost::format("%s_%i") % name.c_str() % i).str());
      }
    }
    else
    {
      for (int i = first_step; i <= last_step; ++i)
      {
        prob.addCost(sco::CostPtr(new CollisionCost(
            prob.GetKin(), prob.GetEnv(), info[static_cast<size_t>(i - first_step)], prob.GetVarRow(i, 0, n_dof))));
        prob.getCosts().back()->setName((boost::format("%s_%i") % name.c_str() % i).str());
      }
    }
  }
  else
  {  // ALMOST COPIED
    if (continuous)
    {
      for (int i = first_step; i < last_step; ++i)
      {
        prob.addIneqConstraint(sco::ConstraintPtr(new CollisionConstraint(prob.GetKin(),
                                                                          prob.GetEnv(),
                                                                          info[static_cast<size_t>(i - first_step)],
                                                                          prob.GetVarRow(i, 0, n_dof),
                                                                          prob.GetVarRow(i + 1, 0, n_dof))));
        prob.getIneqConstraints().back()->setName((boost::format("%s_%i") % name.c_str() % i).str());
      }
    }
    else
    {
      for (int i = first_step; i <= last_step; ++i)
      {
        prob.addIneqConstraint(sco::ConstraintPtr(new CollisionConstraint(
            prob.GetKin(), prob.GetEnv(), info[static_cast<size_t>(i - first_step)], prob.GetVarRow(i, 0, n_dof))));
        prob.getIneqConstraints().back()->setName((boost::format("%s_%i") % name.c_str() % i).str());
      }
    }
  }
}

void TotalTimeTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  json_marshal::childFromJson(params, coeff, "coeff", 1.0);
  json_marshal::childFromJson(params, limit, "limit", 1.0);

  const char* all_fields[] = { "weight", "penalty_type", "limit" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void TotalTimeTermInfo::hatch(TrajOptProb& prob)
{
  Eigen::VectorXd coeff_vec(1);
  coeff_vec[0] = coeff;

  // get all (1/dt) vars except the first
  sco::VarVector time_vars(prob.GetNumSteps() - 1);
  for (std::size_t i = 0; i < time_vars.size(); i++)
  {
    time_vars[i] = prob.GetVar(i + 1, prob.GetNumDOF() - 1);
  }

  // Get correct penalty type
  sco::PenaltyType penalty_type;
  sco::ConstraintType constraint_type;
  if (util::doubleEquals(limit, 0.0))
  {
    penalty_type = sco::SQUARED;
    constraint_type = sco::EQ;
  }
  else
  {
    penalty_type = sco::HINGE;
    constraint_type = sco::INEQ;
  }

  if (term_type & TT_COST)
  {
    prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(sco::VectorOfVectorPtr(new TimeCostCalculator(limit)),
                                                         sco::MatrixOfVectorPtr(new TimeCostJacCalculator()),
                                                         time_vars,
                                                         coeff_vec,
                                                         penalty_type,
                                                         name)));
  }
  else if (term_type & TT_CNT)
  {
    prob.addConstraint(
        sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(sco::VectorOfVectorPtr(new TimeCostCalculator(limit)),
                                                            sco::MatrixOfVectorPtr(new TimeCostJacCalculator()),
                                                            time_vars,
                                                            coeff_vec,
                                                            constraint_type,
                                                            name)));
  }
  else
  {
    PRINT_AND_THROW("A valid term type was not specified in TotalTimeTermInfo");
  }
}

}  // namespace trajopt
