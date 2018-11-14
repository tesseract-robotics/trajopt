#include <boost/algorithm/string.hpp>
#include <tesseract_core/basic_kin.h>
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
#include <ros/ros.h>

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
      PRINT_AND_THROW(boost::format("invalid field found: %1") % it.name());
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

  gRegisteredMakers = true;
}

sco::PenaltyType stringToPenaltyType(std::string str)
{
  if (boost::iequals(str, "hinge"))
  {
    return sco::HINGE;
  }
  else if (boost::iequals(str, "abs"))
  {
    return sco::ABS;
  }
  else if (boost::iequals(str, "squared"))
  {
    return sco::SQUARED;
  }
  else
  {
    PRINT_AND_THROW(boost::format("Failed string to PenaltyType conversion. %s is not a valid penalty type") % str);
  }
}

/**
 * @brief Checks the size of the parameter given and throws if incorrect
 * @param parameter The vector whose size is getting checked
 * @param expected_size The expected size of the vector
 * @param name The name to use when printing an error or warning
 * @param apply_first If true and only one value is given, broadcast value to length of expected_size
 */
void checkParameterSize(DblVec& parameter,
                        const unsigned int& expected_size,
                        const std::string& name,
                        const bool& apply_first = true)
{
  if (apply_first == true && parameter.size() == 1)
  {
    parameter = DblVec(expected_size, parameter[0]);
    ROS_INFO("1 %s given. Applying to all %i joints", name, expected_size);
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
  // TODO: optimization parameters, etc?
}

void ProblemConstructionInfo::readOptInfo(const Json::Value& v)
{
  json_marshal::childFromJson(v, opt_info.improve_ratio_threshold, "improve_ratio_threshold", opt_info.improve_ratio_threshold);
  json_marshal::childFromJson(v, opt_info.min_trust_box_size, "min_trust_box_size", opt_info.min_trust_box_size);
  json_marshal::childFromJson(v, opt_info.min_approx_improve, "min_approx_improve", opt_info.min_approx_improve);
  json_marshal::childFromJson(v, opt_info.min_approx_improve_frac, "min_approx_improve_frac", opt_info.min_approx_improve_frac);
  json_marshal::childFromJson(v, opt_info.max_iter, "max_iter", opt_info.max_iter);
  json_marshal::childFromJson(v, opt_info.trust_shrink_ratio, "trust_shrink_ratio", opt_info.trust_shrink_ratio);
  json_marshal::childFromJson(v, opt_info.trust_expand_ratio, "trust_expand_ratio", opt_info.trust_expand_ratio);
  json_marshal::childFromJson(v, opt_info.cnt_tolerance, "cnt_tolerance", opt_info.cnt_tolerance);
  json_marshal::childFromJson(v, opt_info.max_merit_coeff_increases, "max_merit_coeff_increases", opt_info.max_merit_coeff_increases);
  json_marshal::childFromJson(v, opt_info.merit_coeff_increase_ratio, "merit_coeff_increase_ratio", opt_info.merit_coeff_increase_ratio);
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
    std::string type;
    json_marshal::childFromJson(*it, type, "type");
    LOG_DEBUG("reading term: %s", type.c_str());
    TermInfoPtr term = TermInfo::fromName(type);

    if (!term)
      PRINT_AND_THROW(boost::format("failed to construct cost named %s") % type);
    if (!dynamic_cast<MakesCost*>(term.get()))
      PRINT_AND_THROW(boost::format("%s is only a constraint, but you listed it as a cost") % type);
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
    std::string type;
    json_marshal::childFromJson(*it, type, "type");
    LOG_DEBUG("reading term: %s", type.c_str());
    TermInfoPtr term = TermInfo::fromName(type);

    if (!term)
      PRINT_AND_THROW(boost::format("failed to construct constraint named %s") % type);
    if (!dynamic_cast<MakesConstraint*>(term.get()))
      PRINT_AND_THROW(boost::format("%s is only a cost, but you listed it as a constraint") % type);
    term->term_type = TT_CNT;

    term->fromJson(*this, *it);
    json_marshal::childFromJson(*it, term->name, "name", type);

    cnt_infos.push_back(term);
  }
}

void ProblemConstructionInfo::readInitInfo(const Json::Value& v)
{
  std::string type_str;
  json_marshal::childFromJson(v, type_str, "type");
  int n_steps = basic_info.n_steps;
  int n_dof = kin->numJoints();
  Eigen::VectorXd start_pos = env->getCurrentJointValues(kin->getName());

  if (type_str == "stationary")
  {
    init_info.data = start_pos.transpose().replicate(n_steps, 1);
  }
  else if (type_str == "given_traj")
  {
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
      json_marshal::fromJsonArray(vdata[i], row, n_dof);
      init_info.data.row(i) = util::toVectorXd(row);
    }
  }
  else if (type_str == "straight_line")
  {
    FAIL_IF_FALSE(v.isMember("endpoint"));
    DblVec endpoint;
    json_marshal::childFromJson(v, endpoint, "endpoint");
    if (static_cast<int>(endpoint.size()) != n_dof)
    {
      PRINT_AND_THROW(boost::format("wrong number of dof values in "
                                    "initialization. expected %i got %j") %
                      n_dof % endpoint.size());
    }
    init_info.data = TrajArray(n_steps, n_dof);
    for (int idof = 0; idof < n_dof; ++idof)
    {
      init_info.data.col(idof) = Eigen::VectorXd::LinSpaced(n_steps, start_pos[idof], endpoint[idof]);
    }
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
    PRINT_AND_THROW(boost::format("Manipulator does not exist: %s") % basic_info.manip);
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

TrajOptResult::TrajOptResult(sco::OptResults& opt, TrajOptProb& prob) : cost_vals(opt.cost_vals), cnt_viols(opt.cnt_viols)
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

  TrajOptProbPtr prob(new TrajOptProb(n_steps, pci));
  int n_dof = prob->GetKin()->numJoints();

  if (bi.start_fixed)
  {
    if (pci.init_info.data.rows() < 1)
    {
      PRINT_AND_THROW("Initial trajectory must contain at least the start state.");
    }

    if (pci.init_info.data.cols() != n_dof)
    {
      PRINT_AND_THROW("robot dof values don't match initialization. I don't "
                      "know what you want me to use for the dof values");
    }

    for (int j = 0; j < n_dof; ++j)
    {
      prob->addLinearConstraint(sco::exprSub(sco::AffExpr(prob->m_traj_vars(0, j)), pci.init_info.data(0, j)), sco::EQ);
    }
  }

  if (!bi.dofs_fixed.empty())
  {
    for (const int& dof_ind : bi.dofs_fixed)
    {
      for (int i = 1; i < prob->GetNumSteps(); ++i)
      {
        prob->addLinearConstraint(sco::exprSub(sco::AffExpr(prob->m_traj_vars(i, dof_ind)), sco::AffExpr(prob->m_traj_vars(0, dof_ind))), sco::EQ);
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

  prob->SetInitTraj(pci.init_info.data);

  return prob;
}

TrajOptProbPtr ConstructProblem(const Json::Value& root, tesseract::BasicEnvConstPtr env)
{
  ProblemConstructionInfo pci(env);
  pci.fromJson(root);
  return ConstructProblem(pci);
}

TrajOptProb::TrajOptProb(int n_steps, const ProblemConstructionInfo& pci) : m_kin(pci.kin), m_env(pci.env)
{
  const Eigen::MatrixX2d& limits = m_kin->getLimits();
  int n_dof = m_kin->numJoints();
  Eigen::VectorXd lower, upper;
  lower = limits.col(0);
  upper = limits.col(1);

  DblVec vlower, vupper;
  std::vector<std::string> names;
  for (int i = 0; i < n_steps; ++i)
  {
    vlower.insert(vlower.end(), lower.data(), lower.data() + lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data() + upper.size());
    for (int j = 0; j < n_dof; ++j)
    {
      names.push_back((boost::format("j_%i_%i") % i % j).str());
    }
  }
  sco::VarVector trajvarvec = createVariables(names, vlower, vupper);
  m_traj_vars = VarArray(n_steps, n_dof, trajvarvec.data());
}

TrajOptProb::TrajOptProb() {}

DynamicCartPoseTermInfo::DynamicCartPoseTermInfo()
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
  sco::VectorOfVectorPtr f(new DynamicCartPoseErrCalculator(target, prob.GetKin(), prob.GetEnv(), link, tcp));
  if (term_type == TT_COST)
  {
    prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), sco::ABS, name)));
  }
  else if (term_type == TT_CNT)
  {
    prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), sco::EQ, name)));
  }
  else
  {
    ROS_WARN("DynamicCartPoseTermInfo does not have a term_type defined. No cost/constraint applied");
  }
}

CartPoseTermInfo::CartPoseTermInfo()
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
  Eigen::Isometry3d input_pose;
  Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  input_pose.linear() = q.matrix();
  input_pose.translation() = xyz;

  sco::VectorOfVectorPtr f(new CartPoseErrCalculator(input_pose, prob.GetKin(), prob.GetEnv(), link, tcp));
  if (term_type == TT_COST)
  {
    prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), sco::ABS, name)));
  }
  else if (term_type == TT_CNT)
  {
    prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), sco::EQ, name)));
  }
  else
  {
    ROS_WARN("CartPoseTermInfo does not have a term_type defined. No cost/constraint applied");
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
  if (term_type == TT_COST)
  {
    for (int iStep = first_step; iStep < last_step; ++iStep)
    {
      prob.addCost(sco::CostPtr(new TrajOptCostFromErrFunc(
          sco::VectorOfVectorPtr(new CartVelErrCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          sco::MatrixOfVectorPtr(new CartVelJacCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          concat(prob.GetVarRow(iStep), prob.GetVarRow(iStep + 1)),
          Eigen::VectorXd::Ones(0),
          sco::ABS,
          name)));
    }
  }
  else if (term_type == TT_CNT)
  {
    for (int iStep = first_step; iStep < last_step; ++iStep)
    {
      prob.addConstraint(sco::ConstraintPtr(new TrajOptConstraintFromErrFunc(
          sco::VectorOfVectorPtr(new CartVelErrCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          sco::MatrixOfVectorPtr(new CartVelJacCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
          concat(prob.GetVarRow(iStep), prob.GetVarRow(iStep + 1)),
          Eigen::VectorXd::Ones(0),
          sco::INEQ,
          "CartVel")));
    }
  }
  else
  {
    ROS_WARN("CartVelTermInfo does not have a term_type defined. No cost/constraint applied");
  }
}

void JointPosTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  int n_steps = pci.basic_info.n_steps;
  const Json::Value& params = v["params"];
  json_marshal::childFromJson(params, vals, "vals");
  json_marshal::childFromJson(params, coeffs, "coeffs");
  if (coeffs.size() == 1)
    coeffs = DblVec(n_steps, coeffs[0]);
  unsigned n_dof = pci.kin->numJoints();
  if (vals.size() != n_dof)
  {
    PRINT_AND_THROW(boost::format("wrong number of dof vals. expected %i got %i") % n_dof % vals.size());
  }
  json_marshal::childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "vals", "coeffs", "timestep" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointPosTermInfo::hatch(TrajOptProb& prob)
{
  if (term_type == TT_COST)
  {
    prob.addCost(sco::CostPtr(new JointPosCost(prob.GetVarRow(timestep), util::toVectorXd(vals), util::toVectorXd(coeffs))));
    prob.getCosts().back()->setName(name);
  }
  else if (term_type == TT_CNT)
  {
    sco::VarVector vars = prob.GetVarRow(timestep);
    int n_dof = vars.size();
    for (int j = 0; j < n_dof; ++j)
    {
      prob.addLinearConstraint(sco::exprSub(sco::AffExpr(vars[j]), vals[j]), sco::EQ);
    }
  }
  else
  {
    ROS_WARN("JointPosTermInfo does not have a term_type defined. No cost/constraint applied");
  }
}

void JointVelTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  unsigned int n_dof = pci.kin->numJoints();
  childFromJson(params, coeffs, "coeffs");

  // Optional Parameters
  childFromJson(params, targs, "targs", DblVec(n_dof, 0));
  childFromJson(params, upper_tols, "upper_tols", DblVec(n_dof, 0));
  childFromJson(params, lower_tols, "lower_tols", DblVec(n_dof, 0));
  childFromJson(params, first_step, "first_step", 0);
  childFromJson(params, last_step, "last_step", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "coeffs", "first_step", "last_step", "targs", "lower_tols", "upper_tols" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointVelTermInfo::hatch(TrajOptProb& prob)
{
  if ((term_type != TT_COST) && (term_type != TT_CNT))
  {
    ROS_WARN("JointVelTermInfo does not have a term_type defined. No cost/constraint applied");
  }
  unsigned int n_dof = prob.GetKin()->getJointNames().size();

  // If target or tolerance is not given, set all to 0
  if (targs.empty())
    targs = DblVec(n_dof, 0);
  if (upper_tols.empty())
    upper_tols = DblVec(n_dof, 0);
  if (lower_tols.empty())
    lower_tols = DblVec(n_dof, 0);

  // If only one time step is desired, calculate velocity with next step (2 steps are needed for 1 velocity calculation)
  if ((prob.GetNumSteps() - 2) <= first_step)
    first_step = prob.GetNumSteps() - 2;
  if ((prob.GetNumSteps() - 1) <= last_step)
    last_step = prob.GetNumSteps() - 1;
  if (last_step == first_step)
    last_step += 1;
  if (last_step < first_step){
    int tmp = first_step;
    first_step = last_step;
    last_step = tmp;
    ROS_WARN("Last time step for JointVelTerm comes before first step. Reversing them.");
  }

  // Check if parameters are the correct size.
  checkParameterSize(coeffs, n_dof, "JointVelTermInfo coeffs", true);
  checkParameterSize(targs, n_dof, "JointVelTermInfo upper_tols", true);
  checkParameterSize(upper_tols, n_dof, "JointVelTermInfo upper_tols", true);
  checkParameterSize(lower_tols, n_dof, "JointVelTermInfo lower_tols", true);

  // Check if tolerances are all zeros
  bool is_upper_zeros = std::all_of(upper_tols.begin(), upper_tols.end(), [](double i) { return doubleEquals(i,0.); });
  bool is_lower_zeros = std::all_of(lower_tols.begin(), lower_tols.end(), [](double i) { return doubleEquals(i,0.); });

  if (term_type == TT_COST)
  {
    // If the tolerances are 0, an equality cost is set. Otherwise it's a hinged "inequality" cost
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addCost(CostPtr(new JointVelEqCost(prob.GetVars(), toVectorXd(coeffs), toVectorXd(targs), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
    else
    {
      prob.addCost(CostPtr(new JointVelIneqCost(
          prob.GetVars(), toVectorXd(coeffs), toVectorXd(targs), toVectorXd(upper_tols), toVectorXd(lower_tols), first_step, last_step)));
      prob.getCosts().back()->setName(name);
    }
  }
  else if (term_type == TT_CNT)
  {
    // If the tolerances are 0, an equality cnt is set. Otherwise it's an inequality constraint
    if (is_upper_zeros && is_lower_zeros)
    {
      prob.addConstraint(
          ConstraintPtr(new JointVelEqConstraint(prob.GetVars(), toVectorXd(coeffs), toVectorXd(targs), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
    else
    {
      prob.addConstraint(ConstraintPtr(new JointVelIneqConstraint(
          prob.GetVars(), toVectorXd(coeffs), toVectorXd(targs), toVectorXd(upper_tols), toVectorXd(lower_tols), first_step, last_step)));
      prob.getConstraints().back()->setName(name);
    }
  }
}

void JointAccTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  json_marshal::childFromJson(params, coeffs, "coeffs");
  unsigned n_dof = pci.kin->numJoints();
  if (coeffs.size() == 1)
    coeffs = DblVec(n_dof, coeffs[0]);
  else if (coeffs.size() != n_dof)
  {
    PRINT_AND_THROW(boost::format("wrong number of coeffs. expected %i got %i") % n_dof % coeffs.size());
  }

  const char* all_fields[] = { "coeffs" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointAccTermInfo::hatch(TrajOptProb& prob)
{
  if (term_type == TT_COST)
  {
    prob.addCost(sco::CostPtr(new JointAccCost(prob.GetVars(), util::toVectorXd(coeffs))));
    prob.getCosts().back()->setName(name);
  }
  else if (term_type == TT_CNT)
  {
    // Calculate acceleration as ((i+1) - 2i + (i-1))/dt where dt=1
    for (int i = first_step - 1; i <= last_step - 1; ++i)
    {
      for (std::size_t j = 0; j < coeffs.size(); ++j)
      {
        sco::AffExpr acc = prob.GetVar(i + 1, j) - 2 * prob.GetVar(i, j) + prob.GetVar(i - 1, j);
        prob.addLinearConstraint(acc - coeffs[j], sco::INEQ);
        prob.addLinearConstraint(-acc - coeffs[j], sco::INEQ);
      }
    }
  }
  else
  {
    ROS_WARN("JointAccTermInfo does not have a term_type defined. No cost/constraint applied");
  }
}

void JointJerkTermInfo::fromJson(ProblemConstructionInfo& pci, const Json::Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Json::Value& params = v["params"];

  json_marshal::childFromJson(params, coeffs, "coeffs");
  unsigned n_dof = pci.kin->numJoints();
  if (coeffs.size() == 1)
    coeffs = DblVec(n_dof, coeffs[0]);
  else if (coeffs.size() != n_dof)
  {
    PRINT_AND_THROW(boost::format("wrong number of coeffs. expected %i got %i") % n_dof % coeffs.size());
  }

  const char* all_fields[] = { "coeffs" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointJerkTermInfo::hatch(TrajOptProb& prob)
{
  if (term_type == TT_COST)
  {
    prob.addCost(sco::CostPtr(new JointJerkCost(prob.GetVars(), util::toVectorXd(coeffs))));
    prob.getCosts().back()->setName(name);
  }
  else if (term_type == TT_CNT)
  {
    // Calculate jerk as ((i+2) - 3(i+1) + 3(i) -(i-1))/dt where dt=1
    for (int i = first_step - 1; i <= last_step - 1; ++i)
    {
      for (std::size_t j = 0; j < coeffs.size(); ++j)
      {
        sco::AffExpr jerk =
            prob.GetVar(i + 2, j) - 3 * prob.GetVar(i + 1, j) + 3 * prob.GetVar(i, j) - prob.GetVar(i - 1, j);
        prob.addLinearConstraint(jerk - coeffs[j], sco::INEQ);
        prob.addLinearConstraint(-jerk - coeffs[j], sco::INEQ);
      }
    }
  }
  else
  {
    ROS_WARN("JointJerkTermInfo does not have a term_type defined. No cost/constraint applied");
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
    coeffs = DblVec(n_terms, coeffs[0]);
  else if (static_cast<int>(coeffs.size()) != n_terms)
  {
    PRINT_AND_THROW(boost::format("wrong size: coeffs. expected %i got %i") % n_terms % coeffs.size());
  }
  json_marshal::childFromJson(params, dist_pen, "dist_pen");
  if (dist_pen.size() == 1)
    dist_pen = DblVec(n_terms, dist_pen[0]);
  else if (static_cast<int>(dist_pen.size()) != n_terms)
  {
    PRINT_AND_THROW(boost::format("wrong size: dist_pen. expected %i got %i") % n_terms % dist_pen.size());
  }

  // Create Contact Distance Data for each timestep
  info.reserve(n_terms);
  for (int i = first_step; i <= last_step; ++i)
  {
    SafetyMarginDataPtr data(new SafetyMarginData(dist_pen[i - first_step], coeffs[i - first_step]));
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
        pair_coeffs = DblVec(n_terms, pair_coeffs[0]);
      }
      else if (static_cast<int>(pair_coeffs.size()) != n_terms)
      {
        PRINT_AND_THROW(boost::format("wrong size: coeffs. expected %i got %i") % n_terms % pair_coeffs.size());
      }

      DblVec pair_dist_pen;
      json_marshal::childFromJson(*it, pair_dist_pen, "dist_pen");
      if (pair_dist_pen.size() == 1)
      {
        pair_dist_pen = DblVec(n_terms, pair_dist_pen[0]);
      }
      else if (static_cast<int>(pair_dist_pen.size()) != n_terms)
      {
        PRINT_AND_THROW(boost::format("wrong size: dist_pen. expected %i got %i") % n_terms % pair_dist_pen.size());
      }

      for (auto i = first_step; i <= last_step; ++i)
      {
        auto index = i - first_step;
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
  if (term_type == TT_COST)
  {
    if (continuous)
    {
      for (int i = first_step; i <= last_step - gap; ++i)
      {
        prob.addCost(sco::CostPtr(new CollisionCost(
            prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i), prob.GetVarRow(i + gap))));
        prob.getCosts().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
    else
    {
      for (int i = first_step; i <= last_step; ++i)
      {
        prob.addCost(sco::CostPtr(new CollisionCost(prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i))));
        prob.getCosts().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
  }
  else
  {  // ALMOST COPIED
    if (continuous)
    {
      for (int i = first_step; i < last_step; ++i)
      {
        prob.addIneqConstraint(sco::ConstraintPtr(new CollisionConstraint(
            prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i), prob.GetVarRow(i + 1))));
        prob.getIneqConstraints().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
    else
    {
      for (int i = first_step; i <= last_step; ++i)
      {
        prob.addIneqConstraint(sco::ConstraintPtr(
            new CollisionConstraint(prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i))));
        prob.getIneqConstraints().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
  }
}

}  // namespace trajopt
