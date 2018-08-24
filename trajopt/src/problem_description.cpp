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

using namespace Json;
using namespace std;
using namespace trajopt;
using namespace util;

namespace
{
bool gRegisteredMakers = false;

void ensure_only_members(const Value& v, const char** fields, int nvalid)
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
  TermInfo::RegisterMaker("pose", &PoseCostInfo::create);
  TermInfo::RegisterMaker("static_pose", &StaticPoseCostInfo::create);
  TermInfo::RegisterMaker("joint_pos", &JointPosCostInfo::create);
  TermInfo::RegisterMaker("joint_vel", &JointVelCostInfo::create);
  TermInfo::RegisterMaker("joint_acc", &JointAccCostInfo::create);
  TermInfo::RegisterMaker("joint_jerk", &JointJerkCostInfo::create);
  TermInfo::RegisterMaker("collision", &CollisionCostInfo::create);

  TermInfo::RegisterMaker("joint", &JointConstraintInfo::create);
  TermInfo::RegisterMaker("cart_vel", &CartVelCntInfo::create);
  TermInfo::RegisterMaker("joint_vel_limits", &JointVelConstraintInfo::create);
  TermInfo::RegisterMaker("aligned_axis", &AlignedAxisTermInfo::create);
  TermInfo::RegisterMaker("conical_axis", &ConicalAxisTermInfo::create);

  gRegisteredMakers = true;
}

#if 0
BoolVec toMask(const VectorXd& x) {
  BoolVec out(x.size());
  for (int i=0; i < x.size(); ++i) out[i] = (x[i] > 0);
  return out;
}
#endif
}

namespace trajopt
{
std::map<string, TermInfo::MakerFunc> TermInfo::name2maker;
void TermInfo::RegisterMaker(const std::string& type, MakerFunc f) { name2maker[type] = f; }
TermInfoPtr TermInfo::fromName(const string& type)
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

void ProblemConstructionInfo::readBasicInfo(const Value& v)
{
  childFromJson(v, basic_info.start_fixed, "start_fixed", true);
  childFromJson(v, basic_info.n_steps, "n_steps");
  childFromJson(v, basic_info.manip, "manip");
  childFromJson(v, basic_info.robot, "robot", string(""));
  childFromJson(v, basic_info.dofs_fixed, "dofs_fixed", IntVec());
  // TODO: optimization parameters, etc?
}

void ProblemConstructionInfo::readOptInfo(const Value& v)
{
  childFromJson(v, opt_info.improve_ratio_threshold, "improve_ratio_threshold", opt_info.improve_ratio_threshold);
  childFromJson(v, opt_info.min_trust_box_size, "min_trust_box_size", opt_info.min_trust_box_size);
  childFromJson(v, opt_info.min_approx_improve, "min_approx_improve", opt_info.min_approx_improve);
  childFromJson(v, opt_info.min_approx_improve_frac, "min_approx_improve_frac", opt_info.min_approx_improve_frac);
  childFromJson(v, opt_info.max_iter, "max_iter", opt_info.max_iter);
  childFromJson(v, opt_info.trust_shrink_ratio, "trust_shrink_ratio", opt_info.trust_shrink_ratio);
  childFromJson(v, opt_info.trust_expand_ratio, "trust_expand_ratio", opt_info.trust_expand_ratio);
  childFromJson(v, opt_info.cnt_tolerance, "cnt_tolerance", opt_info.cnt_tolerance);
  childFromJson(v, opt_info.max_merit_coeff_increases, "max_merit_coeff_increases", opt_info.max_merit_coeff_increases);
  childFromJson(
      v, opt_info.merit_coeff_increase_ratio, "merit_coeff_increase_ratio", opt_info.merit_coeff_increase_ratio);
  childFromJson(v, opt_info.max_time, "max_time", opt_info.max_time);
  childFromJson(v, opt_info.merit_error_coeff, "merit_error_coeff", opt_info.merit_error_coeff);
  childFromJson(v, opt_info.trust_box_size, "trust_box_size", opt_info.trust_box_size);
}

void ProblemConstructionInfo::readCosts(const Value& v)
{
  cost_infos.clear();
  cost_infos.reserve(v.size());
  for (Json::Value::const_iterator it = v.begin(); it != v.end(); ++it)
  {
    string type;
    childFromJson(*it, type, "type");
    LOG_DEBUG("reading term: %s", type.c_str());
    TermInfoPtr term = TermInfo::fromName(type);

    if (!term)
      PRINT_AND_THROW(boost::format("failed to construct cost named %s") % type);
    if (!dynamic_cast<MakesCost*>(term.get()))
      PRINT_AND_THROW(boost::format("%s is only a constraint, but you listed it as a cost") % type);
    term->term_type = TT_COST;

    term->fromJson(*this, *it);
    childFromJson(*it, term->name, "name", type);

    cost_infos.push_back(term);
  }
}

void ProblemConstructionInfo::readConstraints(const Value& v)
{
  cnt_infos.clear();
  cnt_infos.reserve(v.size());
  for (Json::Value::const_iterator it = v.begin(); it != v.end(); ++it)
  {
    string type;
    childFromJson(*it, type, "type");
    LOG_DEBUG("reading term: %s", type.c_str());
    TermInfoPtr term = TermInfo::fromName(type);

    if (!term)
      PRINT_AND_THROW(boost::format("failed to construct constraint named %s") % type);
    if (!dynamic_cast<MakesConstraint*>(term.get()))
      PRINT_AND_THROW(boost::format("%s is only a cost, but you listed it as a constraint") % type);
    term->term_type = TT_CNT;

    term->fromJson(*this, *it);
    childFromJson(*it, term->name, "name", type);

    cnt_infos.push_back(term);
  }
}

void ProblemConstructionInfo::readInitInfo(const Value& v)
{
  string type_str;
  childFromJson(v, type_str, "type");
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
    const Value& vdata = v["data"];
    if (static_cast<int>(vdata.size()) != n_steps)
    {
      PRINT_AND_THROW("given initialization traj has wrong length");
    }
    init_info.data.resize(n_steps, n_dof);
    for (int i = 0; i < n_steps; ++i)
    {
      DblVec row;
      fromJsonArray(vdata[i], row, n_dof);
      init_info.data.row(i) = toVectorXd(row);
    }
  }
  else if (type_str == "straight_line")
  {
    FAIL_IF_FALSE(v.isMember("endpoint"));
    DblVec endpoint;
    childFromJson(v, endpoint, "endpoint");
    if (static_cast<int>(endpoint.size()) != n_dof)
    {
      PRINT_AND_THROW(boost::format("wrong number of dof values in "
                                    "initialization. expected %i got %j") %
                      n_dof % endpoint.size());
    }
    init_info.data = TrajArray(n_steps, n_dof);
    for (int idof = 0; idof < n_dof; ++idof)
    {
      init_info.data.col(idof) = VectorXd::LinSpaced(n_steps, start_pos[idof], endpoint[idof]);
    }
  }
}

void ProblemConstructionInfo::fromJson(const Value& v)
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

TrajOptResult::TrajOptResult(OptResults& opt, TrajOptProb& prob) : cost_vals(opt.cost_vals), cnt_viols(opt.cnt_viols)
{
  for (const CostPtr& cost : prob.getCosts())
  {
    cost_names.push_back(cost->name());
  }

  for (const ConstraintPtr& cnt : prob.getConstraints())
  {
    cnt_names.push_back(cnt->name());
  }

  traj = getTraj(opt.x, prob.GetVars());
}

TrajOptResultPtr OptimizeProblem(TrajOptProbPtr prob, const tesseract::BasicPlottingPtr plotter)
{
  BasicTrustRegionSQP opt(prob);
  BasicTrustRegionSQPParameters& param = opt.getParameters();
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
      prob->addLinearConstraint(exprSub(AffExpr(prob->m_traj_vars(0, j)), pci.init_info.data(0, j)), EQ);
    }
  }

  if (!bi.dofs_fixed.empty())
  {
    for (const int& dof_ind : bi.dofs_fixed)
    {
      for (int i = 1; i < prob->GetNumSteps(); ++i)
      {
        prob->addLinearConstraint(
            exprSub(AffExpr(prob->m_traj_vars(i, dof_ind)), AffExpr(prob->m_traj_vars(0, dof_ind))), EQ);
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

  vector<double> vlower, vupper;
  vector<string> names;
  for (int i = 0; i < n_steps; ++i)
  {
    vlower.insert(vlower.end(), lower.data(), lower.data() + lower.size());
    vupper.insert(vupper.end(), upper.data(), upper.data() + upper.size());
    for (int j = 0; j < n_dof; ++j)
    {
      names.push_back((boost::format("j_%i_%i") % i % j).str());
    }
  }
  VarVector trajvarvec = createVariables(names, vlower, vupper);
  m_traj_vars = VarArray(n_steps, n_dof, trajvarvec.data());
}

TrajOptProb::TrajOptProb() {}
PoseCostInfo::PoseCostInfo()
{
  pos_coeffs = Vector3d::Ones();
  rot_coeffs = Vector3d::Ones();
  tcp.setIdentity();
}

void PoseCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  Vector3d tcp_xyz = Vector3d::Zero();
  Vector4d tcp_wxyz = Vector4d(1, 0, 0, 0);

  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  childFromJson(params, target, "target");
  childFromJson(params, pos_coeffs, "pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, rot_coeffs, "rot_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, link, "link");
  childFromJson(params, tcp_xyz, "tcp_xyz", (Vector3d)Vector3d::Zero());
  childFromJson(params, tcp_wxyz, "tcp_wxyz", (Vector4d)Vector4d(1, 0, 0, 0));

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

void PoseCostInfo::hatch(TrajOptProb& prob)
{
  VectorOfVectorPtr f(new CartPoseErrCalculator(target, prob.GetKin(), prob.GetEnv(), link, tcp));
  if (term_type == TT_COST)
  {
    prob.addCost(CostPtr(new CostFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), ABS, name)));
  }
  else if (term_type == TT_CNT)
  {
    prob.addConstraint(
        ConstraintPtr(new ConstraintFromFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), EQ, name)));
  }
}

StaticPoseCostInfo::StaticPoseCostInfo()
{
  pos_coeffs = Vector3d::Ones();
  rot_coeffs = Vector3d::Ones();
  tcp.setIdentity();
}

void StaticPoseCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  Vector3d tcp_xyz = Vector3d::Zero();
  Vector4d tcp_wxyz = Vector4d(1, 0, 0, 0);

  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  childFromJson(params, xyz, "xyz");
  childFromJson(params, wxyz, "wxyz");
  childFromJson(params, pos_coeffs, "pos_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, rot_coeffs, "rot_coeffs", (Vector3d)Vector3d::Ones());
  childFromJson(params, link, "link");
  childFromJson(params, tcp_xyz, "tcp_xyz", (Vector3d)Vector3d::Zero());
  childFromJson(params, tcp_wxyz, "tcp_wxyz", (Vector4d)Vector4d(1, 0, 0, 0));

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

void StaticPoseCostInfo::hatch(TrajOptProb& prob)
{
  Eigen::Affine3d input_pose;
  Eigen::Quaterniond q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  input_pose.linear() = q.matrix();
  input_pose.translation() = xyz;

  VectorOfVectorPtr f(new StaticCartPoseErrCalculator(input_pose, prob.GetKin(), prob.GetEnv(), link, tcp));
  if (term_type == TT_COST)
  {
    prob.addCost(CostPtr(new CostFromErrFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), ABS, name)));
  }
  else if (term_type == TT_CNT)
  {
    prob.addConstraint(
        ConstraintPtr(new ConstraintFromFunc(f, prob.GetVarRow(timestep), concat(rot_coeffs, pos_coeffs), EQ, name)));
  }
}

void JointPosCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  int n_steps = pci.basic_info.n_steps;
  const Value& params = v["params"];
  childFromJson(params, vals, "vals");
  childFromJson(params, coeffs, "coeffs");
  if (coeffs.size() == 1)
    coeffs = DblVec(n_steps, coeffs[0]);

  unsigned n_dof = pci.kin->numJoints();
  if (vals.size() != n_dof)
  {
    PRINT_AND_THROW(boost::format("wrong number of dof vals. expected %i got %i") % n_dof % vals.size());
  }
  childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "vals", "coeffs", "timestep" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointPosCostInfo::hatch(TrajOptProb& prob)
{
  prob.addCost(CostPtr(new JointPosCost(prob.GetVarRow(timestep), toVectorXd(vals), toVectorXd(coeffs))));
  prob.getCosts().back()->setName(name);
}

void CartVelCntInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, first_step, "first_step");
  childFromJson(params, last_step, "last_step");
  childFromJson(params, max_displacement, "max_displacement");

  FAIL_IF_FALSE((first_step >= 0) && (first_step <= pci.basic_info.n_steps - 1) && (first_step < last_step));
  FAIL_IF_FALSE((last_step > 0) && (last_step <= pci.basic_info.n_steps - 1));

  childFromJson(params, link, "link");
  const std::vector<std::string>& link_names = pci.kin->getLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  const char* all_fields[] = { "first_step", "last_step", "max_displacement", "link" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void CartVelCntInfo::hatch(TrajOptProb& prob)
{
  for (int iStep = first_step; iStep < last_step; ++iStep)
  {
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(
        VectorOfVectorPtr(new CartVelCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
        MatrixOfVectorPtr(new CartVelJacCalculator(prob.GetKin(), prob.GetEnv(), link, max_displacement)),
        concat(prob.GetVarRow(iStep), prob.GetVarRow(iStep + 1)),
        VectorXd::Ones(0),
        INEQ,
        "CartVel")));
  }
}

void JointVelCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  childFromJson(params, coeffs, "coeffs");
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

void JointVelCostInfo::hatch(TrajOptProb& prob)
{
  prob.addCost(CostPtr(new JointVelCost(prob.GetVars(), toVectorXd(coeffs))));
  prob.getCosts().back()->setName(name);
}

void JointAccCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  childFromJson(params, coeffs, "coeffs");
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

void JointAccCostInfo::hatch(TrajOptProb& prob)
{
  prob.addCost(CostPtr(new JointAccCost(prob.GetVars(), toVectorXd(coeffs))));
  prob.getCosts().back()->setName(name);
}

void JointJerkCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  childFromJson(params, coeffs, "coeffs");
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

void JointJerkCostInfo::hatch(TrajOptProb& prob)
{
  prob.addCost(CostPtr(new JointJerkCost(prob.GetVars(), toVectorXd(coeffs))));
  prob.getCosts().back()->setName(name);
}

void JointVelConstraintInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  int n_steps = pci.basic_info.n_steps;
  unsigned n_dof = pci.kin->numJoints();
  childFromJson(params, vals, "vals");
  childFromJson(params, first_step, "first_step", 0);
  childFromJson(params, last_step, "last_step", n_steps - 1);
  FAIL_IF_FALSE(vals.size() == n_dof);
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));

  const char* all_fields[] = { "vals", "first_step", "last_step" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointVelConstraintInfo::hatch(TrajOptProb& prob)
{
  for (int i = first_step; i <= last_step - 1; ++i)
  {
    for (std::size_t j = 0; j < vals.size(); ++j)
    {
      AffExpr vel = prob.GetVar(i + 1, j) - prob.GetVar(i, j);
      prob.addLinearConstraint(vel - vals[j], INEQ);
      prob.addLinearConstraint(-vel - vals[j], INEQ);
    }
  }
}

void CollisionCostInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];

  int n_steps = pci.basic_info.n_steps;
  childFromJson(params, continuous, "continuous", true);
  childFromJson(params, first_step, "first_step", 0);
  childFromJson(params, last_step, "last_step", n_steps - 1);
  childFromJson(params, gap, "gap", 1);
  FAIL_IF_FALSE(gap >= 0);
  FAIL_IF_FALSE((first_step >= 0) && (first_step < n_steps));
  FAIL_IF_FALSE((last_step >= first_step) && (last_step < n_steps));

  DblVec coeffs, dist_pen;
  childFromJson(params, coeffs, "coeffs");
  int n_terms = last_step - first_step + 1;
  if (coeffs.size() == 1)
    coeffs = DblVec(n_terms, coeffs[0]);
  else if (static_cast<int>(coeffs.size()) != n_terms)
  {
    PRINT_AND_THROW(boost::format("wrong size: coeffs. expected %i got %i") % n_terms % coeffs.size());
  }
  childFromJson(params, dist_pen, "dist_pen");
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
    const Value& pairs = params["pairs"];
    for (Json::Value::const_iterator it = pairs.begin(); it != pairs.end(); ++it)
    {
      FAIL_IF_FALSE(it->isMember("link"));
      std::string link;
      childFromJson(*it, link, "link");

      FAIL_IF_FALSE(it->isMember("pair"));
      std::vector<std::string> pair;
      childFromJson(*it, pair, "pair");

      if (pair.size() == 0)
      {
        PRINT_AND_THROW(boost::format("wrong size: pair. expected > 0 got %i") % pair.size());
      }

      DblVec pair_coeffs;
      childFromJson(*it, pair_coeffs, "coeffs");
      if (pair_coeffs.size() == 1)
      {
        pair_coeffs = DblVec(n_terms, pair_coeffs[0]);
      }
      else if (static_cast<int>(pair_coeffs.size()) != n_terms)
      {
        PRINT_AND_THROW(boost::format("wrong size: coeffs. expected %i got %i") % n_terms % pair_coeffs.size());
      }

      DblVec pair_dist_pen;
      childFromJson(*it, pair_dist_pen, "dist_pen");
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

void CollisionCostInfo::hatch(TrajOptProb& prob)
{
  if (term_type == TT_COST)
  {
    if (continuous)
    {
      for (int i = first_step; i <= last_step - gap; ++i)
      {
        prob.addCost(CostPtr(new CollisionCost(
            prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i), prob.GetVarRow(i + gap))));
        prob.getCosts().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
    else
    {
      for (int i = first_step; i <= last_step; ++i)
      {
        prob.addCost(CostPtr(new CollisionCost(prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i))));
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
        prob.addIneqConstraint(ConstraintPtr(new CollisionConstraint(
            prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i), prob.GetVarRow(i + 1))));
        prob.getIneqConstraints().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
    else
    {
      for (int i = first_step; i <= last_step; ++i)
      {
        prob.addIneqConstraint(ConstraintPtr(
            new CollisionConstraint(prob.GetKin(), prob.GetEnv(), info[i - first_step], prob.GetVarRow(i))));
        prob.getIneqConstraints().back()->setName((boost::format("%s_%i") % name % i).str());
      }
    }
  }
}

void JointConstraintInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  FAIL_IF_FALSE(v.isMember("params"));
  const Value& params = v["params"];
  childFromJson(params, vals, "vals");

  int n_dof = pci.kin->numJoints();
  if (static_cast<int>(vals.size()) != n_dof)
  {
    PRINT_AND_THROW(boost::format("wrong number of dof vals. expected %i got %i") % n_dof % vals.size());
  }
  childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);

  const char* all_fields[] = { "vals", "timestep" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

void JointConstraintInfo::hatch(TrajOptProb& prob)
{
  VarVector vars = prob.GetVarRow(timestep);
  int n_dof = vars.size();
  for (int j = 0; j < n_dof; ++j)
  {
    prob.addLinearConstraint(exprSub(AffExpr(vars[j]), vals[j]), EQ);
  }
}

// initialize with basic default values
AlignedAxisTermInfo::AlignedAxisTermInfo() : tcp_wxyz(1.0, 0.0, 0.0, 0.0)
{
  axis_coeff = 0.0;
  angle_coeff = 0.0;
}

// get the term information from a json value
void AlignedAxisTermInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  // make sure params is a member of the Json value provided
  FAIL_IF_FALSE(v.isMember("params"));

  // get the params value and read the parameters fmro the file into the object members
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  childFromJson(params, wxyz, "wxyz");
  childFromJson(params, tcp_wxyz, "tcp_xwyz", tcp_wxyz);
  childFromJson(params, axis_coeff, "axis_coeff");
  childFromJson(params, angle_coeff, "angle_coeff");
  childFromJson(params, link, "link");
  childFromJson(params, axis, "axis");
  childFromJson(params, tolerance, "tolerance");

  // make sure the link name provided is valid
  const std::vector<std::string>& link_names = pci.kin->getLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  // make sure there are no extra items in the params value
  const char* all_fields[] = { "timestep", "wxyz", "axis_coeff", "angle_coeff", "link", "tolerance", "axis", "tcp_"
                                                                                                             "wxyz" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

// add the term to the problem description
void AlignedAxisTermInfo::hatch(TrajOptProb& prob)
{
  // construct the desired pose
  Eigen::Quaterniond input_q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  Eigen::Quaterniond tcp_q(tcp_wxyz(0), tcp_wxyz(1), tcp_wxyz(2), tcp_wxyz(3));

  // create the equality and inequality error functions to be used
  VectorOfVectorPtr f_ineq(new AlignedAxisErrCalculator(
      input_q.matrix(), prob.GetKin(), prob.GetEnv(), link, axis, tolerance, tcp_q.matrix()));

  Eigen::Vector4d coeffs(axis_coeff, axis_coeff, axis_coeff, angle_coeff);

  // add the costs or constraints depending on the term type
  if (term_type == TT_COST)
  {
    prob.addCost(CostPtr(new CostFromErrFunc(f_ineq, prob.GetVarRow(timestep), coeffs, HINGE, name)));
  }
  else if (term_type == TT_CNT)
  {
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f_ineq, prob.GetVarRow(timestep), coeffs, INEQ, name)));
  }
}

// initialize with basic default values
ConicalAxisTermInfo::ConicalAxisTermInfo() : tcp_wxyz(1.0, 0.0, 0.0, 0.0) { weight = 0.0; }
// construct the term from a Json file
void ConicalAxisTermInfo::fromJson(ProblemConstructionInfo& pci, const Value& v)
{
  // make sure params is a member of the Json value provided
  FAIL_IF_FALSE(v.isMember("params"));

  // get the params value and read the parameters fmro the file into the object members
  const Value& params = v["params"];
  childFromJson(params, timestep, "timestep", pci.basic_info.n_steps - 1);
  childFromJson(params, wxyz, "wxyz");
  childFromJson(params, tcp_wxyz, "tcp_xwyz", tcp_wxyz);
  childFromJson(params, weight, "weight");
  childFromJson(params, link, "link");
  childFromJson(params, axis, "axis");
  childFromJson(params, tolerance, "tolerance");

  // make sure the link name provided is valid
  const std::vector<std::string>& link_names = pci.kin->getLinkNames();
  if (std::find(link_names.begin(), link_names.end(), link) == link_names.end())
  {
    PRINT_AND_THROW(boost::format("invalid link name: %s") % link);
  }

  // make sure there are no extra items in the params value
  const char* all_fields[] = { "timestep", "wxyz", "weight", "link", "tolerance", "axis", "tcp_wxyz" };
  ensure_only_members(params, all_fields, sizeof(all_fields) / sizeof(char*));
}

// add the term to the problem description
void ConicalAxisTermInfo::hatch(TrajOptProb& prob)
{
  // construct the desired pose
  Eigen::Quaterniond input_q(wxyz(0), wxyz(1), wxyz(2), wxyz(3));
  Eigen::Quaterniond tcp_q(tcp_wxyz(0), tcp_wxyz(1), tcp_wxyz(2), tcp_wxyz(3));

  // create the equality and inequality error functions to be used
  VectorOfVectorPtr f_ineq(new ConicalAxisErrCalculator(
      input_q.matrix(), prob.GetKin(), prob.GetEnv(), link, axis, tolerance, tcp_q.matrix()));

  VectorXd coeffs(1);
  coeffs(0) = weight;

  // add the costs or constraints depending on the term type
  if (term_type == TT_COST)
  {
    prob.addCost(CostPtr(new CostFromErrFunc(f_ineq, prob.GetVarRow(timestep), coeffs, HINGE, name)));
  }
  else if (term_type == TT_CNT)
  {
    prob.addConstraint(ConstraintPtr(new ConstraintFromFunc(f_ineq, prob.GetVarRow(timestep), coeffs, INEQ, name)));
  }
}
}
