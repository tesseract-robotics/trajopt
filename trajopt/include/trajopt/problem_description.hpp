#pragma once
#include <tesseract_core/basic_env.h>
#include <tesseract_core/basic_kin.h>
#include <trajopt/common.hpp>
#include <trajopt/json_marshal.hpp>
#include <trajopt_sco/optimizers.hpp>

namespace sco
{
struct OptResults;
}

namespace trajopt
{

typedef Json::Value TrajOptRequest;
typedef Json::Value TrajOptResponse;

struct TermInfo;
typedef std::shared_ptr<TermInfo> TermInfoPtr;
class TrajOptProb;
typedef std::shared_ptr<TrajOptProb> TrajOptProbPtr;
struct ProblemConstructionInfo;
struct TrajOptResult;
typedef std::shared_ptr<TrajOptResult> TrajOptResultPtr;

TrajOptProbPtr TRAJOPT_API ConstructProblem(const ProblemConstructionInfo&);
TrajOptProbPtr TRAJOPT_API ConstructProblem(const Json::Value&, tesseract::BasicEnvConstPtr env);
TrajOptResultPtr TRAJOPT_API OptimizeProblem(TrajOptProbPtr, const tesseract::BasicPlottingPtr plotter = nullptr);

enum TermType
{
  TT_COST,
  TT_CNT
};

#define DEFINE_CREATE(classname)                                                                                       \
  static TermInfoPtr create()                                                                                          \
  {                                                                                                                    \
    TermInfoPtr out(new classname());                                                                                  \
    return out;                                                                                                        \
  }

/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TRAJOPT_API TrajOptProb : public sco::OptProb
{
public:
  TrajOptProb();
  TrajOptProb(int n_steps, const ProblemConstructionInfo& pci);
  ~TrajOptProb() {}
  sco::VarVector GetVarRow(int i) { return m_traj_vars.row(i); }
  sco::Var& GetVar(int i, int j) { return m_traj_vars.at(i, j); }
  VarArray& GetVars() { return m_traj_vars; }
  int GetNumSteps() { return m_traj_vars.rows(); }
  int GetNumDOF() { return m_traj_vars.cols(); }
  tesseract::BasicKinConstPtr GetKin() { return m_kin; }
  tesseract::BasicEnvConstPtr GetEnv() { return m_env; }
  void SetInitTraj(const TrajArray& x) { m_init_traj = x; }
  TrajArray GetInitTraj() { return m_init_traj; }
  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);

private:
  VarArray m_traj_vars;
  tesseract::BasicKinConstPtr m_kin;
  tesseract::BasicEnvConstPtr m_env;
  TrajArray m_init_traj;
};

// void TRAJOPT_API SetupPlotting(TrajOptProb& prob, Optimizer& opt); TODO: Levi
// Fix

struct TRAJOPT_API TrajOptResult
{
  std::vector<std::string> cost_names, cnt_names;
  DblVec cost_vals, cnt_viols;
  TrajArray traj;
  TrajOptResult(sco::OptResults& opt, TrajOptProb& prob);
};

struct BasicInfo
{
  bool start_fixed;
  int n_steps;
  std::string manip;
  std::string robot;       // optional
  IntVec dofs_fixed;  // optional
};

/**
Initialization info read from json
*/
struct InitInfo
{
  enum Type
  {
    STATIONARY,
    GIVEN_TRAJ,
  };
  Type type;
  TrajArray data;
};

struct TRAJOPT_API MakesCost
{
};
struct TRAJOPT_API MakesConstraint
{
};

/**
When cost or constraint element of JSON doc is read, one of these guys gets
constructed to hold the parameters.
Then it later gets converted to a Cost object by the hatch method
*/
struct TRAJOPT_API TermInfo
{
  std::string name;  // xxx is this used?
  TermType term_type;
  virtual void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) = 0;
  virtual void hatch(TrajOptProb& prob) = 0;

  static TermInfoPtr fromName(const std::string& type);

  /**
   * Registers a user-defined TermInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  typedef TermInfoPtr (*MakerFunc)(void);
  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~TermInfo() {}

private:
  static std::map<std::string, MakerFunc> name2maker;
};

/**
This object holds all the data that's read from the JSON document
*/
struct TRAJOPT_API ProblemConstructionInfo
{
public:
  BasicInfo basic_info;
  sco::BasicTrustRegionSQPParameters opt_info;
  std::vector<TermInfoPtr> cost_infos;
  std::vector<TermInfoPtr> cnt_infos;
  InitInfo init_info;

  tesseract::BasicEnvConstPtr env;
  tesseract::BasicKinConstPtr kin;

  ProblemConstructionInfo(tesseract::BasicEnvConstPtr env) : env(env) {}
  void fromJson(const Json::Value& v);

private:
  void readBasicInfo(const Json::Value& v);
  void readOptInfo(const Json::Value& v);
  void readCosts(const Json::Value& v);
  void readConstraints(const Json::Value& v);
  void readInitInfo(const Json::Value& v);
};

/** @brief This is used when the goal frame is not fixed in space */
struct DynamicCartPoseTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Timestep at which to apply term */
  int timestep;
  std::string target;
  /** @brief Coefficients for position and rotation */
  Eigen::Vector3d pos_coeffs, rot_coeffs;
  /** @brief Link which should reach desired pos */
  std::string link;
  /** @brief Static transform applied to the link */
  Eigen::Isometry3d tcp;

  DynamicCartPoseTermInfo();

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(DynamicCartPoseTermInfo)
};

/** @brief This term is used when the goal frame is fixed in cartesian space

  Set term_type == TT_COST or TT_CNT for cost or constraint.
*/
struct CartPoseTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Timestep at which to apply term */
  int timestep;
  /** @brief  Cartesian position */
  Eigen::Vector3d xyz;
  /** @brief Rotation quaternion */
  Eigen::Vector4d wxyz;
  /** @brief coefficients for position and rotation */
  Eigen::Vector3d pos_coeffs, rot_coeffs;
  /** @brief Link which should reach desired pose */
  std::string link;
  /** @brief Static transform applied to the link */
  Eigen::Isometry3d tcp;

  CartPoseTermInfo();

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CartPoseTermInfo)
};

/**
 \brief Applies cost/constraint to the cartesian velocity of a link

 Constrains the change in position of the link in each timestep to be less than
 max_displacement
 */
struct CartVelTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  /** @brief Timesteps over which to apply term */
  int first_step, last_step;
  /** @brief Link to which the term is applied */
  std::string link;  // LEVI This may need to be moveit LinkModel
  double max_displacement;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CartVelTermInfo)
};

/**
  \brief Joint space position cost
    Position operates on a single point (unlike velocity, etc). This is b/c the primary usecase is joint-space
    position waypoints

  \f{align*}{
  \sum_i c_i (x_i - xtarg_i)^2
  \f}
  where \f$i\f$ indexes over dof and \f$c_i\f$ are coeffs
 */
struct JointPosTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  /** @brief TT_COST: Target joint value. TT_CNT: Joint Limit */
  DblVec vals;
  /** @brief Coefficent that scales the cost. */
  DblVec coeffs;
  /** @brief Time step to which term is applied */
  int timestep;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointPosTermInfo)
};

/**
\brief Used to apply cost/constraint to joint-space velocity

Term is applied to every step between first_step and last_step (TODO). It applies two limits, upper_limits/lower_limits,
to the joint velocity subject to the following cases.

* term_type = TT_COST
** upper_limit=lower_limit - Cost is applied with a SQUARED error scaled by coeffs
** upper_limit!=lower_limit - Cost is applied with a hinge error scaled by coeffs

* term_type = TT_CNT
** upper_limit=lower_limit - Equality constraint is applied
** upper_limit!=lower_limit - 2 Inequality constraints are applied. That velocity < upper_limit and velocity >
lower_limit

Note: targs, upper_limits, and lower_limits are optional. If a term is not given it will default to 0 for all joints. If
one value is given, this will be broadcast to all joints.

\f{align*}{
  cost = \sum_{t=0}^{T-2} \sum_j c_j (x_{t+1,j} - x_{t,j})^2
\f}
where j indexes over DOF, and \f$c_j\f$ are the coeffs.
*/
struct JointVelTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  /** @brief Vector of coefficients that scales cost. */
  DblVec coeffs;
  /** @brief Vector of velocity targets. Default: 0 */
  DblVec targs;
  /** @brief Vector of velocity upper limits. Default: 0 */
  DblVec upper_tols;
  /** @brief Vector of velocity lower limits. Default: 0 */
  DblVec lower_tols;
  /** @brief First time step to which the term is applied (TODO)*/
  int first_step;
  /** @brief Last time step to which the term is applied (TODO) */
  int last_step;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointVelTermInfo)
};

/**
 * @brief  Used to apply cost/constraint to joint-space acceleration
 */
struct JointAccTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  /** @brief For TT_COST: coefficient that scales cost. For TT_CNT: Acceleration limit*/
  DblVec coeffs;
  /** @brief Vector of accel targets. Default: 0 */
  DblVec targs;
  /** @brief Vector of accel upper limits. Default: 0 */
  DblVec upper_tols;
  /** @brief Vector of accel lower limits. Default: 0 */
  DblVec lower_tols;
  /** @brief First time step to which the term is applied */
  int first_step;
  /** @brief Last time step to which the term is applied */
  int last_step;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointAccTermInfo)
};

/**
 * @brief Used to apply cost/constraint to joint-space jerk
 */
struct JointJerkTermInfo : public TermInfo, public MakesCost, public MakesConstraint
{
  /** @brief For TT_COST: coefficient that scales cost. For TT_CNT: Jerk limit */
  DblVec coeffs;
  /** @brief Vector of jerk targets. Default: 0 */
  DblVec targs;
  /** @brief Vector of jerk upper limits. Default: 0 */
  DblVec upper_tols;
  /** @brief Vector of jerk lower limits. Default: 0 */
  DblVec lower_tols;
  /** @brief First time step to which the term is applied */
  int first_step;
  /** @brief Last time step to which the term is applied */
  int last_step;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointJerkTermInfo)
};

/**
\brief %Collision penalty

Distrete-time penalty:
\f{align*}{
  cost = \sum_{t=0}^{T-1} \sum_{A, B} | distpen_t - sd(A,B) |^+
\f}

Continuous-time penalty: same, except you consider swept-out shaps of robot
links. Currently self-collisions are not included.

*/
struct CollisionTermInfo : public TermInfo, public MakesCost
{
  /** @brief first_step and last_step are inclusive */
  int first_step, last_step;

  /** @brief Indicate if continuous collision checking should be used. */
  bool continuous;

  /** @brief for continuous-time penalty, use swept-shape between timesteps t and t+gap */
  /** @brief (gap=1 by default) */
  int gap;

  /** @brief Contains distance penalization data: Safety Margin, Coeff used during */
  /** @brief optimization, etc. */
  std::vector<SafetyMarginDataPtr> info;

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v);
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CollisionTermInfo)
};

}  // namespace trajopt
