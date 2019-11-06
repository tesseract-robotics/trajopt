#pragma once
#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <unordered_map>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract/tesseract.h>
#include <trajopt/common.hpp>
#include <trajopt/json_marshal.hpp>
#include <trajopt_sco/optimizers.hpp>

namespace sco
{
struct OptResults;
}

namespace trajopt
{
using TrajOptRequest = Json::Value;
using TrajOptResponse = Json::Value;

struct ProblemConstructionInfo;

enum TermType
{
  TT_COST = 0x1,      // 0000 0001
  TT_CNT = 0x2,       // 0000 0010
  TT_USE_TIME = 0x4,  // 0000 0100
};

#define DEFINE_CREATE(classname)                                                                                       \
  static TermInfo::Ptr create()                                                                                        \
  {                                                                                                                    \
    TermInfo::Ptr out(new classname());                                                                                \
    return out;                                                                                                        \
  }

/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TRAJOPT_API TrajOptProb : public sco::OptProb
{
public:
  using Ptr = std::shared_ptr<TrajOptProb>;

  TrajOptProb();
  TrajOptProb(int n_steps, const ProblemConstructionInfo& pci);
  virtual ~TrajOptProb() = default;
  sco::VarVector GetVarRow(int i, int start_col, int num_col) { return m_traj_vars.rblock(i, start_col, num_col); }
  sco::VarVector GetVarRow(int i) { return m_traj_vars.row(i); }
  sco::Var& GetVar(int i, int j) { return m_traj_vars.at(i, j); }
  VarArray& GetVars() { return m_traj_vars; }
  /** @brief Returns the number of steps in the problem. This is the number of rows in the optimization matrix.*/
  int GetNumSteps() { return m_traj_vars.rows(); }
  /** @brief Returns the problem DOF. This is the number of columns in the optization matrix.
   * Note that this is not necessarily the same as the kinematic DOF.*/
  int GetNumDOF() { return m_traj_vars.cols(); }
  tesseract_kinematics::ForwardKinematics::ConstPtr GetKin() { return m_kin; }
  tesseract_environment::Environment::ConstPtr GetEnv() { return m_env; }
  void SetInitTraj(const TrajArray& x) { m_init_traj = x; }
  TrajArray GetInitTraj() { return m_init_traj; }
  friend TrajOptProb::Ptr ConstructProblem(const ProblemConstructionInfo&);
  /** @brief Returns TrajOptProb.has_time */
  bool GetHasTime() { return has_time; }
  /** @brief Sets TrajOptProb.has_time  */
  void SetHasTime(bool tmp) { has_time = tmp; }

private:
  /** @brief If true, the last column in the optimization matrix will be 1/dt */
  bool has_time;
  VarArray m_traj_vars;
  tesseract_kinematics::ForwardKinematics::ConstPtr m_kin;
  tesseract_environment::Environment::ConstPtr m_env;
  TrajArray m_init_traj;
};

// void TRAJOPT_API SetupPlotting(TrajOptProb& prob, Optimizer& opt); TODO: Levi
// Fix

struct TRAJOPT_API TrajOptResult
{
  using Ptr = std::shared_ptr<TrajOptResult>;

  std::vector<std::string> cost_names, cnt_names;
  DblVec cost_vals, cnt_viols;
  TrajArray traj;
  sco::OptStatus status;
  TrajOptResult(sco::OptResults& opt, TrajOptProb& prob);
};

struct BasicInfo
{
  /** @brief If true first time step is fixed with a joint level constraint*/
  bool start_fixed;
  /** @brief Number of time steps (rows) in the optimization matrix */
  int n_steps;
  std::string manip;
  std::string robot;             // optional
  IntVec dofs_fixed;             // optional
  sco::ModelType convex_solver;  // which convex solver to use

  /** @brief If true, the last column in the optimization matrix will be 1/dt */
  bool use_time = false;
  /** @brief The upper limit of 1/dt values allowed in the optimization*/
  double dt_upper_lim = 1.0;
  /** @brief The lower limit of 1/dt values allowed in the optimization*/
  double dt_lower_lim = 1.0;
};

/**
Initialization info read from json
*/
struct InitInfo
{
  /** @brief Methods of initializing the optimization matrix

    STATIONARY: Initializes all joint values to the initial value (the current value in the env
    pci.env->getCurrentJointValues)
    JOINT_INTERPOLATED: Linearly interpolates between initial value and the joint position specified in InitInfo.data
    GIVEN_TRAJ: Initializes the matrix to a given trajectory

    In all cases the dt column (if present) is appended the selected method is defined.
 */
  enum Type
  {
    STATIONARY,
    JOINT_INTERPOLATED,
    GIVEN_TRAJ,
  };
  /** @brief Specifies the type of initialization to use */
  Type type;
  /** @brief Data used during initialization. Use depends on the initialization selected. */
  TrajArray data;
  /** @brief Default value the final column of the optimization is initialized too if time is being used */
  double dt = 1.0;
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
  using Ptr = std::shared_ptr<TermInfo>;

  std::string name;
  int term_type;
  int getSupportedTypes() { return supported_term_types_; }
  virtual void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) = 0;
  virtual void hatch(TrajOptProb& prob) = 0;

  static TermInfo::Ptr fromName(const std::string& type);

  /**
   * Registers a user-defined TermInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  typedef std::shared_ptr<TermInfo> (*MakerFunc)(void);
  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~TermInfo() = default;

protected:
  TermInfo(int supported_term_types) : supported_term_types_(supported_term_types) {}

private:
  static std::map<std::string, MakerFunc> name2maker;
  int supported_term_types_;
};

/**
This object holds all the data that's read from the JSON document
*/
struct TRAJOPT_API ProblemConstructionInfo
{
public:
  BasicInfo basic_info;
  sco::BasicTrustRegionSQPParameters opt_info;
  std::vector<TermInfo::Ptr> cost_infos;
  std::vector<TermInfo::Ptr> cnt_infos;
  InitInfo init_info;

  tesseract_environment::Environment::ConstPtr env;
  tesseract_kinematics::ForwardKinematics::ConstPtr kin;

  ProblemConstructionInfo(tesseract::Tesseract::ConstPtr tesseract)
    : env(tesseract->getEnvironmentConst()), tesseract_(tesseract)
  {
  }

  tesseract_kinematics::ForwardKinematics::ConstPtr getManipulator(const std::string& name) const
  {
    return tesseract_->getFwdKinematicsManagerConst()->getFwdKinematicSolver(name);
  }

  void fromJson(const Json::Value& v);

private:
  void readBasicInfo(const Json::Value& v);
  void readOptInfo(const Json::Value& v);
  void readCosts(const Json::Value& v);
  void readConstraints(const Json::Value& v);
  void readInitInfo(const Json::Value& v);

  tesseract::Tesseract::ConstPtr tesseract_;
};

/** @brief User defined error function that is set as a constraint or cost for each timestep.
 *
 * The error function is required, but the jacobian is optional (nullptr).
 *
 * Error Function:
 *   arg: VectorXd will be all of the joint values for one timestep.
 *   return: VectorXd of violations for each joint. Anything != 0 will be a violation
 *
 * Error Function Jacobian:
 *   arg: VectorXd will be all of the joint values for one timestep.
 *   return: Eigen::MatrixXd that represents the change in the error function with respect to joint values
 */
struct UserDefinedTermInfo : public TermInfo
{
  /** @brief The name prefix shown in output */
  std::string name = "UserDefined";

  /** @brief Timesteps over which to apply term */
  int first_step, last_step;

  /** @brief The user defined error function */
  sco::VectorOfVector::func error_function;

  /** @brief The user defined jocobian function */
  sco::MatrixOfVector::func jacobian_function;

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(UserDefinedTermInfo)

  /** @brief Initialize term with it's supported types */
  UserDefinedTermInfo() : TermInfo(TT_COST | TT_CNT) {}
};

/** @brief This is used when the goal frame is not fixed in space */
struct DynamicCartPoseTermInfo : public TermInfo
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /** @brief Timestep at which to apply term */
  int timestep;
  /** @brief The link relative to which the target_tcp is defined */
  std::string target;
  /** @brief Coefficients for position and rotation */
  Eigen::Vector3d pos_coeffs, rot_coeffs;
  /** @brief Link which should reach desired pos */
  std::string link;
  /** @brief Static transform applied to the link_ location */
  Eigen::Isometry3d tcp;
  /** @brief A Static tranform to be applied to target_ location */
  Eigen::Isometry3d target_tcp;

  DynamicCartPoseTermInfo();

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(DynamicCartPoseTermInfo)
};

/** @brief This term is used when the goal frame is fixed in cartesian space

  Set term_type == TT_COST or TT_CNT for cost or constraint.
*/
struct CartPoseTermInfo : public TermInfo
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
  /** @brief The frame relative to which the target position is defined. If empty, frame is assumed to the root,
   * "world", frame */
  std::string target;

  CartPoseTermInfo();

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CartPoseTermInfo)
};

/**
 \brief Applies cost/constraint to the cartesian velocity of a link

 Constrains the change in position of the link in each timestep to be less than
 max_displacement
 */
struct CartVelTermInfo : public TermInfo
{
  /** @brief Timesteps over which to apply term */
  int first_step, last_step;
  /** @brief Link to which the term is applied */
  std::string link;
  double max_displacement;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CartVelTermInfo)

  /** @brief Initialize term with it's supported types */
  CartVelTermInfo() : TermInfo(TT_COST | TT_CNT) {}
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
struct JointPosTermInfo : public TermInfo
{
  /** @brief Vector of coefficients that scale the cost. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec coeffs;
  /** @brief Vector of position targets. This is a required value. Size should be the DOF of the system */
  DblVec targets;
  /** @brief Vector of position upper limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec upper_tols;
  /** @brief Vector of position lower limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec lower_tols;
  /** @brief First time step to which the term is applied. Default: 0 */
  int first_step = 0;
  /** @brief Last time step to which the term is applied. Default: prob.GetNumSteps() - 1*/
  int last_step = -1;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(JointPosTermInfo)

  /** @brief Initialize term with it's supported types */
  JointPosTermInfo() : TermInfo(TT_COST | TT_CNT | TT_USE_TIME) {}
};

/**
@brief Used to apply cost/constraint to joint-space velocity

Term is applied to every step between first_step and last_step. It applies two limits, upper_limits/lower_limits,
to the joint velocity subject to the following cases.

* term_type = TT_COST
** upper_limit = lower_limit = 0 - Cost is applied with a SQUARED error scaled for each joint by coeffs
** upper_limit != lower_limit - 2 hinge costs are applied scaled for each joint by coeffs. If velocity < upper_limit and
velocity > lower_limit, no penalty.

* term_type = TT_CNT
** upper_limit = lower_limit = 0 - Equality constraint is applied
** upper_limit != lower_limit - 2 Inequality constraints are applied. These are both satisfied when velocity <
upper_limit and velocity > lower_limit

Note: coeffs, upper_limits, and lower_limits are optional. If a term is not given it will default to 0 for all joints.
If one value is given, this will be broadcast to all joints.

Note: Velocity is calculated numerically using forward finite difference

\f{align*}{
  cost = \sum_{t=0}^{T-2} \sum_j c_j (x_{t+1,j} - x_{t,j})^2
\f}
where j indexes over DOF, and \f$c_j\f$ are the coeffs.
*/
struct JointVelTermInfo : public TermInfo
{
  /** @brief Vector of coefficients that scale the cost. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec coeffs;
  /** @brief Vector of velocity targets. This is a required value. Size should be the DOF of the system */
  DblVec targets;
  /** @brief Vector of velocity upper limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec upper_tols;
  /** @brief Vector of velocity lower limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec lower_tols;
  /** @brief First time step to which the term is applied. Default: 0*/
  int first_step = 0;
  /** @brief Last time step to which the term is applied. Default: prob.GetNumSteps() - 1*/
  int last_step = -1;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(JointVelTermInfo)

  /** @brief Initialize term with it's supported types */
  JointVelTermInfo() : TermInfo(TT_COST | TT_CNT | TT_USE_TIME) {}
};

/**
@brief Used to apply cost/constraint to joint-space acceleration

Term is applied to every step between first_step and last_step. It applies two limits, upper_limits/lower_limits,
to the joint velocity subject to the following cases.

* term_type = TT_COST
** upper_limit = lower_limit = 0 - Cost is applied with a SQUARED error scaled for each joint by coeffs
** upper_limit != lower_limit - 2 hinge costs are applied scaled for each joint by coeffs. If acceleration < upper_limit
and acceleration > lower_limit, no penalty.

* term_type = TT_CNT
** upper_limit = lower_limit = 0 - Equality constraint is applied
** upper_limit != lower_limit - 2 Inequality constraints are applied. These are both satisfied when acceleration <
upper_limit and acceleration > lower_limit

Note: coeffs, upper_limits, and lower_limits are optional. If a term is not given it will default to 0 for all joints.
If one value is given, this will be broadcast to all joints.

Note: Acceleration is calculated numerically using central finite difference
*/
struct JointAccTermInfo : public TermInfo
{
  /** @brief Vector of coefficients that scale the cost. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec coeffs;
  /** @brief Vector of accel targets. This is a required value. Size should be the DOF of the system */
  DblVec targets;
  /** @brief Vector of accel upper limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec upper_tols;
  /** @brief Vector of accel lower limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec lower_tols;
  /** @brief First time step to which the term is applied. Default: 0 */
  int first_step = 0;
  /** @brief Last time step to which the term is applied. Default: prob.GetNumSteps() - 1 */
  int last_step = -1;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(JointAccTermInfo)

  /** @brief Initialize term with it's supported types */
  JointAccTermInfo() : TermInfo(TT_COST | TT_CNT) {}
};

/**
@brief Used to apply cost/constraint to joint-space jerk

Term is applied to every step between first_step and last_step. It applies two limits, upper_limits/lower_limits,
to the joint velocity subject to the following cases.

* term_type = TT_COST
** upper_limit = lower_limit = 0 - Cost is applied with a SQUARED error scaled for each joint by coeffs
** upper_limit != lower_limit - 2 hinge costs are applied scaled for each joint by coeffs. If jerk < upper_limit and
jerk > lower_limit, no penalty.

* term_type = TT_CNT
** upper_limit = lower_limit = 0 - Equality constraint is applied
** upper_limit != lower_limit - 2 Inequality constraints are applied. These are both satisfied when jerk <
upper_limit and jerk > lower_limit

Note: coeffs, upper_limits, and lower_limits are optional. If a term is not given it will default to 0 for all joints.
If one value is given, this will be broadcast to all joints.

Note: Jerk is calculated numerically using central finite difference
*/
struct JointJerkTermInfo : public TermInfo
{
  /** @brief Vector of coefficients that scale the cost. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec coeffs;
  /** @brief Vector of jerk targets. This is a required value. Size should be the DOF of the system */
  DblVec targets;
  /** @brief Vector of jerk upper limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec upper_tols;
  /** @brief Vector of jerk lower limits. Size should be the DOF of the system. Default: vector of 0's*/
  DblVec lower_tols;
  /** @brief First time step to which the term is applied. Default: 0 */
  int first_step = 0;
  /** @brief Last time step to which the term is applied. Default: prob.GetNumSteps() - 1 */
  int last_step = -1;
  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(JointJerkTermInfo)

  /** @brief Initialize term with it's supported types */
  JointJerkTermInfo() : TermInfo(TT_COST | TT_CNT) {}
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
struct CollisionTermInfo : public TermInfo
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
  std::vector<SafetyMarginData::Ptr> info;

  /** @brief Used to add term to pci from json */
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  /** @brief Converts term info into cost/constraint and adds it to trajopt problem */
  void hatch(TrajOptProb& prob) override;
  DEFINE_CREATE(CollisionTermInfo)

  CollisionTermInfo() : TermInfo(TT_COST | TT_CNT) {}
};

/**
 * @brief Applies a penalty to the total time taken by the trajectory
 */
struct TotalTimeTermInfo : public TermInfo
{
  /** @brief Scales this term. */
  double coeff = 1.0;
  /** @brief If non zero, penalty type will be a hinge on values greater than this limit relative to the target. */
  double limit = 0.0;

  void hatch(TrajOptProb& prob) override;
  void fromJson(ProblemConstructionInfo& pci, const Json::Value& v) override;
  DEFINE_CREATE(TotalTimeTermInfo)

  TotalTimeTermInfo() : TermInfo(TT_COST | TT_CNT | TT_USE_TIME) {}
};

TrajOptProb::Ptr TRAJOPT_API ConstructProblem(const ProblemConstructionInfo&);
TrajOptProb::Ptr TRAJOPT_API ConstructProblem(const Json::Value&, const tesseract::Tesseract::ConstPtr& tesseract);
TrajOptResult::Ptr TRAJOPT_API OptimizeProblem(TrajOptProb::Ptr,
                                               const tesseract_visualization::Visualization::Ptr& plotter = nullptr);

}  // namespace trajopt
