#pragma once
#include <trajopt/common.hpp>
#include <trajopt/json_marshal.hpp>
#include <boost/function.hpp>
#include <boost/shared_ptr.hpp>
#include <trajopt_scene/basic_kin.h>
#include <trajopt_scene/basic_env.h>
#include <trajopt_sco/optimizers.hpp>


namespace sco{struct OptResults;}

namespace trajopt {

using namespace json_marshal;
using namespace Json;

typedef Json::Value TrajOptRequest;
typedef Json::Value TrajOptResponse;
using std::string;

struct TermInfo;
typedef boost::shared_ptr<TermInfo> TermInfoPtr;
class TrajOptProb;
typedef boost::shared_ptr<TrajOptProb> TrajOptProbPtr;
struct ProblemConstructionInfo;
struct TrajOptResult;
typedef boost::shared_ptr<TrajOptResult> TrajOptResultPtr;

TrajOptProbPtr TRAJOPT_API ConstructProblem(const ProblemConstructionInfo&);
TrajOptProbPtr TRAJOPT_API ConstructProblem(const Json::Value&, trajopt_scene::BasicEnvPtr env);
TrajOptResultPtr TRAJOPT_API OptimizeProblem(TrajOptProbPtr, bool plot);

enum TermType {
  TT_COST,
  TT_CNT
};

#define DEFINE_CREATE(classname) \
  static TermInfoPtr create() {\
    TermInfoPtr out(new classname());\
    return out;\
  }


/**
 * Holds all the data for a trajectory optimization problem
 * so you can modify it programmatically, e.g. add your own costs
 */
class TRAJOPT_API TrajOptProb : public OptProb {
public:
  TrajOptProb();
  TrajOptProb(int n_steps,const ProblemConstructionInfo &pci);
  ~TrajOptProb() {}
  VarVector GetVarRow(int i) {
    return m_traj_vars.row(i);
  }
  Var& GetVar(int i, int j) {
    return m_traj_vars.at(i,j);
  }
  VarArray& GetVars() {
    return m_traj_vars;
  }
  int GetNumSteps() {return m_traj_vars.rows();}
  int GetNumDOF() {return m_traj_vars.cols();}
  trajopt_scene::BasicKinConstPtr GetKin() {return m_kin;}
  trajopt_scene::BasicEnvPtr GetEnv() {return m_env;}

  void SetInitTraj(const TrajArray& x) {m_init_traj = x;}
  TrajArray GetInitTraj() {return m_init_traj;}

  friend TrajOptProbPtr ConstructProblem(const ProblemConstructionInfo&);

private:
  VarArray m_traj_vars;
  trajopt_scene::BasicKinConstPtr m_kin;
  trajopt_scene::BasicEnvPtr m_env;
  TrajArray m_init_traj;
};

//void TRAJOPT_API SetupPlotting(TrajOptProb& prob, Optimizer& opt); TODO: Levi Fix

struct TRAJOPT_API TrajOptResult
{
  vector<string> cost_names, cnt_names;
  vector<double> cost_vals, cnt_viols;
  TrajArray traj;
  TrajOptResult(OptResults& opt, TrajOptProb& prob);
};

struct BasicInfo
{
  bool start_fixed;
  int n_steps;
  string manip;
  string robot; // optional
  IntVec dofs_fixed; // optional
};

/**
Initialization info read from json
*/
struct InitInfo {
  enum Type {
    STATIONARY,
    GIVEN_TRAJ,
  };
  Type type;
  TrajArray data;
};


struct TRAJOPT_API MakesCost {
};
struct TRAJOPT_API MakesConstraint {
};

/**
When cost or constraint element of JSON doc is read, one of these guys gets constructed to hold the parameters.
Then it later gets converted to a Cost object by the hatch method
*/
struct TRAJOPT_API TermInfo  {

  string name; // xxx is this used?
  TermType term_type;
  virtual void fromJson(ProblemConstructionInfo &pci, const Json::Value& v)=0;
  virtual void hatch(TrajOptProb& prob) = 0;


  static TermInfoPtr fromName(const string& type);

  /**
   * Registers a user-defined TermInfo so you can use your own cost
   * see function RegisterMakers.cpp
   */
  typedef TermInfoPtr (*MakerFunc)(void);
  static void RegisterMaker(const std::string& type, MakerFunc);

  virtual ~TermInfo() {}
private:
  static std::map<string, MakerFunc> name2maker;
};

/**
This object holds all the data that's read from the JSON document
*/
struct TRAJOPT_API ProblemConstructionInfo {
public:
  BasicInfo basic_info;
  sco::BasicTrustRegionSQPParameters opt_info;
  vector<TermInfoPtr> cost_infos;
  vector<TermInfoPtr> cnt_infos;
  InitInfo init_info;

  trajopt_scene::BasicEnvPtr env;
  trajopt_scene::BasicKinConstPtr kin;

  ProblemConstructionInfo(trajopt_scene::BasicEnvPtr env) : env(env) {}
  void fromJson(const Value& v);

private:
  void readBasicInfo(const Value& v);
  void readOptInfo(const Value& v);
  void readCosts(const Value& v);
  void readConstraints(const Value& v);
  void readInitInfo(const Value& v);
};

/**
 \brief pose error

 See trajopt::PoseTermInfo
 */
struct PoseCostInfo : public TermInfo, public MakesCost, public MakesConstraint {
  int timestep;
  Vector3d xyz;
  Vector4d wxyz;
  Vector3d pos_coeffs, rot_coeffs;

  std::string link;
  Eigen::Affine3d tcp;

  PoseCostInfo();

  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(PoseCostInfo)
};


/**
  \brief Joint space position cost

  \f{align*}{
  \sum_i c_i (x_i - xtarg_i)^2
  \f}
  where \f$i\f$ indexes over dof and \f$c_i\f$ are coeffs
 */
struct JointPosCostInfo : public TermInfo, public MakesCost {
  DblVec vals, coeffs;
  int timestep;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointPosCostInfo)
};



/**
 \brief Motion constraint on link

 Constrains the change in position of the link in each timestep to be less than max_displacement
 */
struct CartVelCntInfo : public TermInfo, public MakesConstraint {
  int first_step, last_step;
  std::string link; //LEVI This may need to be moveit LinkModel
  double max_displacement;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CartVelCntInfo)
};

/**
\brief Joint-space velocity squared

\f{align*}{
  cost = \sum_{t=0}^{T-2} \sum_j c_j (x_{t+1,j} - x_{t,j})^2
\f}
where j indexes over DOF, and \f$c_j\f$ are the coeffs.
*/
struct JointVelCostInfo : public TermInfo, public MakesCost {
  DblVec coeffs;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointVelCostInfo)
};

struct JointVelConstraintInfo : public TermInfo, public MakesConstraint {
  DblVec vals;
  int first_step, last_step;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointVelConstraintInfo)
};

struct JointAccCostInfo : public TermInfo, public MakesCost {
  DblVec coeffs;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointAccCostInfo)
};

struct JointJerkCostInfo : public TermInfo, public MakesCost {
  DblVec coeffs;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointJerkCostInfo)
};

/**
\brief %Collision penalty

Distrete-time penalty:
\f{align*}{
  cost = \sum_{t=0}^{T-1} \sum_{A, B} | distpen_t - sd(A,B) |^+
\f}

Continuous-time penalty: same, except you consider swept-out shaps of robot links. Currently self-collisions are not included.

*/
struct CollisionCostInfo : public TermInfo, public MakesCost {
  /// first_step and last_step are inclusive
  int first_step, last_step;
  /// coeffs.size() = num_timesteps
  DblVec coeffs;
  /// safety margin: contacts with distance < dist_pen are penalized
  DblVec dist_pen;
  bool continuous;
  /// for continuous-time penalty, use swept-shape between timesteps t and t+gap (gap=1 by default)
  int gap;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(CollisionCostInfo)
};


// TODO: unify with joint position constraint
/**
joint-space position constraint
 */
struct JointConstraintInfo : public TermInfo, public MakesConstraint {
  /// joint values. list of length 1 automatically gets expanded to list of length n_dof
  DblVec vals;
  /// which timestep. default = n_timesteps - 1
  int timestep;
  void fromJson(ProblemConstructionInfo &pci, const Value& v);
  void hatch(TrajOptProb& prob);
  DEFINE_CREATE(JointConstraintInfo)
};



}
