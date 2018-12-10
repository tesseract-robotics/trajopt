#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <boost/python.hpp>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt_ros/sco/modeling_utils.hpp>
#include <trajopt_ros/trajopt/collision_checker.hpp>
#include <trajopt_ros/trajopt/numpy_utils.hpp>
#include <trajopt_ros/trajopt/problem_description.hpp>
#include <trajopt_ros/utils/macros.h>
using namespace trajopt;
using namespace Eigen;
using namespace OpenRAVE;
using std::vector;

namespace py = boost::python;

bool gInteractive = false;
py::object openravepy;

EnvironmentBasePtr GetCppEnv(py::object py_env)
{
  py::object openravepy = py::import("openravepy");
  int id = py::extract<int>(openravepy.attr("RaveGetEnvironmentId")(py_env));
  EnvironmentBasePtr cpp_env = RaveGetEnvironment(id);
  return cpp_env;
}
KinBodyPtr GetCppKinBody(py::object py_kb, EnvironmentBasePtr env)
{
  int id = py::extract<int>(py_kb.attr("GetEnvironmentId")());
  return env->GetBodyFromEnvironmentId(id);
}
KinBody::LinkPtr GetCppLink(py::object py_link, EnvironmentBasePtr env)
{
  KinBodyPtr parent = GetCppKinBody(py_link.attr("GetParent")(), env);
  int idx = py::extract<int>(py_link.attr("GetIndex")());
  return parent->GetLinks()[idx];
}

class PyTrajOptProb
{
public:
  TrajOptProbPtr m_prob;
  PyTrajOptProb(TrajOptProbPtr prob) : m_prob(prob) {}
  py::list GetDOFIndices()
  {
    RobotAndDOFPtr rad = boost::dynamic_pointer_cast<RobotAndDOF>(m_prob->GetRAD());
    if (!rad)
      PRINT_AND_THROW("can only call GetDOFIndices on a robot");
    vector<int> inds = rad->GetJointIndices();
    return toPyList(inds);
  }
  void SetRobotActiveDOFs()
  {
    RobotAndDOFPtr rad = boost::dynamic_pointer_cast<RobotAndDOF>(m_prob->GetRAD());
    if (!rad)
      PRINT_AND_THROW("can only call SetRobotActiveDOFs on a robot");
    rad->SetRobotActiveDOFs();
  }
  void AddConstraint1(py::object f, py::list ijs, const string& typestr, const string& name);
  void AddConstraint2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name);
  void AddCost1(py::object f, py::list ijs, const string& name);
  void AddErrCost1(py::object f, py::list ijs, const string& typestr, const string& name);
  void AddErrCost2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name);
};

struct ScalarFuncFromPy : public ScalarOfVector
{
  py::object m_pyfunc;
  ScalarFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
  double operator()(const VectorXd& x) const
  {
    return py::extract<double>(m_pyfunc(toNdarray1<double>(x.data(), x.size())));
  }
};
struct VectorFuncFromPy : public VectorOfVector
{
  py::object m_pyfunc;
  VectorFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
  VectorXd operator()(const VectorXd& x) const
  {
    py::object outarr = np_mod.attr("array")(m_pyfunc(toNdarray1<double>(x.data(), x.size())), "float64");
    VectorXd out = Map<const VectorXd>(getPointer<double>(outarr), py::extract<int>(outarr.attr("size")));
    return out;
  }
};
struct MatrixFuncFromPy : public MatrixOfVector
{
  py::object m_pyfunc;
  MatrixFuncFromPy(py::object pyfunc) : m_pyfunc(pyfunc) {}
  MatrixXd operator()(const VectorXd& x) const
  {
    py::object outarr = np_mod.attr("array")(m_pyfunc(toNdarray1<double>(x.data(), x.size())), "float64");
    py::object shape = outarr.attr("shape");
    MatrixXd out =
        Map<const MatrixXd>(getPointer<double>(outarr), py::extract<int>(shape[0]), py::extract<int>(shape[1]));
    return out;
  }
};

ConstraintType _GetConstraintType(const string& typestr)
{
  if (typestr == "EQ")
    return EQ;
  else if (typestr == "INEQ")
    return INEQ;
  else
    PRINT_AND_THROW("type must be \"EQ\" or \"INEQ\"");
}
PenaltyType _GetPenaltyType(const string& typestr)
{
  if (typestr == "SQUARED")
    return SQUARED;
  else if (typestr == "ABS")
    return ABS;
  else if (typestr == "HINGE")
    return HINGE;
  else
    PRINT_AND_THROW("type must be \"SQUARED\" or \"ABS\" or \"HINGE\"r");
}
VarVector _GetVars(py::list ijs, const VarArray& vars)
{
  VarVector out;
  int n = py::len(ijs);
  for (int k = 0; k < n; ++k)
  {
    int i = py::extract<int>(ijs[k][0]);
    int j = py::extract<int>(ijs[k][1]);
    out.push_back(vars(i, j));
  }
  return out;
}

void PyTrajOptProb::AddConstraint1(py::object f, py::list ijs, const string& typestr, const string& name)
{
  ConstraintType type = _GetConstraintType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  ConstraintPtr c(
      new ConstraintFromFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)), vars, VectorXd::Ones(0), type, name));
  m_prob->addConstraint(c);
}
void PyTrajOptProb::AddConstraint2(py::object f,
                                   py::object dfdx,
                                   py::list ijs,
                                   const string& typestr,
                                   const string& name)
{
  ConstraintType type = _GetConstraintType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  ConstraintPtr c(new ConstraintFromFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)),
                                         MatrixOfVectorPtr(new MatrixFuncFromPy(dfdx)),
                                         vars,
                                         VectorXd::Ones(0),
                                         type,
                                         name));
  m_prob->addConstraint(c);
}
void PyTrajOptProb::AddCost1(py::object f, py::list ijs, const string& name)
{
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  CostPtr c(new CostFromFunc(ScalarOfVectorPtr(new ScalarFuncFromPy(f)), vars, "f"));
  m_prob->addCost(c);
}
void PyTrajOptProb::AddErrCost1(py::object f, py::list ijs, const string& typestr, const string& name)
{
  PenaltyType type = _GetPenaltyType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  CostPtr c(new CostFromErrFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)), vars, VectorXd(), type, name));
  m_prob->addCost(c);
}
void PyTrajOptProb::AddErrCost2(py::object f, py::object dfdx, py::list ijs, const string& typestr, const string& name)
{
  PenaltyType type = _GetPenaltyType(typestr);
  VarVector vars = _GetVars(ijs, m_prob->GetVars());
  CostPtr c(new CostFromErrFunc(VectorOfVectorPtr(new VectorFuncFromPy(f)),
                                MatrixOfVectorPtr(new MatrixFuncFromPy(dfdx)),
                                vars,
                                VectorXd(),
                                type,
                                name));
  m_prob->addCost(c);
}

Json::Value readJsonFile(const std::string& doc)
{
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(doc, root);
  if (!success)
    throw openrave_exception("couldn't parse string as json");
  return root;
}

PyTrajOptProb PyConstructProblem(const std::string& json_string, py::object py_env)
{
  EnvironmentBasePtr cpp_env = GetCppEnv(py_env);
  Json::Value json_root = readJsonFile(json_string);
  TrajOptProbPtr cpp_prob = ConstructProblem(json_root, cpp_env);
  return PyTrajOptProb(cpp_prob);
}

void SetInteractive(py::object b) { gInteractive = py::extract<bool>(b); }
class PyTrajOptResult
{
public:
  PyTrajOptResult(TrajOptResultPtr result) : m_result(result) {}
  TrajOptResultPtr m_result;
  py::object GetCosts()
  {
    py::list out;
    int n_costs = m_result->cost_names.size();
    for (int i = 0; i < n_costs; ++i)
    {
      out.append(py::make_tuple(m_result->cost_names[i], m_result->cost_vals[i]));
    }
    return out;
  }
  py::object GetConstraints()
  {
    py::list out;
    int n_cnts = m_result->cnt_names.size();
    for (int i = 0; i < n_cnts; ++i)
    {
      out.append(py::make_tuple(m_result->cnt_names[i], m_result->cnt_viols[i]));
    }
    return out;
  }
  py::object GetTraj()
  {
    TrajArray& traj = m_result->traj;
    py::object out = np_mod.attr("empty")(py::make_tuple(traj.rows(), traj.cols()));
    for (int i = 0; i < traj.rows(); ++i)
    {
      for (int j = 0; j < traj.cols(); ++j)
      {
        out[i][j] = traj(i, j);
      }
    }
    return out;
  }
  py::object __str__() { return GetCosts().attr("__str__")() + GetConstraints().attr("__str__")(); }
};

PyTrajOptResult PyOptimizeProblem(PyTrajOptProb& prob) { return OptimizeProblem(prob.m_prob, gInteractive); }
class PyCollision
{
public:
  Collision m_c;
  PyCollision(const Collision& c) : m_c(c) {}
  float GetDistance() { return m_c.distance; }
};

py::list toPyList(const vector<Collision>& collisions)
{
  py::list out;
  BOOST_FOREACH (const Collision& c, collisions)
  {
    out.append(PyCollision(c));
  }
  return out;
}

class PyGraphHandle
{
  vector<GraphHandlePtr> m_handles;

public:
  PyGraphHandle(const vector<GraphHandlePtr>& handles) : m_handles(handles) {}
  PyGraphHandle(GraphHandlePtr handle) : m_handles(1, handle) {}
  void SetTransparency1(float alpha)
  {
    BOOST_FOREACH (GraphHandlePtr& handle, m_handles)
    {
      SetTransparency(handle, alpha);
    }
  }
};

class PyCollisionChecker
{
public:
  py::object AllVsAll()
  {
    vector<Collision> collisions;
    m_cc->AllVsAll(collisions);
    return toPyList(collisions);
  }
  py::object BodyVsAll(py::object py_kb)
  {
    KinBodyPtr cpp_kb = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv())
                            ->GetBodyFromEnvironmentId(py::extract<int>(py_kb.attr("GetEnvironmentId")()));
    if (!cpp_kb)
    {
      throw openrave_exception("body isn't part of environment!");
    }
    vector<Collision> collisions;
    m_cc->BodyVsAll(*cpp_kb, collisions);
    return toPyList(collisions);
  }
  PyGraphHandle PlotCollisionGeometry()
  {
    vector<GraphHandlePtr> handles;
    m_cc->PlotCollisionGeometry(handles);
    return PyGraphHandle(handles);
  }
  void ExcludeCollisionPair(py::object link0, py::object link1)
  {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    m_cc->ExcludeCollisionPair(*GetCppLink(link0, env), *GetCppLink(link1, env));
  }
  void IncludeCollisionPair(py::object link0, py::object link1)
  {
    EnvironmentBasePtr env = boost::const_pointer_cast<EnvironmentBase>(m_cc->GetEnv());
    m_cc->IncludeCollisionPair(*GetCppLink(link0, env), *GetCppLink(link1, env));
  }
  PyCollisionChecker(CollisionCheckerPtr cc) : m_cc(cc) {}
private:
  PyCollisionChecker();
  CollisionCheckerPtr m_cc;
};

PyCollisionChecker PyGetCollisionChecker(py::object py_env)
{
  CollisionCheckerPtr cc = CollisionChecker::GetOrCreate(*GetCppEnv(py_env));
  return PyCollisionChecker(cc);
}

class PyOSGViewer
{
public:
  PyOSGViewer(OSGViewerPtr viewer) : m_viewer(viewer) {}
  int Step()
  {
    m_viewer->UpdateSceneData();
    m_viewer->Draw();
    return 0;
  }
  void UpdateSceneData() { m_viewer->UpdateSceneData(); }
  PyGraphHandle PlotKinBody(py::object py_kb)
  {
    return PyGraphHandle(m_viewer->PlotKinBody(GetCppKinBody(py_kb, m_viewer->GetEnv())));
  }
  PyGraphHandle PlotLink(py::object py_link)
  {
    return PyGraphHandle(m_viewer->PlotLink(GetCppLink(py_link, m_viewer->GetEnv())));
  }
  void SetTransparency(py::object py_kb, float alpha)
  {
    m_viewer->SetTransparency(GetCppKinBody(py_kb, m_viewer->GetEnv()), alpha);
  }
  void SetAllTransparency(float a) { m_viewer->SetAllTransparency(a); }
  void Idle()
  {
    assert(!!m_viewer);
    m_viewer->Idle();
  }
  PyGraphHandle DrawText(std::string text, float x, float y, float fontsize, py::object pycolor)
  {
    OpenRAVE::Vector color = OpenRAVE::Vector(py::extract<float>(pycolor[0]),
                                              py::extract<float>(pycolor[1]),
                                              py::extract<float>(pycolor[2]),
                                              py::extract<float>(pycolor[3]));
    return PyGraphHandle(m_viewer->drawtext(text, x, y, fontsize, color));
  }

private:
  OSGViewerPtr m_viewer;
  PyOSGViewer() {}
};
PyOSGViewer PyGetViewer(py::object py_env)
{
  EnvironmentBasePtr env = GetCppEnv(py_env);
  OSGViewerPtr viewer = OSGViewer::GetOrCreate(env);
  ALWAYS_ASSERT(!!viewer);
  return PyOSGViewer(viewer);
}

BOOST_PYTHON_MODULE(ctrajoptpy)
{
  np_mod = py::import("numpy");

  py::object openravepy = py::import("openravepy");

  string pyversion = py::extract<string>(openravepy.attr("__version__"));
  if (OPENRAVE_VERSION_STRING != pyversion)
  {
    PRINT_AND_THROW("the openrave on your pythonpath is different from the "
                    "openrave version that trajopt links to!");
  }

  py::class_<PyTrajOptProb>("TrajOptProb", py::no_init)
      .def("GetDOFIndices", &PyTrajOptProb::GetDOFIndices)
      .def("SetRobotActiveDOFs",
           &PyTrajOptProb::SetRobotActiveDOFs,
           "Sets the active DOFs of the robot to the DOFs in the optimization "
           "problem")
      .def("AddConstraint",
           &PyTrajOptProb::AddConstraint1,
           "Add constraint from python function (using numerical "
           "differentiation)",
           (py::arg("f"), "var_ijs", "constraint_type", "name"))
      .def("AddConstraint",
           &PyTrajOptProb::AddConstraint2,
           "Add constraint from python error function and analytic derivative",
           (py::arg("f"), "dfdx", "var_ijs", "constraint_type", "name"))
      .def("AddCost",
           &PyTrajOptProb::AddCost1,
           "Add cost from python "
           "scalar-valued function (using "
           "numerical differentiation)",
           (py::arg("func"), "var_ijs", "name"))
      .def("AddErrorCost",
           &PyTrajOptProb::AddErrCost1,
           "Add error cost from python vector-valued error function (using "
           "numerical differentiation)",
           (py::arg("f"), "var_ijs", "penalty_type", "name"))
      .def("AddErrorCost",
           &PyTrajOptProb::AddErrCost2,
           "Add error cost from python vector-valued error function and "
           "analytic derivative",
           (py::arg("f"), "dfdx", "var_ijs", "penalty_type", "name"));
  py::def("SetInteractive", &SetInteractive, "if True, pause and plot every iteration");
  py::def("ConstructProblem", &PyConstructProblem, "create problem from JSON string");
  py::def("OptimizeProblem", &PyOptimizeProblem);

  py::class_<PyTrajOptResult>("TrajOptResult", py::no_init)
      .def("GetCosts", &PyTrajOptResult::GetCosts)
      .def("GetConstraints", &PyTrajOptResult::GetConstraints)
      .def("GetTraj", &PyTrajOptResult::GetTraj)
      .def("__str__", &PyTrajOptResult::__str__);

  py::class_<PyCollisionChecker>("CollisionChecker", py::no_init)
      .def("AllVsAll", &PyCollisionChecker::AllVsAll)
      .def("BodyVsAll", &PyCollisionChecker::BodyVsAll)
      .def("PlotCollisionGeometry", &PyCollisionChecker::PlotCollisionGeometry)
      .def("ExcludeCollisionPair", &PyCollisionChecker::ExcludeCollisionPair)
      .def("IncludeCollisionPair", &PyCollisionChecker::IncludeCollisionPair);
  py::def("GetCollisionChecker", &PyGetCollisionChecker);
  py::class_<PyCollision>("Collision", py::no_init).def("GetDistance", &PyCollision::GetDistance);
  py::class_<PyGraphHandle>("GraphHandle", py::no_init).def("SetTransparency", &PyGraphHandle::SetTransparency1);

  py::class_<PyOSGViewer>("OSGViewer", py::no_init)
      .def("UpdateSceneData", &PyOSGViewer::UpdateSceneData)
      .def("Step", &PyOSGViewer::Step)
      .def("PlotKinBody", &PyOSGViewer::PlotKinBody)
      .def("PlotLink", &PyOSGViewer::PlotLink)
      .def("SetTransparency", &PyOSGViewer::SetTransparency)
      .def("SetAllTransparency", &PyOSGViewer::SetAllTransparency)
      .def("Idle", &PyOSGViewer::Idle)
      .def("DrawText", &PyOSGViewer::DrawText);
  py::def("GetViewer", &PyGetViewer, "Get OSG viewer for environment or create a new one");
}
