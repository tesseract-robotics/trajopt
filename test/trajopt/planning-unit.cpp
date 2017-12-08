#include <gtest/gtest.h>
#include <openrave-core.h>
#include <openrave/openrave.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "trajopt/common.hpp"
#include "trajopt/problem_description.hpp"
#include "sco/optimizers.hpp"
#include "trajopt/rave_utils.hpp"
#include "osgviewer/osgviewer.hpp"
#include <ctime>
#include "utils/eigen_conversions.hpp"
#include "utils/clock.hpp"
#include <boost/foreach.hpp>
#include <boost/assign.hpp>
#include "utils/config.hpp"
#include "trajopt_test_utils.hpp"
using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;

namespace {

bool plotting=false, verbose=false;

#if 0
OR::Transform randomReachablePose(RobotAndDOF& rad, KinBody::LinkPtr link) {
  // todo: save & restore
  DblVec dofvals = rad.RandomDOFValues();
  rad.SetDOFValues(dofvals);
  return link->GetTransform();
}
#endif

}


class PlanningTest : public testing::TestWithParam<const char*> {
public:
  static EnvironmentBasePtr env;
  static OSGViewerPtr viewer;

  void SetUp() {
    RAVELOG_DEBUG("SetUp\n");
    RobotBasePtr robot = GetRobot(*env);
    robot->SetDOFValues(DblVec(robot->GetDOF(), 0));
    Transform I; I.identity();
    robot->SetTransform(I);
  }


  static void SetUpTestCase() {
    RAVELOG_DEBUG("SetupTestCase\n");
    RaveInitialize(false, verbose ? Level_Debug : Level_Info);
    env = RaveCreateEnvironment();
    env->StopSimulation();
    env->Load("robots/pr2-beta-static.zae") ;
    env->Load(string(DATA_DIR) + "/table.xml");
    if (plotting) {
      viewer.reset(new OSGViewer(env));
      viewer->UpdateSceneData();
      env->AddViewer(viewer);
    }
  }

  static void TearDownTestCase() {
    RAVELOG_DEBUG("TearDownTestCase\n");
    viewer.reset();
    env.reset();

    RaveDestroy();

  }
};
EnvironmentBasePtr PlanningTest::env;
OSGViewerPtr PlanningTest::viewer;

TEST_F(PlanningTest, numerical_ik1) {
  Json::Value root = readJsonFile(string(DATA_DIR) + "/numerical_ik1.json");
  TrajOptProbPtr prob = ConstructProblem(root, env);
  ASSERT_TRUE(!!prob);

  BasicTrustRegionSQP opt(prob);
//  opt.addCallback(boost::bind(&PlotCosts, boost::ref(prob->getCosts()),*prob->GetRAD(), prob->GetVars(), _1));
  opt.initialize(DblVec(prob->GetNumDOF(), 0));
  double tStart = GetClock();
  opt.optimize();
  RAVELOG_INFO("planning time: %.3f\n", GetClock()-tStart);

}

TEST_F(PlanningTest, arm_around_table) {
  RAVELOG_DEBUG("TEST\n");

  RobotBasePtr pr2 = GetRobot(*env);

  ProblemConstructionInfo pci(env);
  Json::Value root = readJsonFile(string(DATA_DIR) + "/arm_around_table.json");
  pci.fromJson(root);
  pci.rad->SetDOFValues(toDblVec(pci.init_info.data.row(0)));
  TrajOptProbPtr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);


  BasicTrustRegionSQP opt(prob);
  TrajPlotter plotter(env, pci.rad, prob->GetVars());
  if (plotting) {
    plotter.Add(prob->getCosts());
    if (plotting) opt.addCallback(boost::bind(&TrajPlotter::OptimizerCallback, boost::ref(plotter), _1, _2));
    plotter.AddLink(pr2->GetLink("r_gripper_tool_frame"));
  }
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  double tStart = GetClock();
  opt.optimize();
  RAVELOG_INFO("planning time: %.3f\n", GetClock()-tStart);

  vector<Collision> collisions;
  CollisionChecker::GetOrCreate(*env)->ContinuousCheckTrajectory(getTraj(opt.x(), prob->GetVars()), *pci.rad, collisions);
  RAVELOG_INFO("number of continuous collisions: %i\n", collisions.size());
  ASSERT_EQ(collisions.size(), 0);


}


int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  {
    Config config;
    config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
    CommandParser parser(config);
    parser.read(argc, argv);
  }
  srand(0);

   return RUN_ALL_TESTS();
}
