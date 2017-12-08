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
#include "trajopt/plot_callback.hpp"
#include "trajopt_test_utils.hpp"
#include "trajopt/collision_terms.hpp"
using namespace trajopt;
using namespace std;
using namespace OpenRAVE;
using namespace util;
using namespace boost::assign;

string data_dir() {
  string out = DATA_DIR;
  return out;
}

bool plotting=false, verbose=false;

#ifdef __CDT_PARSER__
#define TEST(a,b) void asdf()
#endif

TEST(cast, boxes) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  ASSERT_TRUE(env->Load(data_dir() + "/boxbot.xml"));
  ASSERT_TRUE(env->Load(data_dir() + "/box.xml"));
  KinBodyPtr box = env->GetKinBody("box");
  RobotBasePtr boxbot = env->GetRobot("boxbot");
  RobotAndDOFPtr rad(new RobotAndDOF(boxbot, IntVec(), DOF_X | DOF_Y, Vector()));
  rad->GetRobot()->SetActiveDOFs(rad->GetJointIndices(), DOF_X | DOF_Y, Vector());
  Json::Value root = readJsonFile(string(DATA_DIR) + "/box_cast_test.json");
  DblVec start_dofs; start_dofs += -1.9, 0;
  rad->SetDOFValues(start_dofs);
  TrajOptProbPtr prob = ConstructProblem(root, env);
  TrajArray traj = prob->GetInitTraj();


  //shouldn't be necessary:
#if 0
  ASSERT_TRUE(!!prob);
  double dist_pen = .02, coeff = 10;
  prob->addCost(CostPtr(new CollisionCost(dist_pen, coeff, rad, prob->GetVarRow(0), prob->GetVarRow(1))));
  prob->addCost(CostPtr(new CollisionCost(dist_pen, coeff, rad, prob->GetVarRow(1), prob->GetVarRow(2))));
  CollisionCheckerPtr checker = CollisionChecker::GetOrCreate(*prob->GetEnv());
  {
    vector<Collision> collisions;
    checker->ContinuousCheckTrajectory(traj, *rad, collisions);
  }
  vector<Collision> collisions;
  cout << "traj: " << endl << traj << endl;
  checker->CastVsAll(*rad, rad->GetRobot()->GetLinks(), toDblVec(traj.row(0)), toDblVec(traj.row(1)), collisions);
  cout << collisions.size() << endl;
#endif

  BasicTrustRegionSQP opt(prob);
  if (plotting) opt.addCallback(PlotCallback(*prob));
  opt.initialize(trajToDblVec(prob->GetInitTraj()));
  opt.optimize();

  vector<Collision> collisions;
  CollisionChecker::GetOrCreate(*env)->ContinuousCheckTrajectory(getTraj(opt.x(), prob->GetVars()), *rad, collisions);
  RAVELOG_INFO("number of continuous collisions: %i\n", collisions.size());
  ASSERT_EQ(collisions.size(), 0);



}


int main(int argc, char** argv)
{
  {
    Config config;
    config.add(new Parameter<bool>("plotting", &plotting, "plotting"));
    config.add(new Parameter<bool>("verbose", &verbose, "verbose"));
    CommandParser parser(config);
    parser.read(argc, argv);
  }


  ::testing::InitGoogleTest(&argc, argv);
  RaveInitialize(false);
  if (verbose) RaveSetDebugLevel(Level_Debug);
  int result = RUN_ALL_TESTS();
  RaveDestroy();
  return result;
}
