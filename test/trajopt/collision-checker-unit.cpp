#include <gtest/gtest.h>
#include <openrave-core.h>
#include "trajopt/collision_checker.hpp"
#include "utils/stl_to_string.hpp"
#include "utils/eigen_conversions.hpp"
using namespace OpenRAVE;
using namespace std;
using namespace trajopt;
using namespace util;

string data_dir() {
  string out = DATA_DIR;
  return out;
}



void PrintCollisions(const vector<Collision>& collisions) {
  RAVELOG_INFO("%i collisions found\n", collisions.size());
  for (int i=0; i < collisions.size(); ++i) {
    const Collision& c = collisions[i];
    RAVELOG_INFO("%i: bodies: %s-%s. normal: %s. ptA: %s. distance: %.3e\n", i, c.linkA->GetName().c_str(), c.linkB->GetName().c_str(),
      Str(c.normalB2A).c_str(), Str(c.ptA).c_str(), c.distance);
  }
}



TEST(collision_checker, box_distance) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  ASSERT_TRUE(env->Load(data_dir() + "/box.xml"));
  KinBodyPtr box0 = env->GetKinBody("box");
  box0->SetName("box0");
  ASSERT_TRUE(env->Load(DATA_DIR + string("/box.xml")));
  KinBodyPtr box1 = env->GetKinBody("box");
  box1->SetName("box1");
  vector<KinBodyPtr> bodies; env->GetBodies(bodies);
  RAVELOG_DEBUG("%i kinbodies in rave env\n", bodies.size());
  CollisionCheckerPtr checker = CreateCollisionChecker(env);

#define EXPECT_NUM_COLLISIONS(n)\
  vector<Collision> allvsall_collisions, bodyvsall_collisions;\
  checker->AllVsAll(allvsall_collisions);\
  /*EXPECT_EQ(n, allvsall_collisions.size());*/\
  checker->BodyVsAll(*box0, bodyvsall_collisions);\
  EXPECT_EQ(n, bodyvsall_collisions.size());

  {
    checker->SetContactDistance(0);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(.9,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(1);
  }

  {
    checker->SetContactDistance(0);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(0);
  }

  {
    checker->SetContactDistance(.04);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(0);
  }

  {
    checker->SetContactDistance(.1);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.09,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(1);
  }


  {
    checker->SetContactDistance(.2);
    box1->SetTransform(Transform(Vector(1,0,0,0), Vector(1.1,0,0)));
    box0->SetTransform(Transform(Vector(1,0,0,0), Vector(0,0,0)));
    EXPECT_NUM_COLLISIONS(1);
  }

  {
    env->Remove(box1);
    EXPECT_NUM_COLLISIONS(0);
  }


}

#define EXPECT_VECTOR_NEAR(_vec0, _vec1, abstol)\
    OpenRAVE::Vector vec0=_vec0, vec1=_vec1;\
    if (fabs(vec0.x  - vec1.x) > abstol || \
        fabs(vec0.x  - vec1.x) > abstol ||\
        fabs(vec0.x  - vec1.x) > abstol) {\
      char msg[1000];\
      sprintf(msg, "%s != %s    (tol %.2e) at %s:%i\n", CSTR(vec0), CSTR(vec1), abstol, __FILE__, __LINE__);\
      GTEST_NONFATAL_FAILURE_(msg);\
    }

TEST(continuous_collisions, boxes) {
  EnvironmentBasePtr env = RaveCreateEnvironment();
  ASSERT_TRUE(env->Load(data_dir() + "/box.xml"));
  ASSERT_TRUE(env->Load(data_dir() + "/boxbot.xml"));
  KinBodyPtr box = env->GetKinBody("box");
  RobotBasePtr boxbot = env->GetRobot("boxbot");

  CollisionCheckerPtr checker = CreateCollisionChecker(env);
  {
    RobotAndDOFPtr rad(new RobotAndDOF(boxbot, IntVec(), DOF_X | DOF_Y, Vector()));
    TrajArray traj(2,2);
    traj << -1.9,0,  0,1.9;
    vector<Collision> collisions;
    checker->ContinuousCheckTrajectory(traj, *rad, collisions);
    ASSERT_EQ(collisions.size(), 1);
    Collision col = collisions[0];
    Vector robot_normal = (float)(col.linkA == boxbot->GetLinks()[0].get() ? 1. : -1.) * col.normalB2A;
    EXPECT_VECTOR_NEAR(robot_normal, Vector(-1, 0, 0), 1e-4);
  }
  
  #define TRAJ_TEST_BOILERPLATE\
    RobotAndDOFPtr rad(new RobotAndDOF(boxbot, IntVec(), DOF_X | DOF_Y, Vector()));\
    vector<Collision> collisions;\
    checker->CastVsAll(*rad, rad->GetRobot()->GetLinks(), toDblVec(traj.row(0)), toDblVec(traj.row(1)), collisions);\
    ASSERT_EQ(collisions.size(), 1);\
    Collision col = collisions[0];\
    Vector robot_normal = (float)(col.linkA == boxbot->GetLinks()[0].get() ? 1. : -1.) * col.normalB2A;

  {
    TrajArray traj(2,2);
    traj << -1.9,0,  0,1.9;
    TRAJ_TEST_BOILERPLATE
    EXPECT_VECTOR_NEAR(robot_normal, Vector(-1/sqrtf(2), 1/sqrtf(2), 0), 1e-4);
    EXPECT_NEAR(col.time, .5, 1e-1);
  }
  {
    TrajArray traj(2,2);
    traj << 0, .9,  0,2;
    TRAJ_TEST_BOILERPLATE    
    EXPECT_VECTOR_NEAR(robot_normal, Vector(0,1,0), 1e-4);
    EXPECT_NEAR(col.time, 0, 1e-6);
  }
  {
    TrajArray traj(2,2);
    traj << 0,2,  0,.9;
    TRAJ_TEST_BOILERPLATE    
    EXPECT_VECTOR_NEAR(robot_normal, Vector(0,1,0), 1e-4);
    EXPECT_NEAR(col.time, 1, 1e-6);
  }


}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  RaveInitialize(false);
  
  return RUN_ALL_TESTS();
}
