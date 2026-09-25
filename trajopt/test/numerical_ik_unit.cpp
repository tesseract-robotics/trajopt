#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <sstream>
#include <gtest/gtest.h>
#include <tesseract/common/types.h>
#include <tesseract/common/stopwatch.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/scene_state.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/utils.h>
#include <tesseract/visualization/visualization.h>
#include <tesseract/common/logging.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_common/clock.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/stl_to_string.hpp>
#include "trajopt_test_utils.hpp"

using namespace trajopt;
using namespace std;
using namespace trajopt_common;
using namespace tesseract::environment;
using namespace tesseract::collision;
using namespace tesseract::kinematics;
using namespace tesseract::visualization;
using namespace tesseract::scene_graph;
using namespace tesseract::common;

class NumericalIKTest : public testing::TestWithParam<const char*>
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>(); /**< Tesseract */
  Visualization::Ptr plotter_;                             /**< Trajopt Plotter */
  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    // Create plotting tool
    //    plotter_.reset(new tesseract_ros::ROSBasicPlotting(env_));

    SceneState::JointValues ipos;
    ipos["torso_lift_joint"] = 0.0;
    env_->setState(ipos);
  }
};

void runTest(const Environment::Ptr& env, const Visualization::Ptr& /*plotter*/, bool use_multi_threaded)
{
  TESSERACT_LOG_DEBUG("NumericalIKTest, numerical_ik1");

  const Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/numerical_ik1.json");

  //  plotter_->plotScene();

  ProblemConstructionInfo pci(env);
  pci.fromJson(root);
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  const TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  sco::BasicTrustRegionSQP::Ptr opt;
  if (use_multi_threaded)
  {
    opt = std::make_shared<sco::BasicTrustRegionSQPMultiThreaded>(prob);
    opt->getParameters().num_threads = 5;
  }
  else
  {
    opt = std::make_shared<sco::BasicTrustRegionSQP>(prob);
  }

  //  if (plotting)
  //  {
  //    opt.addCallback(PlotCallback(*prob, plotter));
  //  }

  TESSERACT_LOG_DEBUG("DOF: {}", prob->GetNumDOF());
  opt->initialize(DblVec(static_cast<std::size_t>(prob->GetNumDOF()), 0));
  const double tStart = GetClock();
  TESSERACT_LOG_DEBUG("Size: {}", opt->x().size());
  std::stringstream ss;
  ss << toVectorXd(opt->x()).transpose();
  TESSERACT_LOG_DEBUG("Initial Vars: {}", ss.str());
  const Eigen::Isometry3d change_base = prob->GetEnv()->getLinkTransform(prob->GetKin()->getBaseLinkId());
  Eigen::Isometry3d initial_pose = prob->GetKin()->calcFwdKin(toVectorXd(opt->x())).at("l_gripper_tool_frame");
  initial_pose = change_base * initial_pose;

  ss = std::stringstream();
  ss << initial_pose.translation().transpose();
  TESSERACT_LOG_DEBUG("Initial Position: {}", ss.str());
  tesseract::common::Stopwatch stopwatch;
  stopwatch.start();
  const sco::OptStatus status = opt->optimize();
  stopwatch.stop();
  TESSERACT_LOG_ERROR("Test took {} seconds.", stopwatch.elapsedSeconds());
  TESSERACT_LOG_DEBUG("Status: {}", sco::toString(status));
  Eigen::Isometry3d final_pose = prob->GetKin()->calcFwdKin(toVectorXd(opt->x())).at("l_gripper_tool_frame");
  final_pose = change_base * final_pose;

  Eigen::Isometry3d goal;
  goal.setIdentity();
  goal.translation() << 0.4, 0, 0.8;
  goal.linear() = Eigen::Quaterniond(0, 0, 1, 0).toRotationMatrix();

  for (auto i = 0; i < 4; ++i)
  {
    for (auto j = 0; j < 4; ++j)
    {
      EXPECT_NEAR(goal(i, j), final_pose(i, j), 1e-3);
    }
  }

  ss = std::stringstream();
  ss << final_pose.translation().transpose();
  TESSERACT_LOG_DEBUG("Final Position: {}", ss.str());

  ss = std::stringstream();
  ss << toVectorXd(opt->x()).transpose();
  TESSERACT_LOG_DEBUG("Final Vars: {}", ss.str());

  TESSERACT_LOG_DEBUG("planning time: {:.3f}", GetClock() - tStart);
}

TEST_F(NumericalIKTest, numerical_ik1)  // NOLINT
{
  runTest(env_, plotter_, false);
}

TEST_F(NumericalIKTest, numerical_ik1_multi_threaded)  // NOLINT
{
  runTest(env_, plotter_, true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  //  pnh.param("plotting", plotting, false);
  return RUN_ALL_TESTS();
}
