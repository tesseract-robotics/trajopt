#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <benchmark/benchmark.h>
#include <algorithm>
TESSERACT_COMMON_IGNORE_WARNINGS_POP
#include <tesseract_environment/environment.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_support/tesseract_support_resource_locator.h>

#include <trajopt/collision_terms.hpp>
#include <trajopt/common.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_utils/config.hpp>
#include <trajopt_utils/eigen_conversions.hpp>
#include <trajopt_utils/logging.hpp>
#include <trajopt_utils/stl_to_string.hpp>

#include "../trajopt_test_utils.hpp"

using namespace trajopt;
using namespace std;
using namespace util;
using namespace tesseract_environment;
using namespace tesseract_kinematics;
using namespace tesseract_collision;
using namespace tesseract_visualization;
using namespace tesseract_scene_graph;
using namespace tesseract_geometry;
using namespace tesseract_common;

/** @brief Benchmark trajopt simple collision solve */
static void BM_TRAJOPT_SIMPLE_COLLISION_SOLVE(benchmark::State& state, Environment::Ptr env, Json::Value root)
{
  for (auto _ : state)
  {
    TrajOptProb::Ptr prob = ConstructProblem(root, env);
    sco::BasicTrustRegionSQP opt(prob);
    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    opt.optimize();
  }
}

/** @brief Benchmark trajopt planning solve */
static void BM_TRAJOPT_PLANNING_SOLVE(benchmark::State& state, Environment::Ptr env, Json::Value root)
{
  for (auto _ : state)
  {
    ProblemConstructionInfo pci(env);
    pci.fromJson(root);
    pci.basic_info.convex_solver = sco::ModelType::OSQP;
    TrajOptProb::Ptr prob = ConstructProblem(pci);
    sco::BasicTrustRegionSQP opt(prob);
    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    opt.optimize();
  }
}

/** @brief Benchmark trajopt simple collision solve */
static void BM_TRAJOPT_MULTI_THREADED_SIMPLE_COLLISION_SOLVE(benchmark::State& state,
                                                             Environment::Ptr env,
                                                             Json::Value root)
{
  for (auto _ : state)
  {
    TrajOptProb::Ptr prob = ConstructProblem(root, env);
    sco::BasicTrustRegionSQPMultiThreaded opt(prob);
    opt.getParameters().num_threads = 5;
    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    opt.optimize();
  }
}

/** @brief Benchmark trajopt planning solve */
static void BM_TRAJOPT_MULTI_THREADED_PLANNING_SOLVE(benchmark::State& state, Environment::Ptr env, Json::Value root)
{
  for (auto _ : state)
  {
    ProblemConstructionInfo pci(env);
    pci.fromJson(root);
    pci.basic_info.convex_solver = sco::ModelType::OSQP;
    TrajOptProb::Ptr prob = ConstructProblem(pci);
    sco::BasicTrustRegionSQPMultiThreaded opt(prob);
    opt.getParameters().num_threads = 5;
    opt.initialize(trajToDblVec(prob->GetInitTraj()));
    opt.optimize();
  }
}

int main(int argc, char** argv)
{
  gLogLevel = util::LevelError;

  //////////////////////////////////////
  // Simple Collision Solve
  //////////////////////////////////////
  {
    Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/simple_collision_test.json");
    boost::filesystem::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.urdf");
    boost::filesystem::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/spherebot.srdf");

    ResourceLocator::Ptr locator = std::make_shared<TrajOptSupportResourceLocator>();
    auto env = std::make_shared<Environment>();
    env->init(urdf_file, srdf_file, locator);

    std::unordered_map<std::string, double> ipos;
    ipos["spherebot_x_joint"] = -0.75;
    ipos["spherebot_y_joint"] = 0.75;
    env->setState(ipos);

    std::function<void(benchmark::State&, Environment::Ptr, Json::Value)> BM_SOLVE_FUNC =
        BM_TRAJOPT_SIMPLE_COLLISION_SOLVE;
    std::string name = "BM_TRAJOPT_SIMPLE_COLLISION_SOLVE";
    benchmark::RegisterBenchmark(name.c_str(), BM_SOLVE_FUNC, env, root)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMicrosecond);
  }

  //////////////////////////////////////
  // Planning Solve
  //////////////////////////////////////
  {
    Json::Value root = readJsonFile(std::string(TRAJOPT_DIR) + "/test/data/config/arm_around_table.json");
    tesseract_common::fs::path urdf_file(std::string(TRAJOPT_DIR) + "/test/data/arm_around_table.urdf");
    tesseract_common::fs::path srdf_file(std::string(TRAJOPT_DIR) + "/test/data/pr2.srdf");

    ResourceLocator::Ptr locator = std::make_shared<TrajOptSupportResourceLocator>();
    auto env = std::make_shared<Environment>();
    env->init(urdf_file, srdf_file, locator);

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0;
    ipos["r_shoulder_pan_joint"] = -1.832;
    ipos["r_shoulder_lift_joint"] = -0.332;
    ipos["r_upper_arm_roll_joint"] = -1.011;
    ipos["r_elbow_flex_joint"] = -1.437;
    ipos["r_forearm_roll_joint"] = -1.1;
    ipos["r_wrist_flex_joint"] = -1.926;
    ipos["r_wrist_roll_joint"] = 3.074;
    env->setState(ipos);

    std::function<void(benchmark::State&, Environment::Ptr, Json::Value)> BM_SOLVE_FUNC = BM_TRAJOPT_PLANNING_SOLVE;
    std::string name = "BM_TRAJOPT_PLANNING_SOLVE";
    benchmark::RegisterBenchmark(name.c_str(), BM_SOLVE_FUNC, env, root)
        ->UseRealTime()
        ->Unit(benchmark::TimeUnit::kMillisecond)
        ->MinTime(100);
  }

  benchmark::Initialize(&argc, argv);
  benchmark::RunSpecifiedBenchmarks();
}
