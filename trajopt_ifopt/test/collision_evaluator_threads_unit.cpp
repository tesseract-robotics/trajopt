#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <gtest/gtest.h>
#include <Eigen/Core>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <filesystem>
#include <functional>
#include <string>
#include <thread>
#include <vector>
TRAJOPT_IGNORE_WARNINGS_POP

#include <tesseract/common/resource_locator.h>
#include <tesseract/common/logging.h>
#include <tesseract/collision/types.h>
#include <tesseract/environment/environment.h>
#include <tesseract/kinematics/joint_group.h>

#include <trajopt_common/collision_types.h>
#include <trajopt_ifopt/constraints/collision/continuous_collision_evaluators.h>
#include <trajopt_ifopt/constraints/collision/discrete_collision_evaluators.h>

using namespace trajopt_ifopt;
using namespace tesseract::collision;
using namespace tesseract::environment;

namespace
{
/** @brief Threads that evaluate one shared evaluator at once */
constexpr std::size_t NUM_THREADS{ 4 };

/** @brief Evaluations each thread runs */
constexpr int NUM_EVALUATIONS{ 100 };

/** @brief Collision margin (m), larger than every separation below, so each evaluation reports a contact */
constexpr double MARGIN{ 2.0 };

/** @brief Collision margin buffer (m) */
constexpr double MARGIN_BUFFER{ 0.05 };

/** @brief Longest valid segment length (m) for the LVS types: a 1 m segment is checked in 20 steps */
constexpr double LONGEST_VALID_SEGMENT_LENGTH{ 0.05 };

/** @brief Gradient sets kept per evaluation */
constexpr std::size_t MAX_ALLOWED{ 1000 };

/** @brief Tolerance (m) between a thread's distances and the single-threaded reference */
constexpr double DISTANCE_TOL{ 1e-6 };

/**
 * @brief The spherebot's x position (m) for thread i.
 *
 * The robot and the obstacle are spheres of radius 0.5 m at the origin, so thread i stays 0.2 + 0.1 * i m clear of
 * the obstacle and gets a distance no other thread gets.
 */
double threadX(std::size_t i) { return 1.2 + (0.1 * static_cast<double>(i)); }

/** @brief The distances of every contact in the map, sorted */
std::vector<double> sortedDistances(const ContactResultMap& contacts)
{
  ContactResultVector flat;
  contacts.flattenCopyResults(flat);
  std::vector<double> distances;
  distances.reserve(flat.size());
  for (const auto& contact : flat)
    distances.push_back(contact.distance);
  std::sort(distances.begin(), distances.end());
  return distances;
}

bool near(const std::vector<double>& a, const std::vector<double>& b)
{
  if (a.size() != b.size())
    return false;
  for (std::size_t j = 0; j < a.size(); ++j)
  {
    if (std::abs(a[j] - b[j]) > DISTANCE_TOL)
      return false;
  }
  return true;
}

/**
 * @brief Evaluate one shared evaluator from NUM_THREADS threads at once and compare against a single-threaded run.
 * @param evaluate Runs the evaluator once at thread i's configuration and returns its contacts
 */
void runSharedEvaluator(const std::function<ContactResultMap(std::size_t)>& evaluate)
{
  std::vector<std::vector<double>> reference(NUM_THREADS);
  for (std::size_t i = 0; i < NUM_THREADS; ++i)
  {
    reference[i] = sortedDistances(evaluate(i));
    ASSERT_FALSE(reference[i].empty()) << "thread " << i << " has no contact to compare";
  }
  // Each thread must see its own distances, or a result computed at another thread's state would go unnoticed
  for (std::size_t i = 1; i < NUM_THREADS; ++i)
    ASSERT_FALSE(near(reference[i], reference[i - 1])) << "threads " << i - 1 << " and " << i << " are alike";

  std::vector<int> mismatches(NUM_THREADS, 0);
  std::atomic<std::size_t> ready{ 0 };
  std::vector<std::thread> threads;
  threads.reserve(NUM_THREADS);
  for (std::size_t i = 0; i < NUM_THREADS; ++i)
  {
    threads.emplace_back([&, i] {
      // Start together, so the evaluations overlap
      ++ready;
      while (ready.load() < NUM_THREADS)
        std::this_thread::yield();

      for (int k = 0; k < NUM_EVALUATIONS; ++k)
      {
        if (!near(sortedDistances(evaluate(i)), reference[i]))
          ++mismatches[i];
      }
    });
  }
  for (auto& thread : threads)
    thread.join();

  EXPECT_EQ(mismatches, std::vector<int>(NUM_THREADS, 0));
}
}  // namespace

class CollisionEvaluatorThreadsTest : public testing::Test
{
public:
  Environment::Ptr env = std::make_shared<Environment>();
  std::shared_ptr<const tesseract::kinematics::JointGroup> manip;

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/spherebot.srdf");

    const tesseract::common::ResourceLocator::Ptr locator =
        std::make_shared<tesseract::common::GeneralResourceLocator>();
    ASSERT_TRUE(env->init(urdf_file, srdf_file, locator));
    manip = env->getJointGroup("manipulator");

    tesseract::common::getLogger()->set_level(spdlog::level::err);
  }

  static trajopt_common::TrajOptCollisionConfig makeConfig(CollisionEvaluatorType type)
  {
    trajopt_common::TrajOptCollisionConfig config(MARGIN, 1);
    config.collision_check_config.type = type;
    config.collision_check_config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
    config.collision_margin_buffer = MARGIN_BUFFER;
    return config;
  }

  /** @brief Evaluate a segment evaluator on thread i's segment: x fixed, y swept from -0.5 m to 0.5 m */
  static ContactResultMap evaluateSegment(ContinuousCollisionEvaluator& evaluator, std::size_t i)
  {
    Eigen::VectorXd dof_vals0(2);
    dof_vals0 << threadX(i), -0.5;
    Eigen::VectorXd dof_vals1(2);
    dof_vals1 << threadX(i), 0.5;

    trajopt_common::CollisionCacheData data;
    evaluator.calcCollisionData(data, dof_vals0, dof_vals1, false, false, MAX_ALLOWED);
    return data.contact_results_map;
  }
};

TEST_F(CollisionEvaluatorThreadsTest, SingleTimestepSharedAcrossThreads)  // NOLINT
{
  SingleTimestepCollisionEvaluator evaluator(manip, env, makeConfig(CollisionEvaluatorType::DISCRETE));
  runSharedEvaluator([&evaluator](std::size_t i) {
    Eigen::VectorXd dof_vals(2);
    dof_vals << threadX(i), 0.0;

    trajopt_common::CollisionCacheData data;
    evaluator.calcCollisions(data, dof_vals, MAX_ALLOWED);
    return data.contact_results_map;
  });
}

TEST_F(CollisionEvaluatorThreadsTest, LVSDiscreteSharedAcrossThreads)  // NOLINT
{
  LVSDiscreteCollisionEvaluator evaluator(manip, env, makeConfig(CollisionEvaluatorType::LVS_DISCRETE));
  runSharedEvaluator([&evaluator](std::size_t i) { return evaluateSegment(evaluator, i); });
}

TEST_F(CollisionEvaluatorThreadsTest, LVSContinuousSharedAcrossThreads)  // NOLINT
{
  LVSContinuousCollisionEvaluator evaluator(manip, env, makeConfig(CollisionEvaluatorType::LVS_CONTINUOUS));
  runSharedEvaluator([&evaluator](std::size_t i) { return evaluateSegment(evaluator, i); });
}

TEST_F(CollisionEvaluatorThreadsTest, ContinuousSharedAcrossThreads)  // NOLINT
{
  LVSContinuousCollisionEvaluator evaluator(manip, env, makeConfig(CollisionEvaluatorType::CONTINUOUS));
  runSharedEvaluator([&evaluator](std::size_t i) { return evaluateSegment(evaluator, i); });
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
