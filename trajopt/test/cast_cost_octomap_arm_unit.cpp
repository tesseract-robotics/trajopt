#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <ctime>
#include <gtest/gtest.h>
#include <octomap/Pointcloud.h>
#include <octomap/OcTree.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/commands.h>
#include <tesseract/environment/utils.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/octree.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/utils.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
#include <trajopt_common/clock.hpp>
#include <trajopt_common/config.hpp>
#include <trajopt_common/eigen_conversions.hpp>
#include <trajopt_common/logging.hpp>
#include <trajopt_common/stl_to_string.hpp>
#include "trajopt_test_utils.hpp"

using namespace trajopt;
using namespace std;
using namespace trajopt_common;
using namespace tesseract::environment;
using namespace tesseract::kinematics;
using namespace tesseract::collision;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;
using namespace tesseract::common;

static const double LONGEST_VALID_SEGMENT_LENGTH = 0.05;

class CastOctomapArmTest : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>();

  void SetUp() override
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/arm_around_table.urdf");
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/pr2.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    std::unordered_map<std::string, double> ipos;
    ipos["torso_lift_joint"] = 0.0;
    env_->setState(ipos);

    gLogLevel = trajopt_common::LevelError;

    // Create a 0.4m octree at resolution 0.2m (2^3 = 8 voxels).
    // Coarse resolution keeps collision checking fast while still exercising the octree code path.
    octomap::Pointcloud point_cloud;
    const double delta = 0.2;
    const double half_size = 0.2;
    auto length = static_cast<int>(2 * half_size / delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          point_cloud.push_back(static_cast<float>(-half_size + (x + 0.5) * delta),
                                static_cast<float>(-half_size + (y + 0.5) * delta),
                                static_cast<float>(-half_size + (z + 0.5) * delta));

    const std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(delta);
    octree->insertPointCloud(point_cloud, octomap::point3d(0, 0, 0));

    const Octree::Ptr coll_octree = std::make_shared<Octree>(octree, OctreeSubType::BOX);
    auto vis_box = std::make_shared<Box>(0.4, 0.4, 0.4);

    auto visual = std::make_shared<Visual>();
    visual->geometry = vis_box;
    visual->origin = Eigen::Isometry3d::Identity();

    auto collision = std::make_shared<Collision>();
    collision->geometry = coll_octree;
    collision->origin = Eigen::Isometry3d::Identity();

    Link new_link("octomap_attached");
    new_link.visual.push_back(visual);
    new_link.collision.push_back(collision);

    // Place the octree where the arm sweeps through during the arm_around_table motion.
    // At (0.5, -0.3, 0.8) relative to base_link, the initial straight-line trajectory
    // passes through the octree with multiple arm links.
    Joint new_joint("base_link-octomap_attached");
    new_joint.type = JointType::FIXED;
    new_joint.parent_link_name = "base_link";
    new_joint.child_link_name = "octomap_attached";
    new_joint.parent_to_joint_origin_transform = Eigen::Isometry3d::Identity();
    new_joint.parent_to_joint_origin_transform.translation() = Eigen::Vector3d(0.5, -0.3, 0.8);

    env_->applyCommand(std::make_shared<AddLinkCommand>(new_link, new_joint));
  }
};

/** @brief Count contacts involving a specific link */
static int countLinkContacts(const std::vector<ContactResultMap>& collisions, const std::string& link_name)
{
  int count = 0;
  for (const auto& step : collisions)
    for (const auto& pair : step)
      for (const auto& r : pair.second)
        if (r.link_ids[0].name() == link_name || r.link_ids[1].name() == link_name)
          ++count;
  return count;
}

/** @brief Log contacts involving a specific link */
static void logContacts(const std::vector<ContactResultMap>& collisions,
                        const std::string& label,
                        const std::string& filter_link)
{
  for (size_t i = 0; i < collisions.size(); ++i)
    for (const auto& pair : collisions[i])
      for (const auto& r : pair.second)
        if (r.link_ids[0].name() == filter_link || r.link_ids[1].name() == filter_link)
          CONSOLE_BRIDGE_logError("  %s step %zu: %s vs %s dist=%.4f",
                                  label.c_str(),
                                  i,
                                  r.link_ids[0].name().c_str(),
                                  r.link_ids[1].name().c_str(),
                                  r.distance);
}

/**
 * @brief Compare Bullet vs Coal continuous collision detection on the same trajectory.
 *
 * This test does NOT run the optimizer. It checks the initial arm_around_table trajectory
 * with both continuous backends and compares octree contact counts. The hypothesis is that
 * Coal's continuous check under-reports octree contacts compared to Bullet.
 */
TEST_F(CastOctomapArmTest, continuous_detection_gap)  // NOLINT
{
  const Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/arm_around_table.json");

  std::unordered_map<std::string, double> ipos;
  ipos["torso_lift_joint"] = 0;
  ipos["r_shoulder_pan_joint"] = -1.832;
  ipos["r_shoulder_lift_joint"] = -0.332;
  ipos["r_upper_arm_roll_joint"] = -1.011;
  ipos["r_elbow_flex_joint"] = -1.437;
  ipos["r_forearm_roll_joint"] = -1.1;
  ipos["r_wrist_flex_joint"] = -1.926;
  ipos["r_wrist_roll_joint"] = 3.074;
  env_->setState(ipos);

  // Build problem to get the initial trajectory and kinematic info
  ProblemConstructionInfo pci(env_);
  pci.fromJson(root);
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  const TrajOptProb::Ptr prob = ConstructProblem(pci);
  ASSERT_TRUE(!!prob);

  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& active_links = prob->GetKin()->getActiveLinkNames();
  const TrajArray& init_traj = prob->GetInitTraj();
  const tesseract::scene_graph::StateSolver::UPtr state_solver = prob->GetEnv()->getStateSolver();

  tesseract::collision::CollisionCheckConfig config;
  config.type = tesseract::collision::CollisionEvaluatorType::CONTINUOUS;
  config.longest_valid_segment_length = LONGEST_VALID_SEGMENT_LENGTH;
  // Check ALL trajectory steps so both backends are compared on the same segments.
  // The default FIRST exit condition stops at the first collision, which masks
  // differences when backends detect collisions at different steps.
  config.exit_condition = tesseract::collision::CollisionCheckExitType::ALL;

  // --- Check with Bullet continuous ---
  ASSERT_TRUE(env_->setActiveContinuousContactManager("BulletCastBVHManager"));
  const ContinuousContactManager::Ptr bullet_mgr = env_->getContinuousContactManager();
  bullet_mgr->setActiveCollisionObjects(active_links);
  bullet_mgr->setDefaultCollisionMargin(0);

  std::vector<ContactResultMap> bullet_collisions;
  bool bullet_found = checkTrajectory(bullet_collisions, *bullet_mgr, *state_solver, joint_names, init_traj, config);
  int bullet_octree = countLinkContacts(bullet_collisions, "octomap_attached");

  CONSOLE_BRIDGE_logError("Bullet continuous: collision=%s, octree contacts=%d",
                          bullet_found ? "YES" : "NO",
                          bullet_octree);
  logContacts(bullet_collisions, "bullet", "octomap_attached");

  // --- Check with Coal continuous ---
  ASSERT_TRUE(env_->setActiveContinuousContactManager("CoalCastBVHManager"));
  const ContinuousContactManager::Ptr coal_mgr = env_->getContinuousContactManager();
  coal_mgr->setActiveCollisionObjects(active_links);
  coal_mgr->setDefaultCollisionMargin(0);

  std::vector<ContactResultMap> coal_collisions;
  bool coal_found = checkTrajectory(coal_collisions, *coal_mgr, *state_solver, joint_names, init_traj, config);
  int coal_octree = countLinkContacts(coal_collisions, "octomap_attached");

  CONSOLE_BRIDGE_logError("Coal continuous:   collision=%s, octree contacts=%d",
                          coal_found ? "YES" : "NO",
                          coal_octree);
  logContacts(coal_collisions, "coal", "octomap_attached");

  // --- Per-step comparison ---
  const size_t max_steps = std::max(bullet_collisions.size(), coal_collisions.size());
  for (size_t step = 0; step < max_steps; ++step)
  {
    int b_count = (step < bullet_collisions.size()) ? countLinkContacts({ bullet_collisions[step] }, "octomap_attached")
                                                    : 0;
    int c_count =
        (step < coal_collisions.size()) ? countLinkContacts({ coal_collisions[step] }, "octomap_attached") : 0;
    if (b_count > 0 || c_count > 0)
      CONSOLE_BRIDGE_logError("  Step %zu: Bullet=%d  Coal=%d octree contacts", step, b_count, c_count);
  }

  // --- Cross-check with discrete Bullet (ground truth) ---
  const DiscreteContactManager::Ptr disc_mgr = env_->getDiscreteContactManager();
  disc_mgr->setActiveCollisionObjects(active_links);
  disc_mgr->setDefaultCollisionMargin(0);

  tesseract::collision::CollisionCheckConfig disc_config;
  disc_config.type = tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE;
  disc_config.longest_valid_segment_length = 0.02;

  std::vector<ContactResultMap> disc_collisions;
  bool disc_found = checkTrajectory(disc_collisions, *disc_mgr, *state_solver, joint_names, init_traj, disc_config);
  int disc_octree = countLinkContacts(disc_collisions, "octomap_attached");

  CONSOLE_BRIDGE_logError("Discrete (Bullet): collision=%s, octree contacts=%d",
                          disc_found ? "YES" : "NO",
                          disc_octree);

  // Both continuous backends should detect octree collisions on this trajectory
  EXPECT_TRUE(bullet_found) << "Bullet continuous should detect collision";
  EXPECT_TRUE(coal_found) << "Coal continuous should detect collision";
  EXPECT_TRUE(disc_found) << "Discrete check should detect collision";

  // Coal should find a comparable number of octree contacts as Bullet.
  // A large gap means Coal's continuous check is blind to octree geometry.
  EXPECT_GT(coal_octree, 0) << "Coal should detect at least some octree contacts";
  EXPECT_GE(coal_octree, bullet_octree / 2)
      << "Coal found " << coal_octree << " octree contacts vs Bullet's " << bullet_octree
      << " — Coal's continuous octree detection is significantly weaker";
}

/**
 * @brief Optimization result for backend comparison.
 */
struct OptResult
{
  sco::OptStatus status;
  double cost;
  int disc_octree_contacts;
  double min_octree_distance;
};

/**
 * @brief Run trajopt optimization with the given continuous backend and verify with discrete check.
 *
 * Uses a small number of iterations to keep runtime reasonable (~20s per iteration with octrees).
 * The goal is not to find a collision-free trajectory but to verify both backends make comparable
 * optimization progress.
 */
static OptResult optimizeAndVerify(Environment::Ptr env, const std::string& manager_name)
{
  EXPECT_TRUE(env->setActiveContinuousContactManager(manager_name));

  const Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/arm_around_table.json");

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

  ProblemConstructionInfo pci(env);
  pci.fromJson(root);
  pci.basic_info.convex_solver = sco::ModelType::OSQP;
  const TrajOptProb::Ptr prob = ConstructProblem(pci);
  EXPECT_TRUE(!!prob);
  if (!prob)
    return { sco::INVALID, 0.0, -1, 0.0 };

  const std::vector<std::string>& joint_names = prob->GetKin()->getJointNames();
  const std::vector<std::string>& active_links = prob->GetKin()->getActiveLinkNames();
  const tesseract::scene_graph::StateSolver::UPtr state_solver = prob->GetEnv()->getStateSolver();

  auto opt = std::make_shared<sco::BasicTrustRegionSQP>(prob);
  // Few iterations keep runtime manageable (~20s/iter with octree collision checking).
  // Enough for the optimizer to show avoidance progress if the backend provides gradients.
  opt->getParameters().max_iter = 3;
  opt->getParameters().max_merit_coeff_increases = 1;
  opt->initialize(trajToDblVec(prob->GetInitTraj()));
  const double t_start = GetClock();
  opt->optimize();
  CONSOLE_BRIDGE_logError("%s optimization: %.1fs, status=%d",
                          manager_name.c_str(),
                          (GetClock() - t_start) / 1000.0,
                          static_cast<int>(opt->results().status));

  const double final_cost = opt->results().total_cost;
  TrajArray final_traj = getTraj(opt->x(), prob->GetVars());

  // Verify final trajectory with discrete Bullet (independent ground truth)
  const DiscreteContactManager::Ptr disc_mgr = prob->GetEnv()->getDiscreteContactManager();
  disc_mgr->setActiveCollisionObjects(active_links);
  disc_mgr->setDefaultCollisionMargin(0);

  tesseract::collision::CollisionCheckConfig disc_config;
  disc_config.type = tesseract::collision::CollisionEvaluatorType::LVS_DISCRETE;
  disc_config.longest_valid_segment_length = 0.02;

  std::vector<ContactResultMap> disc_collisions;
  checkTrajectory(disc_collisions, *disc_mgr, *state_solver, joint_names, final_traj, disc_config);
  int disc_octree = countLinkContacts(disc_collisions, "octomap_attached");

  // Find the minimum (most-penetrating) distance among octree contacts
  double min_dist = 0.0;
  for (const auto& step : disc_collisions)
    for (const auto& pair : step)
      for (const auto& r : pair.second)
        if (r.link_ids[0].name() == "octomap_attached" || r.link_ids[1].name() == "octomap_attached")
          min_dist = std::min(min_dist, r.distance);

  CONSOLE_BRIDGE_logError("%s: cost=%.1f, octree contacts=%d, min dist=%.4f",
                          manager_name.c_str(),
                          final_cost,
                          disc_octree,
                          min_dist);
  if (disc_octree > 0)
    logContacts(disc_collisions, manager_name, "octomap_attached");

  return { opt->results().status, final_cost, disc_octree, min_dist };
}

/**
 * @brief Verify Coal produces equivalent optimization results to Bullet on octree scenarios.
 *
 * Both backends run a few optimizer iterations on the same arm-around-table problem with an octree
 * obstacle. The test asserts Coal is no worse than Bullet: similar cost, similar or fewer residual
 * octree contacts, and no deeper penetration. Neither backend is expected to fully solve this
 * problem in so few iterations.
 */
TEST_F(CastOctomapArmTest, optimization_discrete_verify)  // NOLINT
{
  auto bullet = optimizeAndVerify(env_, "BulletCastBVHManager");
  auto coal = optimizeAndVerify(env_, "CoalCastBVHManager");

  CONSOLE_BRIDGE_logError("Summary: Bullet cost=%.1f octree=%d min_dist=%.4f, "
                          "Coal cost=%.1f octree=%d min_dist=%.4f",
                          bullet.cost,
                          bullet.disc_octree_contacts,
                          bullet.min_octree_distance,
                          coal.cost,
                          coal.disc_octree_contacts,
                          coal.min_octree_distance);

  // Coal should produce no more octree contacts than Bullet
  EXPECT_LE(coal.disc_octree_contacts, bullet.disc_octree_contacts + 2)
      << "Coal produced significantly more octree contacts than Bullet";

  // Coal's deepest penetration should not be much worse than Bullet's
  EXPECT_GE(coal.min_octree_distance, bullet.min_octree_distance - 0.01)
      << "Coal penetrates octree significantly deeper than Bullet";

  // Coal's final cost should be in the same ballpark as Bullet's
  if (bullet.cost > 0)
  {
    EXPECT_LT(coal.cost, bullet.cost * 2.0)
        << "Coal's optimization cost is much higher than Bullet's, suggesting weaker gradients";
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
