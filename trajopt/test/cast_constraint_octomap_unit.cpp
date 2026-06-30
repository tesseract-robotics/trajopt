#include <trajopt_common/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <algorithm>
#include <ctime>
#include <gtest/gtest.h>
#include <octomap/Pointcloud.h>
#include <octomap/OcTree.h>
#include <tesseract/common/types.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/collision/continuous_contact_manager.h>
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/scene_graph/link.h>
#include <tesseract/scene_graph/joint.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/environment/environment.h>
#include <tesseract/environment/commands.h>
#include <tesseract/environment/utils.h>
#include <tesseract/geometry/impl/box.h>
#include <tesseract/geometry/impl/octree.h>
#include <tesseract/visualization/visualization.h>
#include <console_bridge/console.h>
TRAJOPT_IGNORE_WARNINGS_POP

#include <trajopt/collision_terms.hpp>
#include <trajopt/plot_callback.hpp>
#include <trajopt/utils.hpp>
#include <trajopt/problem_description.hpp>
#include <trajopt_sco/optimizers.hpp>
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
using namespace tesseract::visualization;
using namespace tesseract::scene_graph;
using namespace tesseract::geometry;
using namespace tesseract::common;

static const bool plotting = false;

namespace
{
struct ExpectedContactSummary
{
  bool found_pair{ false };
  double min_distance{ std::numeric_limits<double>::max() };
  std::size_t pair_contact_count{ 0 };
};

bool isExpectedPair(const tesseract::collision::ContactResult& contact,
                    const std::string& link_name_1,
                    const std::string& link_name_2)
{
  return ((contact.link_ids[0].name() == link_name_1 && contact.link_ids[1].name() == link_name_2) ||
          (contact.link_ids[0].name() == link_name_2 && contact.link_ids[1].name() == link_name_1));
}

ExpectedContactSummary logAndSummarizeContacts(const std::vector<ContactResultMap>& collisions,
                                               const std::string& test_name,
                                               const std::string& expected_link_name_1,
                                               const std::string& expected_link_name_2)
{
  ExpectedContactSummary summary;

  for (std::size_t step = 0; step < collisions.size(); ++step)
  {
    ContactResultVector flattened_results;
    collisions[step].flattenCopyResults(flattened_results);
    std::sort(flattened_results.begin(),
              flattened_results.end(),
              [](const tesseract::collision::ContactResult& lhs, const tesseract::collision::ContactResult& rhs) {
                if (lhs.link_ids[0].name() != rhs.link_ids[0].name())
                  return lhs.link_ids[0].name() < rhs.link_ids[0].name();
                if (lhs.link_ids[1].name() != rhs.link_ids[1].name())
                  return lhs.link_ids[1].name() < rhs.link_ids[1].name();

                if (lhs.distance != rhs.distance)
                  return lhs.distance < rhs.distance;

                return lhs.cc_time < rhs.cc_time;
              });

    for (const auto& contact : flattened_results)
    {
      CONSOLE_BRIDGE_logError("%s step=%zu pair=(%s,%s) distance=%.9f penetration=%s cc_time=(%.6f,%.6f) shape=(%d,%d) "
                              "subshape=(%d,%d)",
                              test_name.c_str(),
                              step,
                              contact.link_ids[0].name().c_str(),
                              contact.link_ids[1].name().c_str(),
                              contact.distance,
                              (contact.distance <= 0.0) ? "true" : "false",
                              contact.cc_time[0],
                              contact.cc_time[1],
                              contact.shape_id[0],
                              contact.shape_id[1],
                              contact.subshape_id[0],
                              contact.subshape_id[1]);

      if (isExpectedPair(contact, expected_link_name_1, expected_link_name_2))
      {
        summary.found_pair = true;
        summary.min_distance = std::min(summary.min_distance, contact.distance);
        ++summary.pair_contact_count;
      }
    }
  }

  if (summary.found_pair)
  {
    CONSOLE_BRIDGE_logError("%s expected_pair=(%s,%s) contacts=%zu min_distance=%.9f penetration=%s",
                            test_name.c_str(),
                            expected_link_name_1.c_str(),
                            expected_link_name_2.c_str(),
                            summary.pair_contact_count,
                            summary.min_distance,
                            (summary.min_distance <= 0.0) ? "true" : "false");
  }
  else
  {
    CONSOLE_BRIDGE_logError("%s expected_pair=(%s,%s) contacts=0",
                            test_name.c_str(),
                            expected_link_name_1.c_str(),
                            expected_link_name_2.c_str());
  }

  return summary;
}

}  // namespace

class CastConstraintOctomapTestBase : public testing::Test
{
public:
  Environment::Ptr env_ = std::make_shared<Environment>();
  Visualization::Ptr plotter_;

protected:
  void initEnvironment(const std::string& urdf_filename)
  {
    const std::filesystem::path urdf_file(std::string(TRAJOPT_DATA_DIR) + "/" + urdf_filename);
    const std::filesystem::path srdf_file(std::string(TRAJOPT_DATA_DIR) + "/boxbot.srdf");

    const ResourceLocator::Ptr locator = std::make_shared<tesseract::common::GeneralResourceLocator>();
    EXPECT_TRUE(env_->init(urdf_file, srdf_file, locator));

    gLogLevel = trajopt_common::LevelError;

    octomap::Pointcloud point_cloud;
    const double delta = 0.05;
    auto length = static_cast<int>(1 / delta);

    for (int x = 0; x < length; ++x)
      for (int y = 0; y < length; ++y)
        for (int z = 0; z < length; ++z)
          point_cloud.push_back(-0.5F + static_cast<float>(x * delta),
                                -0.5F + static_cast<float>(y * delta),
                                -0.5F + static_cast<float>(z * delta));

    const std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(2 * delta);
    octree->insertPointCloud(point_cloud, octomap::point3d(0, 0, 0));

    const Octree::Ptr coll_octree = std::make_shared<Octree>(octree, OctreeSubType::BOX);
    auto vis_box = std::make_shared<Box>(1.0, 1.0, 1.0);

    auto visual = std::make_shared<Visual>();
    visual->geometry = vis_box;
    visual->origin = Eigen::Isometry3d::Identity();

    auto collision = std::make_shared<Collision>();
    collision->geometry = coll_octree;
    collision->origin = Eigen::Isometry3d::Identity();

    Link new_link("octomap_attached");
    new_link.visual.push_back(visual);
    new_link.collision.push_back(collision);

    Joint new_joint("base_link-octomap_attached");
    new_joint.type = JointType::FIXED;
    new_joint.parent_link_id = "base_link";
    new_joint.child_link_id = "octomap_attached";

    env_->applyCommand(std::make_shared<AddLinkCommand>(new_link, new_joint));
  }
};

class CastConstraintOctomapTest : public CastConstraintOctomapTestBase
{
public:
  void SetUp() override { initEnvironment("boxbot_world.urdf"); }
};

class CastConstraintOctomapCylinderTest : public CastConstraintOctomapTestBase
{
public:
  void SetUp() override { initEnvironment("boxbot_cylinder_world.urdf"); }
};

void runConstraintTest(const Environment::Ptr& env,
                       const Visualization::Ptr& plotter,
                       bool use_multi_threaded,
                       const std::string& test_name,
                       const std::string& expected_link_name,
                       const std::string& config_filename,
                       bool expect_final_collision_free)
{
  CONSOLE_BRIDGE_logDebug("CastConstraintOctomapTest, boxes");

  const Json::Value root = readJsonFile(std::string(TRAJOPT_DATA_DIR) + "/config/" + config_filename);

  std::unordered_map<std::string, double> ipos;
  ipos["boxbot_x_joint"] = -1.9;
  ipos["boxbot_y_joint"] = 0;
  env->setState(ipos);

  const TrajOptProb::Ptr prob = ConstructProblem(root, env);
  ASSERT_TRUE(!!prob);
  EXPECT_EQ(prob->getCosts().size(), 1);

  std::vector<ContactResultMap> collisions;
  const tesseract::scene_graph::StateSolver::UPtr state_solver = prob->GetEnv()->getStateSolver();
  const ContinuousContactManager::Ptr manager = prob->GetEnv()->getContinuousContactManager();

  CONSOLE_BRIDGE_logError("CastConstraintOctomapTest using continuous manager: %s", manager->getName().c_str());

  manager->setActiveCollisionObjects(prob->GetKin()->getActiveLinkIds());
  manager->setDefaultCollisionMargin(0);

  tesseract::collision::CollisionCheckConfig config;
  config.type = tesseract::collision::CollisionEvaluatorType::CONTINUOUS;
  bool found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), prob->GetInitTraj(), config);

  EXPECT_TRUE(found);
  const ExpectedContactSummary initial_summary =
      logAndSummarizeContacts(collisions, test_name, "octomap_attached", expected_link_name);
  EXPECT_TRUE(initial_summary.found_pair);
  EXPECT_GT(initial_summary.pair_contact_count, 0);
  EXPECT_LT(initial_summary.min_distance, 0.11);
  CONSOLE_BRIDGE_logDebug((found) ? ("Initial trajectory is in collision") : ("Initial trajectory is collision free"));

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

  if (plotting)
    opt->addCallback(PlotCallback(plotter));
  opt->initialize(trajToDblVec(prob->GetInitTraj()));
  opt->optimize();

  CONSOLE_BRIDGE_logError("%s optimizer_status=%s", test_name.c_str(), sco::toString(opt->results().status).c_str());

  if (plotting)
    plotter->clear();

  collisions.clear();
  found = checkTrajectory(
      collisions, *manager, *state_solver, prob->GetKin()->getJointNames(), getTraj(opt->x(), prob->GetVars()), config);

  const std::string final_test_name = test_name + "_final";
  const ExpectedContactSummary final_summary =
      logAndSummarizeContacts(collisions, final_test_name, "octomap_attached", expected_link_name);

  if (expect_final_collision_free)
  {
    EXPECT_FALSE(found);
    EXPECT_FALSE(final_summary.found_pair);
  }
  CONSOLE_BRIDGE_logDebug((found) ? ("Final trajectory is in collision") : ("Final trajectory is collision free"));
}

TEST_F(CastConstraintOctomapTest, boxes)
{
  runConstraintTest(
      env_, plotter_, false, "CastConstraintOctomapTest.boxes", "boxbot_link", "box_cast_constraint_test.json", true);
}

TEST_F(CastConstraintOctomapTest, boxes_multi_threaded)
{
  runConstraintTest(env_,
                    plotter_,
                    true,
                    "CastConstraintOctomapTest.boxes_multi_threaded",
                    "boxbot_link",
                    "box_cast_constraint_test.json",
                    true);
}

TEST_F(CastConstraintOctomapCylinderTest, cylinder)
{
  runConstraintTest(env_,
                    plotter_,
                    false,
                    "CastConstraintOctomapCylinderTest.cylinder",
                    "boxbot_link",
                    "box_cast_constraint_cylinder_test.json",
                    true);
}

TEST_F(CastConstraintOctomapCylinderTest, cylinder_multi_threaded)
{
  runConstraintTest(env_,
                    plotter_,
                    true,
                    "CastConstraintOctomapCylinderTest.cylinder_multi_threaded",
                    "boxbot_link",
                    "box_cast_constraint_cylinder_test.json",
                    true);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}