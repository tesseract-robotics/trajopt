#include <pluginlib/class_list_macros.h>
#include <trajopt_moveit/collision_detector_allocator_bullet.h>

namespace collision_detection
{
const std::string CollisionDetectorAllocatorBullet::NAME_ = "BULLET";

bool CollisionBulletPluginLoader::initialize(const planning_scene::PlanningScenePtr& scene, bool exclusive) const
{
  scene->setActiveCollisionDetector(CollisionDetectorAllocatorBullet::create(), exclusive);
  return true;
}
}

PLUGINLIB_EXPORT_CLASS(collision_detection::CollisionBulletPluginLoader, collision_detection::CollisionPlugin)
