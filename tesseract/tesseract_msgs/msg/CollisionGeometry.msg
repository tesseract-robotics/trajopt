# solid geometric primitives
shape_msgs/SolidPrimitive[] primitives
geometry_msgs/Pose[] primitive_poses
std_msgs/Int32[] primitive_collision_object_types
std_msgs/ColorRGBA[] primitive_colors # The color is optional

# meshes
shape_msgs/Mesh[] meshes
geometry_msgs/Pose[] mesh_poses
std_msgs/Int32[] mesh_collision_object_types
std_msgs/ColorRGBA[] mesh_colors # The color is optional

# bounding planes (equation is specified, but the plane can be oriented using an additional pose)
shape_msgs/Plane[] planes
geometry_msgs/Pose[] plane_poses
std_msgs/Int32[] plane_collision_object_types
std_msgs/ColorRGBA[] plane_colors # The color is optional

# The octomap
octomap_msgs/Octomap[] octomaps
geometry_msgs/Pose[] octomap_poses
std_msgs/Int32[] octomap_collision_object_types
std_msgs/ColorRGBA[] octomap_colors # The color is optional
