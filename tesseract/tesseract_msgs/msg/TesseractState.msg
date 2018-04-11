# This message contains information about the state, i.e. the positions of its joints and links
sensor_msgs/JointState joint_state

# Joints that may have multiple DOF are specified here
sensor_msgs/MultiDOFJointState multi_dof_joint_state

# Attachable Objects
AttachableObject[] attachable_objects

# Attached Objects
AttachedBodyInfo[] attached_bodies

# Attached objects and even the octomap or collision map can have
# colors associated to them. This array specifies them.
ObjectColor[] object_colors

# Optional, various links can be highlighted
ObjectColor[] highlight_links

# Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
# This is mostly important for handling the attached bodies (whether or not to clear the attached bodies
# before updating it with this message)
bool is_diff
