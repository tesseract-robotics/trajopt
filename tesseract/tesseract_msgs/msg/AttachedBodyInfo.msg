# The attachable object name
string object_name

# The body will be attached with a fixed joint to this link
string parent_link_name

# The transform between the parent link and object
geometry_msgs/Pose transform

# The set of links that the attached body is allowed to touch
# by default - the link_name is already considered by default
string[] touch_links

# Adds the object to the environment. If the body previously existed, it is replaced.
byte ADD=0

# Removes the body from the environment entirely (everything that matches the specified id)
byte REMOVE=1

# If an body already exists in the scene, new poses can be sent if solely moving the object is desired
byte MOVE=2

# Operation to be performed
byte operation
