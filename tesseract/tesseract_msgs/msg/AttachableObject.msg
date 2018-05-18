# the name of the object
string name

# the objects visual geometry
VisualGeometry visual

# the objects collision geometry
CollisionGeometry collision

# The weight of the attached object, if known
geometry_msgs/Inertia inertia

# Adds the object. If the object previously existed, it is replaced.
byte ADD=0

# Removes the object (everything that matches the specified id)
byte REMOVE=1

# Operation to be performed
byte operation
