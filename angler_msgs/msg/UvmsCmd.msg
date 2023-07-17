# A system velocity command for an underwater vehicle manipulator system.
# This is comprised of the base velocities intended for the AUV and the
# joint velocities for the manipulator.

# The desired velocity command for the AUV
geometry_msgs/Twist base

# The desired velocity command for the manipulator
float64[] manipulator
