# Holds the averaged properties of a single, merged staircase estimate,
# suitable for display in a UI or for high-level logic. This avoids
# conflict with the internal C++ 'StaircaseInfo' struct.

# A unique identifier for this staircase, consistent across messages.
int32 stair_id

# The list of robot names that have contributed to this staircase estimate.
string[] contributing_robots

# The average geometric properties of the staircase.
float32 stair_width
float32 stair_depth
float32 stair_height

# The average yaw of the first and last steps.
float32 start_yaw
float32 end_yaw