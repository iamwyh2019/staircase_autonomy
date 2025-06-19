# A list of all staircases currently being tracked by the client node.
# This message is intended for UIs or other high-level consumers.
std_msgs/Header header

# An array containing the details for each tracked staircase.
staircase_msgs/SingleStaircaseDetails[] staircases