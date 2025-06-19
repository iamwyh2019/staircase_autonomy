uint8 stair_id
uint8 stair_count

geometry_msgs/Point[] steps_start_p
geometry_msgs/Point[] steps_end_p

float32[] steps_radii
float32[] steps_theta

float32[] start_z_covariances
float32[] end_z_covariances

float32[] steps_radii_covariances
float32[] steps_theta_covariances

float32 stair_height
float32 stair_depth
float32 stair_width
float32 stair_curvature

string robot_id
string frame_id