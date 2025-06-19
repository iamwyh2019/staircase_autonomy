
uint8 stair_count # Number of detected stairs in the measurement, used to index the location of stairs from the points below
bool is_ascending # Used for visualization scripts to display both ascending and descending stairs

geometry_msgs/Point[] steps_start_p #Starting points of each line in detected steps
geometry_msgs/Point[] steps_end_p # Ending points of each line in detected steps
float32[] step_lengths # Step lengths, can be computed by using norm of the points above. Extra data
float32[] step_radii # Line radius in polar form
float32[] step_angles # Line angle in polar form
float32[] r_r_covariance
float32[] r_t_covariance
float32[] t_t_covariance
float32[] z_covariance

geometry_msgs/Transform robot_transform # Pose of the robot when detected staircase. Measuements are local, and need to be transformed by this pose to transform to the corresponding frame_id
string frame_id # Target frame_id of the staircase after transforming