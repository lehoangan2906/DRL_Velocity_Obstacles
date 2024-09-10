# Bounding box coordinates (in pixel coordinates)
int32 bbox_upper_left_x
int32 bbox_upper_left_y
int32 bbox_bottom_right_x
int32 bbox_bottom_right_y

# Unique ID for the pedestrian
int32 id 

# Distance from the camera to the pedestrian (depth in meters)
float32 depth

# Angle relative to the camera's perpendicular bisector (in radians)
float32 angle 

# Pedestrian velocity in the X direction (m/s)
float32 velocity_x

# Pedestrian velocity in the Z direction (m/s)
float32 velocity_z