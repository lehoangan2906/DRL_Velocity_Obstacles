cmake_minimum_required(VERSION 3.5)
project(drl_vo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules (scripts)
install(
  PROGRAMS
    src/cmd_vel_pub.py
    src/cnn_data_pub.py
    src/drl_vo_inference.py 
    src/drl_vo_train.py 
    src/dynamic_goal_publisher.py
    src/track_data_pub.py
    src/track_utils/Tracking_Face/app3.py
    src/track_utils/Tracking_Face/find_distance.py
    src/track_utils/Tracking_Face/get_newdata.py
    src/track_utils/Tracking_Face/test.py
    src/track_utils/Tracking_Face/testvideo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the entire track_utils directory (including Python files)
install(DIRECTORY src/track_utils
  DESTINATION lib/${PROJECT_NAME}/track_utils
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING PATTERN "*.py"
)

# Install tracking face-related resources (non-Python files)
install(DIRECTORY src/track_utils/Tracking_Face/yolov8_tracking
  DESTINATION share/${PROJECT_NAME}/track_utils/Tracking_Face/yolov8_tracking
)

# Install other required directories (models and configuration files)
install(DIRECTORY src/track_utils/buffalo_sc
  DESTINATION share/${PROJECT_NAME}/track_utils/buffalo_sc
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Mark package.xml for installation
ament_package()
