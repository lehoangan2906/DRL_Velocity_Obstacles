cmake_minimum_required(VERSION 3.5)
project(drl_vo)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install Python modules
install(
  PROGRAMS
    src/cmd_vel_pub.py
    src/cnn_data_pub.py
    src/drl_vo_inference.py 
    src/drl_vo_train.py 
    src/track_data_pub.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Mark package.xml for installation
ament_package()
