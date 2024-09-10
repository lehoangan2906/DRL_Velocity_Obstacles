cmake_minimum_required(VERSION 3.5)
project(track_ped_msgs)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Include directories
include_directories(include)

# Generate messages
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrackedPerson.msg"
  "msg/TrackedPersons.msg"
  DEPENDENCIES std_msgs
)

ament_package()
