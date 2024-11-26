cmake_minimum_required(VERSION 3.8)
project(hybrid_multi_sensor_fusion_for_360_object_detection_and_localization)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Add subdirectories for the two packages
add_subdirectory(ros2_depth_map_detection_localization_cpp)
add_subdirectory(ros2_lidar_camera_fusion_with_detection_cpp)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files from subpackages
install(DIRECTORY
  ros2_depth_map_detection_localization_cpp/launch
  DESTINATION share/${PROJECT_NAME}/ros2_depth_map_detection_localization_cpp/launch
)

install(DIRECTORY
  ros2_lidar_camera_fusion_with_detection_cpp/launch
  DESTINATION share/${PROJECT_NAME}/ros2_lidar_camera_fusion_with_detection_cpp/launch
)

# Declare the package
ament_package()
