cmake_minimum_required(VERSION 3.5)
project(ana)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare a C++ executable for the PID node
add_executable(point_cloud_filter_node src/PointCloudFilter.cpp)
add_executable(publish_pcd_node src/ply2pcd.cpp)

# Specify libraries to link a library or executable target against
ament_target_dependencies(
  point_cloud_filter_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  pcl_conversions
)

ament_target_dependencies(
  publish_pcd_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(point_cloud_filter_node 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
)
target_link_libraries(publish_pcd_node
  ${rclcpp_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables and/or libraries
install(TARGETS
  point_cloud_filter_node
  publish_pcd_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories like launch files, configurations, etc.
install(
  DIRECTORY config description launch worlds meshes rtab_maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
