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

# Compiler options for GNU and Clang compilers to enable all warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# If CMake version >= 3.18, set CUDA architechture and policy
if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.18")
  cmake_policy(SET CMP0104 NEW)
  set(CMAKE_CUDA_ARCHITECTURES 75)  # CUDA architechture, 75 == 7.5 for 2080 super
endif()

# Locate and load settings for required dependencies packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)  # Ensure this line is present
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(pcl_conversions REQUIRED)
find_package(CUDA REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories for header files of directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${nav2_costmap_2d_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/cuda
)

# Link directories for libraries of dependencies
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Declare executables for nodes in project
add_executable(depth_live_filter_node src/nodes/depth_live_filter.cpp)
add_executable(publish_pcd_node src/nodes/ply2pcd.cpp)
add_executable(occupancy_grid_subscriber_node src/nodes/occupancy_grid_subscriber.cpp)

# Enable CUDA language support and define CUDA library
enable_language(CUDA)  # Enable CUDA language support (for CMake >= 3.18)
add_library(cuda_operations src/cuda/pointcloud_processing.cu)
set_target_properties(cuda_operations PROPERTIES CUDA_SEPARABLE_COMPILATION ON)
target_compile_options(cuda_operations PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler -Wno-line>)
# target_compile_options(cuda_operations PRIVATE $<$<COMPILE_LANGUAGE:CUDA>:-Xcompiler -Wno-line>)


# Specify libraries to link with each executable:
ament_target_dependencies(depth_live_filter_node
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  pcl_conversions
)
ament_target_dependencies(publish_pcd_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  nav2_costmap_2d
)
ament_target_dependencies(occupancy_grid_subscriber_node
  rclcpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
  pcl_conversions
  nav2_costmap_2d
)


# If building tests, find and load test dependencies
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Install executables and/or libraries
install(TARGETS
  depth_live_filter_node
  publish_pcd_node
  occupancy_grid_subscriber_node
  cuda_operations
  DESTINATION lib/${PROJECT_NAME}
)

# Install other directories like launch files, configurations, etc.
install(DIRECTORY 
  config 
  description 
  launch 
  worlds 
  meshes 
  rtab_maps
  other
  DESTINATION share/${PROJECT_NAME}
)

# Link libraries to executables
target_link_libraries(depth_live_filter_node 
  ${PCL_LIBRARIES} 
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}  # Link CUDA libraries
  cuda_operations  # Link your CUDA library
)
target_link_libraries(publish_pcd_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  cuda_operations
  ${CUDA_LIBRARIES}
  ${CUDA_cudart_LIBRARY}
  ${nav2_costmap_2d_LIBRARIES}
)
target_link_libraries(occupancy_grid_subscriber_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${PCL_LIBRARIES}
  cuda_operations
  ${CUDA_LIBRARIES}
  ${CUDA_cudart_LIBRARY}
  ${nav2_costmap_2d_LIBRARIES}
)
# target_link_libraries(PointCloudFilterCuda ${CUDA_LIBRARIES})

# Finalize the ament package
ament_package()
