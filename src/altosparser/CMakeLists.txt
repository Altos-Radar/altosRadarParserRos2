cmake_minimum_required(VERSION 3.8)
set(CMAKE_POLICY_DEFAULT_CMP0144 NEW CACHE STRING "CMake policy CMP0144 default")
project(altosparser)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Build executable
add_executable(altosparser src/altosparser.cpp)
ament_target_dependencies(altosparser
  rclcpp
  ament_index_cpp
  pcl_conversions
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  visualization_msgs
  geometry_msgs
)
target_link_libraries(altosparser ${PCL_LIBRARIES})

# Install
install(TARGETS altosparser
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY param launch data
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()