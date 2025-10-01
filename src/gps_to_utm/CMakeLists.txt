cmake_minimum_required(VERSION 3.8)
project(gps_to_utm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# f9r_to_utm executable
add_executable(f9r_to_utm src/f9r_to_utm.cpp)
ament_target_dependencies(f9r_to_utm rclcpp sensor_msgs geometry_msgs)
target_include_directories(f9r_to_utm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# f9p_to_utm executable
add_executable(f9p_to_utm src/f9p_to_utm.cpp)
ament_target_dependencies(f9p_to_utm rclcpp sensor_msgs geometry_msgs)
target_include_directories(f9p_to_utm PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# tf_gps_csv_node executable
add_executable(tf_gps_csv_node src/tf_gps_csv.cpp)
ament_target_dependencies(tf_gps_csv_node rclcpp nav_msgs std_msgs geometry_msgs visualization_msgs tf2_ros tf2_geometry_msgs)
target_include_directories(tf_gps_csv_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# azimuth_angle_calculator_node executable
add_executable(azimuth_angle_calculator_node src/azimuth_angle_calculator.cpp)
ament_target_dependencies(azimuth_angle_calculator_node rclcpp sensor_msgs std_msgs)
target_include_directories(azimuth_angle_calculator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install executables
install(TARGETS
  f9r_to_utm
  f9p_to_utm
  tf_gps_csv_node
  azimuth_angle_calculator_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header
install(
  DIRECTORY include/
  DESTINATION include
)

# Install Python scripts
install(PROGRAMS
  src/f9r_to_csv.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config files
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()