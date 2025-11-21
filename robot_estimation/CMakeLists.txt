cmake_minimum_required(VERSION 3.8)
project(robot_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

# Lowpass IMU node
add_executable(lowpass_imu_node src/lowpass_imu_node.cpp)
ament_target_dependencies(lowpass_imu_node
  rclcpp
  sensor_msgs
)

add_executable(imu_bias_correction_node src/imu_bias_correction_node.cpp)
ament_target_dependencies(imu_bias_correction_node
  rclcpp
  sensor_msgs
)

add_executable(complementary_orientation_node src/complementary_orientation_node.cpp)
ament_target_dependencies(complementary_orientation_node
  rclcpp
  sensor_msgs
  nav_msgs
  tf2
  tf2_ros
  geometry_msgs
)


add_executable(motor_rpm_node src/motor_rpm_node.cpp)
ament_target_dependencies(motor_rpm_node
  rclcpp
  std_msgs
)

add_executable(cmd_vel_to_rpm src/cmd_vel_to_rpm.cpp)
ament_target_dependencies(cmd_vel_to_rpm rclcpp geometry_msgs std_msgs)

add_executable(wheel_odometry_node src/wheel_odometry_node.cpp)
ament_target_dependencies(wheel_odometry_node
    rclcpp std_msgs nav_msgs geometry_msgs tf2 tf2_ros
)

install(TARGETS
  imu_bias_correction_node
  lowpass_imu_node
  complementary_orientation_node
  motor_rpm_node
  cmd_vel_to_rpm
  wheel_odometry_node
  DESTINATION lib/${PROJECT_NAME})

  install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
