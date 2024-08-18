cmake_minimum_required(VERSION 3.8)
project(spot_control_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(urdf REQUIRED)

include_directories(include/spot_control_pkg)

add_executable(spot_control_node src/spot_control_node.cpp src/Kinematics.cpp)  # Kinematics.cpp 추가
ament_target_dependencies(spot_control_node rclcpp sensor_msgs geometry_msgs)

install(TARGETS
  spot_control_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch urdf mesh meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
