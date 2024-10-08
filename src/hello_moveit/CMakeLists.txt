cmake_minimum_required(VERSION 3.8)
project(hello_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(m0609_interfaces REQUIRED)

add_executable(hello_moveit src/hello_moveit.cpp)
add_executable(get_eef_pos src/get_eef_pos.cpp)
add_executable(key_controll src/key_controll.cpp)
add_executable(movepose_test src/movepose_test.cpp)
add_executable(movepose_test_2 src/movepose_test_2.cpp)
add_executable(service_test src/pose_array_service_server_test.cpp)

target_include_directories(hello_moveit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(hello_moveit PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  hello_moveit
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  get_eef_pos
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  get_eef_pos
  "moveit_ros_planning_interface"
  "rclcpp"
)

ament_target_dependencies(
  key_controll
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)


ament_target_dependencies(
  movepose_test
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)


ament_target_dependencies(
  movepose_test_2
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)


ament_target_dependencies(
  service_test
  "m0609_interfaces"
  "moveit_ros_planning_interface"
  "rclcpp"
  "tf2"
  "tf2_geometry_msgs"
)

install(TARGETS hello_moveit get_eef_pos key_controll movepose_test movepose_test_2 service_test
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
