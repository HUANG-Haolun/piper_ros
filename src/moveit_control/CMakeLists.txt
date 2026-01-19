cmake_minimum_required(VERSION 3.8)
project(moveit_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
find_package(moveit_ros_planning REQUIRED)          # often pulls in robot_model_loader etc.
find_package(moveit_core REQUIRED)


add_executable(moveit_control src/moveit_control.cpp)
ament_target_dependencies(
  moveit_control
  "moveit_ros_planning_interface"
  "rclcpp"
)


add_executable(workspace_visualizer src/workspace_visualizer.cpp)
ament_target_dependencies(
  workspace_visualizer
  "moveit_ros_planning"
  "moveit_ros_planning_interface"
  "moveit_core"
  "visualization_msgs"
  "Eigen3"
  "rclcpp"
)

target_include_directories(moveit_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(moveit_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17


install(TARGETS 
  moveit_control
  workspace_visualizer
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
