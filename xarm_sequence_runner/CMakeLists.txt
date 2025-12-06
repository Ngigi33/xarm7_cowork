cmake_minimum_required(VERSION 3.8)
project(xarm_sequence_runner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xarm_msgs REQUIRED)

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

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(xarm_msgs REQUIRED)


add_executable(task_sequencer src/task_sequencer.cpp)
add_executable(emergency_stop_subscriber src/emergency_stop_subscriber.cpp)
add_executable(process_launcher src/process_launcher.cpp)
ament_target_dependencies(task_sequencer rclcpp std_msgs xarm_msgs)
ament_target_dependencies(emergency_stop_subscriber rclcpp std_msgs xarm_msgs)
ament_target_dependencies(process_launcher rclcpp xarm_msgs)

install(TARGETS
  task_sequencer
  emergency_stop_subscriber
  process_launcher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
