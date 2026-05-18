cmake_minimum_required(VERSION 3.8)
project(robotic_arm_app)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Dependencies ───────────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ── Executable 1: moveit_interface (voice → MoveIt) ───────────────────────
add_executable(moveit_interface src/moveit_interface.cpp)
ament_target_dependencies(moveit_interface
  rclcpp
  std_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

# ── Executable 2: box_picker (vision → pick sequence) ─────────────────────
add_executable(box_gripper src/box_gripper.cpp)
ament_target_dependencies(box_gripper
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

add_executable(draw_rectangle src/draw_rectangle.cpp)
ament_target_dependencies(draw_rectangle
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# ── Install C++ executables ────────────────────────────────────────────────
install(TARGETS
  moveit_interface
  box_gripper
  draw_rectangle
  DESTINATION lib/${PROJECT_NAME}
)

# ── Install launch files ───────────────────────────────────────────────────
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# ── Python package + scripts ───────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  robotic_arm_app/voice_commander.py
  robotic_arm_app/voice_to_moveit.py
  robotic_arm_app/servo_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

# ── Tests ──────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()