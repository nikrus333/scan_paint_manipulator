cmake_minimum_required(VERSION 3.8)
project(mcx_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(DIRECTORY license
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/action_client.py
  src/client.py
  src/ros2api.py
  src/execute_trajectory_action.py
  src/laser_trajectory_action.py
  src/test_paint_squad.py
  DESTINATION lib/${PROJECT_NAME}
)

install (
  PROGRAMS
  libs/__init__.py
  libs/motion_program.py
  libs/robot_command.py
  libs/motionSL_pb2.py
  libs/system_defs.py
  libs/motionSL_hash.json
  DESTINATION lib/${PROJECT_NAME}/libs
)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MoveSrv.srv"
)
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
