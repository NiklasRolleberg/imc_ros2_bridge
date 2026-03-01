cmake_minimum_required(VERSION 3.8)
project(imc_ros_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs)
find_package(rosidl_default_generators REQUIRED)

# Add messages here!!
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PlanControl.msg"
  "msg/PlanControlState.msg"
  "msg/VehicleState.msg"
  "msg/EstimatedState.msg"
  "msg/RemoteState.msg"
  "msg/SonarData.msg"
  "msg/DesiredHeading.msg"
  "msg/DesiredHeadingRate.msg"
  "msg/DesiredPitch.msg"
  "msg/DesiredRoll.msg"
  "msg/DesiredSpeed.msg"
  "msg/DesiredZ.msg"
  "msg/PlanDB.msg"
  "msg/PlanSpecification.msg"
  "msg/PlanDBInformation.msg"
  "msg/PlanDBState.msg"
  "msg/PlanManeuver.msg"
  "msg/Maneuver.msg"
  DEPENDENCIES geometry_msgs sensor_msgs std_msgs nav_msgs
 )
  

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(PROGRAMS scripts/imc_ros2_bridge.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/pyimc_generated DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY scripts/pyimclsts DESTINATION lib/${PROJECT_NAME})


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
