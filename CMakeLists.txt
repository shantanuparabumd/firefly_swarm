cmake_minimum_required(VERSION 3.8)
project(firefly_swarm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_cmake_gtest)
find_package(xacro REQUIRED)
find_package(ros2_control REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(master src/main.cpp)
ament_target_dependencies(master tf2 tf2_ros rclcpp cv_bridge std_msgs sensor_msgs geometry_msgs nav_msgs)
add_library(robot_library src/robot.cpp src/master.cpp)
target_include_directories(robot_library PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
ament_target_dependencies(robot_library tf2 tf2_ros  cv_bridge rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

target_link_directories(master PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(master robot_library)


install(TARGETS
  master

  # slave
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  worlds
  urdf
  models
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY include/firefly_swarm
  DESTINATION include
)


# install(
#   PROGRAMS
#   DESTINATION lib/${PROJECT_NAME}
# )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  
  set(TESTFILES
    test/test.cpp)

  ament_add_gtest(test_${PROJECT_NAME} ${TESTFILES})

  ament_target_dependencies(test_${PROJECT_NAME} tf2 tf2_ros rclcpp cv_bridge std_msgs sensor_msgs geometry_msgs nav_msgs)
  target_link_libraries(test_${PROJECT_NAME} robot_library)

  install(TARGETS
    test_${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME})

  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
