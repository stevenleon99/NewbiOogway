cmake_minimum_required(VERSION 3.8)
project(manipulatorx_ign)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(open_manipulator_x_description REQUIRED)
find_package(realsense2_description REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

add_library( test_manipulatorx_ign SHARED src/test_manipulatorx_ign.cpp )
ament_target_dependencies( test_manipulatorx_ign std_msgs sensor_msgs rclcpp)

add_executable(test_manipulatorx_ign_node src/test_manipulatorx_ign_node.cpp)
target_link_libraries(test_manipulatorx_ign_node test_manipulatorx_ign)
ament_target_dependencies(test_manipulatorx_ign_node std_msgs sensor_msgs rclcpp)


install(DIRECTORY urdf 
                  launch 
                  worlds
                  rviz 
                  DESTINATION share/${PROJECT_NAME})

install(TARGETS test_manipulatorx_ign LIBRARY DESTINATION lib)
install(TARGETS test_manipulatorx_ign_node DESTINATION lib/${PROJECT_NAME})

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
