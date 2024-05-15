cmake_minimum_required(VERSION 3.8)
project(arm_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

add_library(arm_subscriber SHARED src/arm_subscriber.cpp)
ament_target_dependencies(arm_subscriber rclcpp sensor_msgs open_manipulator_msgs)

add_executable(test_armsubcriber src/test_armsubscriber.cpp)
target_link_libraries(test_armsubcriber arm_subscriber)
ament_target_dependencies(test_armsubcriber rclcpp)


install( TARGETS arm_subscriber LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/)
install( TARGETS test_armsubcriber DESTINATION lib/${PROJECT_NAME})

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

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(open_manipulator_msgs)
ament_export_libraries(arm_subscriber)
ament_package()
