cmake_minimum_required(VERSION 3.8)
project(arm_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(open_manipulator_msgs REQUIRED)

include_directories(include)

add_library(movearm SHARED src/arm_movejoint.cpp src/arm_movecart.cpp src/arm_movetool.cpp)
ament_target_dependencies(movearm rclcpp open_manipulator_msgs)


add_executable(test_movejoint src/test_movejoint.cpp)
target_link_libraries(test_movejoint movearm)
ament_target_dependencies(test_movejoint rclcpp open_manipulator_msgs)

add_executable(test_movetool src/test_movetool.cpp)
target_link_libraries(test_movetool movearm)
ament_target_dependencies(test_movetool rclcpp open_manipulator_msgs)

add_executable(test_movecart src/test_movecart.cpp)
target_link_libraries(test_movecart movearm)
ament_target_dependencies(test_movecart 
                            rclcpp 
                            open_manipulator_msgs)

add_executable(test_pnp src/test_pnp.cpp)
target_link_libraries(test_pnp movearm)
ament_target_dependencies(test_pnp rclcpp open_manipulator_msgs)

install( TARGETS movearm LIBRARY DESTINATION lib)
install( TARGETS test_movejoint test_movetool test_movecart test_pnp DESTINATION lib/${PROJECT_NAME})


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
ament_export_dependencies(open_manipulator_msgs)
ament_export_dependencies(arm_subscriber)
ament_export_libraries(movearm)
ament_package()
