cmake_minimum_required(VERSION 3.8)
project(spin_around)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)  # Ensure Eigen3 is properly found

find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include ${EIGEN3_INCLUDE_DIR})

# Add library
add_library(spin SHARED src/spin.cpp)
target_link_libraries(spin Eigen3::Eigen)  # Link against Eigen3 using modern CMake target
ament_target_dependencies(spin rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs)

# Add executable
add_executable(spin_node src/spin_node.cpp)
target_link_libraries(spin_node spin)
ament_target_dependencies(spin_node rclcpp tf2 tf2_ros tf2_geometry_msgs geometry_msgs)


install( TARGETS spin LIBRARY DESTINATION lib )
install( TARGETS spin_node DESTINATION lib/${PROJECT_NAME} )


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
