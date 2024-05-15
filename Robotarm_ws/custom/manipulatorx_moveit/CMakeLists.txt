cmake_minimum_required(VERSION 3.22)
project(manipulatorx_moveit)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread serialization)
find_package(moveit_common REQUIRED)
moveit_package()


find_package(rclcpp REQUIRED)
find_package(class_loader REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)



include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN_INCLUDE_DIR})
include_directories( ${catkin_INCLUDE_DIRS} )

link_directories(${Boost_LIBRARY_DIRS})

add_library( manipulatorx_context SHARED src/manipulatorx_context.cpp )
ament_target_dependencies( manipulatorx_context rclcpp moveit_core moveit_msgs )
target_link_libraries( manipulatorx_context ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

add_library( manipulatorx_plugin SHARED src/manipulatorx_plugin.cpp)
ament_target_dependencies( manipulatorx_plugin rclcpp class_loader moveit_core moveit_msgs pluginlib )
target_link_libraries( manipulatorx_plugin manipulatorx_context ${MOVEIT_LIB_NAME} ${Boost_LIBRARIES})

install(TARGETS manipulatorx_context manipulatorx_plugin 
  EXPORT asbr_planners_manipulatorxTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )



install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY srdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})


pluginlib_export_plugin_description_file(moveit_core manipulatorx_plugin.xml)
ament_export_targets( asbr_planners_manipulatorxTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_core)
ament_package()