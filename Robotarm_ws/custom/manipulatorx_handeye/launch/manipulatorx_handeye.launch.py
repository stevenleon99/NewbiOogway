from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory

def launch_setup(*args, **kwargs):

    # General arguments
    open_manipulator_x_controller_package = get_package_share_directory("open_manipulator_x_controller")
    aruco_ros_package = get_package_share_directory("aruco_ros")
    realsense2_camera_package = get_package_share_directory("realsense2_camera")
    manipulatorx_handeye_package = get_package_share_directory("manipulatorx_handeye")
    
    
    open_manipulator_x_controller_launch = os.path.join(open_manipulator_x_controller_package, 
                                                        "launch", 
                                                        "open_manipulator_x_controller.launch.py")
    aruco_ros_launch = os.path.join(aruco_ros_package, 
                                    "launch", 
                                    "single.launch.py")
    
    realsense2_camera_launch = os.path.join(realsense2_camera_package, 
                                            "launch", 
                                            "rs_launch.py")
    
    xacro_file_path = os.path.join(manipulatorx_handeye_package,
                                   "urdf",
                                   "manipulator_camera.urdf.xacro")
    
    rviz_file = os.path.join(manipulatorx_handeye_package,
                            "rviz",
                            "handeye.rviz")
    
    open_manipulator_x_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(open_manipulator_x_controller_launch)
    )
    
    aruco_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(aruco_ros_launch),
        launch_arguments={
            'marker_size': LaunchConfiguration('marker_size'),
            'marker_id': LaunchConfiguration('marker_id'),
        }.items(),
    )
    
    realsense2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense2_camera_launch),
        launch_arguments={
            'pointcloud.enable': LaunchConfiguration('pointcloud.enable'),
        }.items(),
    )
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file_path
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}
    
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
    )
    
    initial_positions =  {
        "joint1": 0.0,
        "joint2": 0.0,
        "joint3": 0.0,
        "joint4": 0.0,
        "gripper": 0.0,
        "gripper_sub": 0.0
    }
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        arguments=[xacro_file_path],
        parameters=[{'source_list': ['joint_states'], 'use_gui': False, 'zeros': initial_positions}]
    )


    nodes_to_start = [
        open_manipulator_x_controller,
        aruco_ros,
        realsense2_camera,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.028",
            description="Marker size in m.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "marker_size",
            default_value="0.028",
            description="Marker size in m.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'marker_id', 
            default_value='26',
            description='Marker ID. '
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'reference_frame', default_value='camera_color_optical_frame',
            description='Reference frame. '
            'Leave it empty and the pose will be published wrt param parent_name. '
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'pointcloud.enable', default_value='true',
            description='PointCloud2 enabled'
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            'sim_ignition', default_value='false',
            description='enble simulation in ignition'
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


