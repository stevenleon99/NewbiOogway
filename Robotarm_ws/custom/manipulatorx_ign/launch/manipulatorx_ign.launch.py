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

def launch_setup(context, *args, **kwargs):

    
    # General arguments
    manipulatorx_ign_package = get_package_share_directory("manipulatorx_ign")
    xacro_file_path = os.path.join(manipulatorx_ign_package, "urdf", "manipulator_camera.urdf.xacro")
    rviz_file_path = os.path.join(manipulatorx_ign_package, "rviz", "manipulatorx_viz.rviz")
    world_file_path = os.path.join(manipulatorx_ign_package, "worlds", "mx_camera.sdf")
    realsense2_description_package = get_package_share_directory("realsense2_description")
    manipulatorx_description_package = get_package_share_directory("open_manipulator_x_description")
    initial_joint_controllers = os.path.join(manipulatorx_ign_package, "config", "mx_controllers.yaml")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    
    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(realsense2_description_package, 'meshes'), ':' +
            os.path.join(manipulatorx_description_package, 'meshes'), ':' +
            os.path.join(manipulatorx_description_package, 'worlds'), ':' +
            os.path.join(manipulatorx_description_package, 'config')])
    
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file_path,
            " ",
            "simulation_controllers:=",
            initial_joint_controllers,
            " ",
            "sim_ignition:=true"
        ]
    )
    
    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
        remappings= [('/joint_states', '/world/empty/model/mx/joint_state')] 
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_file_path],
        # condition=IfCondition(launch_rviz),
    )
    
    ###############################################################
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    joint_group_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_velocities', '--controller-manager', '/controller_manager'],
    )
    
    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )
    
    ###############################################################
    
    
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
        parameters=[{'source_list': ['joint_states'], 'use_gui': False, 'zeros': initial_positions}]
    )
    
    ros_gz_sim = get_package_share_directory("ros_gz_sim")
    ignition_launch_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": [f"-r {world_file_path}"],
        }.items(),
    )

    
    # Ignition nodes
    ignition_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "mx",
            "-allow_renaming",
            "true",
        ],
    )

    state_ign_bridge = Node(
        package='ros_ign_bridge', executable='parameter_bridge',
        arguments=['/world/empty/model/mx/joint_state@sensor_msgs/msg/JointState@ignition.msgs.Model', 
                   '/image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                   '/camera_depth/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'],
        output='screen'
    )
    
    rqt_reconfigure_node = Node(
        package='rqt_reconfigure',
        executable='rqt_reconfigure',
        name='rqt_reconfigure_node'
    )


    nodes_to_start = [
        ign_resource_path,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        joint_group_velocity_controller_spawner,
        joint_state_publisher_node,
        # rviz_node,
        ignition_spawn_entity,
        ignition_launch_description,
        state_ign_bridge,
        # rqt_reconfigure_node
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Robot controller to start.",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


