import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ur_moveit_config.launch_common import load_yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def launch_setup(context, *args, **kwargs):

    # General arguments
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    manipulatorx_ign_package = get_package_share_directory("manipulatorx_moveit")
    xacro_file_path = os.path.join(manipulatorx_ign_package, "urdf", "open_manipulator_x.urdf.xacro")
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "moveit.rviz"]
    )
    ros2_controllers_yaml_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "ros2_controllers.yaml"]
    )
    
    
    # robot description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            xacro_file_path,
            " ",
            "sim_ignition:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, 
                    robot_description],
    )
    
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": LaunchConfiguration("allow_trajectory_execution"),
        # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
        "capabilities": ParameterValue(
            LaunchConfiguration("capabilities"), value_type=str
        ),
        "disable_capabilities": ParameterValue(
            LaunchConfiguration("disable_capabilities"), value_type=str
        ),
        # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "monitor_dynamics": False,
    }
    
    
    # MoveIt Configuration
    srdf_path = PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", "open_manipulator_x.srdf"]).perform(context)
    robot_description_semantic_content = open(srdf_path).read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}


    robot_description_kinematics = load_yaml("manipulatorx_moveit", "config/kinematics.yaml")
    kinematic_description = {"robot_description_kinematics": robot_description_kinematics}


    # Planning Configuration
    # planning_pipeline_config = {
    #     "move_group": {
    #         "planning_plugin": "ompl_interface/OMPLPlanner",
    #         "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
    #         "start_state_max_bounds_error": 0.1,
    #     }
    # }
    # ompl_planning_yaml = load_yaml("manipulatorx_moveit", "config/ompl_planning.yaml")
    # planning_pipeline_config["move_group"].update(ompl_planning_yaml)
    
    # custom planning configuration
    planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "manipulatorx_moveit/ASBRPlanner",
            "start_state_max_bounds_error": 0.1,
        }
    }
    asbr_planning_yaml = load_yaml("manipulatorx_moveit", "config/custom_planning.yaml")
    planning_pipeline_config["move_group"].update(asbr_planning_yaml)

    
    controller_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_content,
            ros2_controllers_yaml_file,
        ],
    )
    
    joint_state_broadcaster_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen"
    )
    
    open_manipulator_x_controller_node = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["open_manipulator_x_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )
    
    
    # Trajectory Execution Configuration
    controllers_yaml = load_yaml("manipulatorx_moveit", "config/moveit_controllers.yaml")
    moveit_controllers = controllers_yaml


    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            move_group_configuration,
            {"use_sim_time": use_sim_time},
        ],
    )
    

    # rviz with moveit configuration
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            kinematic_description,
        ],
    )

    # nodes_to_start = [move_group_node, rviz_node ]
    nodes_to_start = [robot_state_publisher_node,
                      controller_node,
                      joint_state_broadcaster_node,
                      open_manipulator_x_controller_node,
                      move_group_node,
                      rviz_node]
    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="manipulatorx_moveit",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Make MoveIt to use simulation time. This is needed for the trajectory planing in simulation.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    # load non-default MoveGroup capabilities (space separated)
    declared_arguments.append(
        DeclareLaunchArgument("capabilities", default_value=""))
    # inhibit these default MoveGroup capabilities (space separated)
    declared_arguments.append(
        DeclareLaunchArgument("disable_capabilities", default_value=""))
    declared_arguments.append(
        DeclareBooleanLaunchArg("allow_trajectory_execution", default_value=True)
    )
    

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])