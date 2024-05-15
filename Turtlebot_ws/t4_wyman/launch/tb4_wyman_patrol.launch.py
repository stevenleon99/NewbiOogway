# Copyright 2023 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Roni Kreinin (rkreinin@clearpathrobotics.com)
# @Modified by: Zhiyuan Zhu - Apr 2024

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from launch.launch_context import LaunchContext

from nav2_common.launch import RewrittenYaml

import os

from pathlib import Path


# ARGUMENTS = [
#     # DeclareLaunchArgument('namespace', default_value='',
#     #                       description='Robot namespace'),
#     # DeclareLaunchArgument('rviz', default_value='false',
#     #                       choices=['true', 'false'], description='Start rviz.'),
#     # DeclareLaunchArgument('world', default_value='warehouse',
#     #                       description='Ignition World'),
#     # DeclareLaunchArgument('model', default_value='standard',
#     #                       choices=['standard', 'lite'],
#     #                       description='Turtlebot4 Model'),
# ]

def generate_launch_description():
    # Directories
    pkg_turtlebot4_ignition_bringup = get_package_share_directory(
        'turtlebot4_ignition_bringup')
    pkg_turtlebot4_ignition_gui_plugins = get_package_share_directory(
        'turtlebot4_ignition_gui_plugins')
    pkg_turtlebot4_description = get_package_share_directory(
        'turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory(
        'irobot_create_description')
    pkg_irobot_create_ignition_bringup = get_package_share_directory(
        'irobot_create_ignition_bringup')
    pkg_irobot_create_ignition_plugins = get_package_share_directory(
        'irobot_create_ignition_plugins')
    pkg_ros_ign_gazebo = get_package_share_directory(
        'ros_ign_gazebo')
    

    # Paths
    # Set ignition resource path
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_bringup, 'worlds'), ':' +
            os.path.join(pkg_irobot_create_ignition_bringup, 'worlds'), ':' +
            str(Path(pkg_turtlebot4_description).parent.resolve()), ':' +
            str(Path(pkg_irobot_create_description).parent.resolve())])

    ign_gui_plugin_path = SetEnvironmentVariable(
        name='IGN_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_turtlebot4_ignition_gui_plugins, 'lib'), ':' +
            os.path.join(pkg_irobot_create_ignition_plugins, 'lib')])

    # Paths
    ign_gazebo_launch = PathJoinSubstitution(
        [pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py'])

    wyman_world = os.path.join(
        get_package_share_directory("t4_wyman"), "worlds", "wyman.sdf"
    )

    # Ignition gazebo
    ignition_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ign_gazebo_launch]),
        launch_arguments=[
            ('ign_args', [
                            wyman_world,
                          ' -v 4',
                          ' --gui-config ',
                          PathJoinSubstitution(
                            [pkg_turtlebot4_ignition_bringup,
                             'gui',
                             "standard",
                             'gui.config'])])
        ]
    )

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
                        name='clock_bridge',
                        output='screen',
                        arguments=[
                            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
                        ])
    
    robot_spawn_launch = PathJoinSubstitution(
        [pkg_turtlebot4_ignition_bringup, 'launch', 'turtlebot4_spawn.launch.py'])
    
    wyman_map = os.path.join(
        get_package_share_directory("t4_wyman"), "maps", "wyman.yaml"
    )
    
    nav2_yaml = os.path.join(get_package_share_directory("t4_wyman"), "config", "nav2.yaml")

    move_bt = os.path.join(
        get_package_share_directory("t4_wyman"), "config", "move.xml"
    )

    context = LaunchContext()
    param_substitutions = {}

    param_substitutions.update(
        {'default_nav_to_pose_bt_xml': move_bt}
    )


    configured_params = RewrittenYaml(
        source_file=nav2_yaml,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    new_yaml = configured_params.perform(context)



    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('namespace', ''),
            ('rviz', 'true'),
            ('slam', 'false'),
            ('x', '0.0'),
            ('y', '0.0'),
            ('z', '0.0'),
            ('yaw', '0.0'),
            ('localization', 'true'),
            ('nav2', 'true'),
            ('map', wyman_map),
            ('params_file', new_yaml),
            ]
    )

    # ld = LaunchDescription(ARGUMENTS)
    # ld.add_action(ign_resource_path)
    # ld.add_action(ign_gui_plugin_path)
    # ld.add_action(ignition_gazebo)
    # ld.add_action(clock_bridge)
    # ld.add_action(robot_spawn)
    # return ld

    return LaunchDescription(
        [
            ign_resource_path,
            ign_gui_plugin_path,
            ignition_gazebo,
            clock_bridge,
            robot_spawn,
        ]
    )