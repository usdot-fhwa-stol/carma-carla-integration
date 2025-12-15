#!/usr/bin/env python
# Copyright (C) 2021 LEIDOS.
# Developed by Ryan Fleming @ UGA MSC Lab 2025
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.actions import TimerAction

def generate_launch_description():
    """
    Launches both the CARLA Bridge and conditionally the Ackermann Control node and vehicle spawn script.
    """
    return launch.LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            name='host',
            default_value='localhost',
            description='IP of the CARLA server'
        ),
        DeclareLaunchArgument(
            name='port',
            default_value='2000',
            description='TCP port of the CARLA server'
        ),
        DeclareLaunchArgument(
            name='timeout',
            default_value='10.0',
            description='Time to wait for a successful connection to the CARLA server'
        ),
        DeclareLaunchArgument(
            name='passive',
            default_value='false',
            description='When enabled, the ROS bridge will take a backseat and another client must tick the world (only in synchronous mode)'
        ),
        DeclareLaunchArgument(
            name='synchronous_mode',
            default_value='true',
            description='Enable/disable synchronous mode. If enabled, the ROS bridge waits until the expected data is received for all sensors'
        ),
        DeclareLaunchArgument(
            name='synchronous_mode_wait_for_vehicle_control_command',
            default_value='false',
            description='When enabled, pauses the tick until a vehicle control is completed (only in synchronous mode)'
        ),
        DeclareLaunchArgument(
            name='fixed_delta_seconds',
            default_value='0.05',
            description='Simulation time (delta seconds) between simulation steps'
        ),
        DeclareLaunchArgument(
            name='town',
            default_value='Town01',
            description='Either use an available CARLA town (eg. "Town01") or an OpenDRIVE file (ending in .xodr)'
        ),
        DeclareLaunchArgument(
            name='role_name',
            default_value='carma_1',
            description='Role name to identify ego vehicle, should match role_name in config at hero_config_path'
        ),
        DeclareLaunchArgument(
            name='vehicle_filter',
            default_value='vehicle.*',
            description='Selects which vehicles are availiable in CARLA for spawn'
        ),
        DeclareLaunchArgument(
            name='spawn_point',
            default_value='None',
            description='Spawn point to be used for vehicle spawn in CARLA'
        ),
        DeclareLaunchArgument(
            name='launch_spawn_vehicle',
            default_value='true',
            description='Determines if spawn_hero_vehicle script is launched alongside carla-ros2-bridge node'
        ),
        DeclareLaunchArgument(
            name='autopilot',
            default_value='false',
            description='Determines if the spawned vehicle should use CARLA autopilot'
        ),
        DeclareLaunchArgument(
            name='launch_ackermann_control',
            default_value='true',
            description='Determines if ackermann control node is launched alongside carla-ros2-bridge node'
        ),
        DeclareLaunchArgument(
            name='hero_config_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('carla_ros2_bridge'),
                'configs',
                'stack.json'  # vehicle role_name is hardcoded in this file, it should match the role_name argument
            ]),
            description='Path to the hero vehicle JSON config'
        ),

        # Main CARLA bridge node
        Node(
            package='carla_ros2_bridge',
            executable='bridge',
            name='carla_ros_bridge',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': True},
                {'host': LaunchConfiguration('host')},
                {'port': LaunchConfiguration('port')},
                {'timeout': LaunchConfiguration('timeout')},
                {'synchronous_mode': LaunchConfiguration('synchronous_mode')},
                {'synchronous_mode_wait_for_vehicle_control_command': LaunchConfiguration('synchronous_mode_wait_for_vehicle_control_command')},
                {'fixed_delta_seconds': LaunchConfiguration('fixed_delta_seconds')},
                {'ego_vehicle_role_name': LaunchConfiguration('role_name')},
                {'passive': LaunchConfiguration('passive')},
            ]
        ),

        # Delay spawn_vehicle_node to ensure bridge starts first
        TimerAction(
            period=5.0,  # Vehicle Spawn node
            actions=[
                Node(
                    package='carla_ros2_bridge',
                    executable='spawn_vehicle',
                    name='spawn_vehicle_node',
                    output='screen',
                    emulate_tty=True,
                    parameters=[
                        {'host': LaunchConfiguration('host')},
                        {'port': LaunchConfiguration('port')},
                        {'config_file': LaunchConfiguration('hero_config_path')},
                        {'autopilot': LaunchConfiguration('autopilot')},
                        {'spawn_point': LaunchConfiguration('spawn_point')},
                    ],
                    condition=IfCondition(LaunchConfiguration('launch_spawn_vehicle')),
                ),
            ]
        ),

        # Ackermann control node
        Node(
            package='carla_ros2_bridge',
            executable='ackermann_control',
            name='ackermann_control_node',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'role_name': LaunchConfiguration('role_name')},
                {'speed_Kp': 0.4},
                {'speed_Ki': 0.03},
                {'speed_Kd': 0.0},
                {'accel_Kp': 0.05},
                {'accel_Ki': 0.0},
                {'accel_Kd': 0.05},
            ],
            condition=IfCondition(LaunchConfiguration('launch_ackermann_control')),
        ),
        
    ])