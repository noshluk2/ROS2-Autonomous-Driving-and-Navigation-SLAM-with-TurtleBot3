#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
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
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    hotel_path = os.path.join(get_package_share_directory('autonomous_tb3'),'world','hotel','model.sdf')
    table_path = os.path.join(get_package_share_directory('autonomous_tb3'),'models','table','model.sdf')
    config_dir = os.path.join(get_package_share_directory('autonomous_tb3'),'config')
    map_file = os.path.join(config_dir,'hotel_map.yaml')
    actor_path= os.path.join(get_package_share_directory('autonomous_tb3'),'models','actor','model.sdf')

    world_path = os.path.join(get_package_share_directory('autonomous_tb3'),'world','hotel','empty_world.world')
    params_file = os.path.join(config_dir,'tb3_nav_params.yaml')
    map_config= os.path.join(config_dir,'mapping.rviz')
    nav_config= os.path.join(config_dir,'tb3_nav.rviz')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-4.5')
    y_pose = LaunchConfiguration('y_pose', default='0.51')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),launch_arguments={'world': world_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    hotel_spawner=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[hotel_path,"hotel","0.0" ,"0.0" ]

    )

    table_spawner_1=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[table_path,"table_1","-0.6" ,"3.99" ]

    )

    table_spawner_2=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[table_path,"table_2","4.52" ,"3.99" ]

    )

    table_spawner_3=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[table_path,"table_3","4.53" ,"-3.17" ]

    )

    table_spawner_4=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[table_path,"table_4","-0.6" ,"-3.09" ]

    )

    actor_spawn=Node(
        package='autonomous_tb3',
        output='screen',
        executable='sdf_spawner',
        name='hotel_spawner',
        arguments=[actor_path,"actor","-0.6" ,"-3.09" ]

    )


    hotel_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),'launch', 'online_async_launch.py')
        ),
    )

    hotel_nav=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
        launch_arguments={
        'map':map_file,
        'params_file': params_file}.items(),

    )
    rviz=Node(
        package='rviz2',
        output='screen',
        executable='rviz2',
        name='rviz2_node',
        # arguments=['-d',map_config]
        arguments=['-d',nav_config]

    )




    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    ld.add_action(spawn_turtlebot_cmd)

    ld.add_action(hotel_spawner)
    ld.add_action(table_spawner_1)
    ld.add_action(table_spawner_2)
    ld.add_action(table_spawner_3)
    ld.add_action(table_spawner_4)

    # ld.add_action(hotel_mapping)
    ld.add_action(rviz)

    ld.add_action(hotel_nav)

    return ld