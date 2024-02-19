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
import sys
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
import numpy as np

for arg in sys.argv:
    if arg.startswith("node_count:="):
        count = int(arg.split(":=")[1])
    else:
        count = 10


def generate_launch_description():

    launch_file_dir = os.path.join(
        get_package_share_directory('firefly_swarm'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('firefly_swarm'),
        'worlds',
        'red_box.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
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



    # Get the urdf file
    TURTLEBOT3_MODEL = 'burger'
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf.xacro'

    print('urdf_file_name : {}'.format(urdf_file_name))

    urdf_path = os.path.join(
        get_package_share_directory('firefly_swarm'),
        'urdf',
        urdf_file_name)
    
    ld = LaunchDescription()
    
    n= count
    side_length = 20
    # Calculate the number of points per side in a grid
    points_per_side = int(np.ceil(np.sqrt(n)))
    # Calculate the spacing between points
    spacing = side_length / (points_per_side - 1)

    # Create the grid of points
    points = []
    for i in range(points_per_side):
        for j in range(points_per_side):
            x = i * spacing
            y = j * spacing
            points.append((x-side_length/2, y-side_length/2))

        

    for i,point in zip(range(count),points):
        robot_name = "firefly_"+str(i)
        x_val,y_val = point
        node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', robot_name,
                '-file', urdf_path,
                '-x', str(x_val),
                '-y', str(y_val),
                '-z', '0.01',
                '-robot_namespace', robot_name
            ],
            output='screen',
        )
        ld.add_action(TimerAction(period=10.0+float(i*2),
                                  actions=[node],))

    # ld.add_action(TimerAction(period=20.0+float(count*2),
    #         actions=[runner_node],))

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)

    # ld.add_action(spawn_turtlebot_cmd)
    # ld.add_action(node_arg)

    return ld
