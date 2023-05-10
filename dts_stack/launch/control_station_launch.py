# MIT License

# Copyright (c) 2022 Rickard Häll   rickard.hall@ri.se

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

# Launch file to bring up control station nodes

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # get the directories
    dts_dir = get_package_share_directory('dts_stack')
    
    # args that can be set from the command line
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # args that can be set from the command line or a default will be used
    rviz_la = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(dts_dir, 'rviz/rviz_display_config.rviz'),
        description='Full path to rviz display config file')
    joy_la = DeclareLaunchArgument(
        'joy_config', default_value=os.path.join(dts_dir, 'config/joy_teleop.yaml'),
        description='Full path to params file')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation/Gazebo clock')
        
    # include launch files
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=[
            '-d', rviz_config],
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
    )
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args
    ld.add_action(joy_la)
    ld.add_action(rviz_la)
    ld.add_action(use_sim_time_la)    
    
    # start nodes
    ld.add_action(joy_node)
    ld.add_action(rviz_node)    
    return ld    
