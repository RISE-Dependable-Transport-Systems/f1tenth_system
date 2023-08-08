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
import xacro

def generate_launch_description():
    # get the directories
    dts_dir = get_package_share_directory('dts_stack')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    xacro_file = os.path.join(dts_dir, 'urdf', 'ackermannRobot', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    
    # args that can be set from the command line
    model = LaunchConfiguration('model')
    rviz_config = LaunchConfiguration('rviz_config')
    ekf_config = LaunchConfiguration('ekf_config')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    default_bt_xml_filename = LaunchConfiguration('default_bt_xml_filename')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    
    # args that can be set from the command line or a default will be used
    joy_la = DeclareLaunchArgument(
        'joy_config', default_value=os.path.join(dts_dir, 'config/joy_teleop.yaml'),
        description='Full path to params file')
    default_bt_xml_filename_la = DeclareLaunchArgument('default_bt_xml_filename', 
                                                       default_value=os.path.join(dts_dir, 'config/navigate_w_replanning_and_recovery.xml'))
    map_subscribe_transient_local_la = DeclareLaunchArgument('map_subscribe_transient_local', 
                                                       default_value='true')
    rviz_la = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(dts_dir, 'rviz/rviz_display_config.rviz'),
        description='Full path to rviz display config file')
    ekf_la = DeclareLaunchArgument(
        'ekf_config', default_value=os.path.join(dts_dir, 'config/ekf.yaml'),
        description='Full path to ekf config file')     
    nav2_la = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(dts_dir, 'config/nav2_params_modified.yaml'),
        description='Full path to nav2 params file')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='False',
        description='Use simulation/Gazebo clock')
        
    # include launch files
    slam_toolbox_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(slam_toolbox_dir, 'launch/online_async_launch.py')]),
        launch_arguments={
            'slam_params_file': params_file,
            'use_sim_time': use_sim_time
            }.items()
    )
    nav2_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch/navigation_launch.py')]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'default_bt_xml_filename': default_bt_xml_filename,
            'map_subscribe_transient_local': map_subscribe_transient_local
            }.items()
    )

    # start nodes and use args to set parameters
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    ) 
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }]
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
    ekf_filter_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
        remappings=[('/odometry/filtered', '/ekf_odom')]
    )
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args
    ld.add_action(joy_la)
    ld.add_action(default_bt_xml_filename_la)
    ld.add_action(map_subscribe_transient_local_la)
    ld.add_action(rviz_la)
    ld.add_action(ekf_la)
    ld.add_action(use_sim_time_la)    
    ld.add_action(nav2_la)
    
    # start nodes
    ld.add_action(joy_node) 
    ld.add_action(joy_teleop_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    ld.add_action(ekf_filter_node)
    
    # start launch files
    ld.add_action(slam_toolbox_start) 
    ld.add_action(nav2_start)
    
    return ld    
