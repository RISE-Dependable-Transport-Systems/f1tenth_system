# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetRemap

def generate_launch_description():
    # get the directories
    dts_dir = get_package_share_directory('dts_stack')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    # args that can be set from the command line    
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_scan_topic = LaunchConfiguration('lidar_scan_topic')
        
    # args that can be set from the command line or a default will be used   
    vesc_la = DeclareLaunchArgument(
        'vesc_config', default_value=os.path.join(dts_dir, 'config/vesc.yaml'),
        description='Full path to params file')
    mux_la = DeclareLaunchArgument(
        'mux_config', default_value=os.path.join(dts_dir, 'config/mux.yaml'),
        description='Full path to params file')
    lidar_frame_id_la = DeclareLaunchArgument('lidar_frame_id', default_value='lidar_link')
    lidar_scan_topic_la = DeclareLaunchArgument('lidar_scan_topic', default_value='/scan_lidar')
    
    # include launch files    
    sllidar_s1_launch_file = GroupAction(
        actions=[
            SetRemap(src='/scan',dst=lidar_scan_topic),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(sllidar_dir, 'launch/sllidar_s1_launch.py')),
                launch_arguments = {
                    'frame_id' : lidar_frame_id
                    }.items(),
            )
        ]
    )
    
    # start nodes and use args to set parameters
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')]
    )    
    twist_to_ackermann_node = Node(
        package='dts_stack',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen'
    )       
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args 
    ld.add_action(vesc_la)    
    ld.add_action(mux_la)
    ld.add_action(lidar_frame_id_la)
    ld.add_action(lidar_scan_topic_la)
    
    # start nodes
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ackermann_mux_node)    
    ld.add_action(twist_to_ackermann_node)  
    
    # run another launch flie 
    ld.add_action(sllidar_s1_launch_file)
    
    return ld
                
