# launch file to bring up rover nodes

import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, SetRemap
import xacro

def generate_launch_description():
    # get the directories
    dts_dir = get_package_share_directory('dts_stack')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    xacro_file = os.path.join(dts_dir, 'urdf', 'ackermannRobot', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # args that can be set from the command line
    ekf_config = LaunchConfiguration('ekf_config')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')    
    lidar_frame_id = LaunchConfiguration('lidar_frame_id')
    lidar_scan_topic = LaunchConfiguration('lidar_scan_topic')
        
    # args that can be set from the command line or a default will be used
    joy_la = DeclareLaunchArgument(
        'joy_config', default_value=os.path.join(dts_dir, 'config/joy_teleop.yaml'),
        description='Full path to params file')
    ekf_la = DeclareLaunchArgument(
        'ekf_config', default_value=os.path.join(dts_dir, 'config/ekf.yaml'),
        description='Full path to ekf config file')        
    vesc_la = DeclareLaunchArgument(
        'vesc_config', default_value=os.path.join(dts_dir, 'config/vesc.yaml'),
        description='Full path to params file')
    mux_la = DeclareLaunchArgument(
        'mux_config', default_value=os.path.join(dts_dir, 'config/mux.yaml'),
        description='Full path to params file')
    nav2_la = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(dts_dir, 'config/nav2_params_modified.yaml'),
        description='Full path to nav2 params file')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation/Gazebo clock')
    lidar_frame_id_la = DeclareLaunchArgument('lidar_frame_id', default_value='lidar_link')
    lidar_scan_topic_la = DeclareLaunchArgument('lidar_scan_topic', default_value='/scan_lidar')
    
    # include launch files    
    sllidar_s1_launch_include = GroupAction(
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
            os.path.join(dts_dir, 'launch/navigation_launch.py')]),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': use_sim_time
            }.items()
    )
    
    # start nodes and use args to set parameters
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
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
            'rate': 50,
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
    twist_to_ackermann_node = Node(
        package='dts_stack',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen'
    )       
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args
    ld.add_action(joy_la)
    ld.add_action(ekf_la)
    ld.add_action(use_sim_time_la)    
    ld.add_action(nav2_la)
    ld.add_action(vesc_la)
    ld.add_action(mux_la)  
    ld.add_action(lidar_frame_id_la)
    ld.add_action(lidar_scan_topic_la)
    
    # start nodes  
    ld.add_action(joy_teleop_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(ekf_filter_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(ackermann_mux_node)    
    ld.add_action(twist_to_ackermann_node)     
    
    # start launch files   
    ld.add_action(slam_toolbox_start)  
    ld.add_action(sllidar_s1_launch_include)  
    ld.add_action(nav2_start)
    
    return ld
