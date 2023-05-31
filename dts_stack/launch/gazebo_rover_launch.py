import os

from ament_index_python import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessExit, OnExecutionComplete, OnProcessStart
from launch_ros.event_handlers import OnStateTransition
import xacro
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    gazebo_dir = get_package_share_directory('ros_gz_sim')  
    dts_dir = get_package_share_directory('dts_stack')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')    
    xacro_file = os.path.join(dts_dir, 'urdf', 'ackermannRobot', 'robot.urdf.xacro')
    robot_name = "robot"    
    
    # args that can be set from the command line
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    ekf_config = LaunchConfiguration('ekf_config')
    params_file = LaunchConfiguration('params_file')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')
    world = LaunchConfiguration('world')
    world_name = 'simple_world'
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')    
    
    # args that can be set from the command line or a default will be used
    rviz_la = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(dts_dir, 'rviz', 'gazebo_rviz_config.rviz'),
        description='Full path to rviz display config file')
    use_sim_time_la = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use simulation/Gazebo clock')
    world_config_la = DeclareLaunchArgument(
        'world',
        default_value=[os.path.join(dts_dir, 'worlds', 'simple_world.sdf'), ''],
        description='SDF world file')
    nav2_la = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(dts_dir, 'config','slam_nav2_params.yaml'),
        description='Full path to nav2 params file')
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
        'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
        'map_subscribe_transient_local': map_subscribe_transient_local}
    configured_params = RewrittenYaml(
            source_file=params_file,
            root_key="",
            param_rewrites=param_substitutions,
            convert_types=True)
    default_nav_to_pose_bt_xml_filename_la = DeclareLaunchArgument('default_nav_to_pose_bt_xml', 
                                                    default_value=os.path.join(dts_dir, 'behavior_trees','navigate_to_pose_w_replanning_and_recovery.xml'))    
    default_nav_through_poses_bt_xml_filename_la = DeclareLaunchArgument('default_nav_through_poses_bt_xml', 
                                                    default_value=os.path.join(dts_dir, 'behavior_trees','navigate_through_poses_w_replanning_and_recovery.xml'))
    map_subscribe_transient_local_la = DeclareLaunchArgument('map_subscribe_transient_local', 
                                                    default_value='true')
    robot_localization_la = DeclareLaunchArgument(
        'ekf_config', default_value=os.path.join(dts_dir, 'config/ekf.yaml'),
        description='Full path to ekf config file')  
    mux_la = DeclareLaunchArgument(
        'mux_config', default_value=os.path.join(dts_dir, 'config/mux.yaml'),
        description='Full path to params file')
    joy_la = DeclareLaunchArgument(
        'joy_config', default_value=os.path.join(dts_dir, 'config/joy_teleop.yaml'),
        description='Full path to params file')
        
        
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
        
    # start nodes and use args to set parameters
    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'ekf_config': ekf_config,
            'use_sim_time': use_sim_time
        }]
    )
    
    # include launch files    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_dir, 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'ign_args': [world, " --gui-config ",os.path.join(dts_dir, 'config', 'gazebo_gui.config')]}.items()
    )
    
    spawn_entity = Node(package='ros_gz_sim', executable='create',
        arguments=['-topic', 'robot_description', "-entity", robot_name],
        output='screen')    
        
    start_rviz2_cmd = ExecuteProcess(
        cmd=['rviz2','--display-config',rviz_config], 
        cwd=[os.path.join(dts_dir, 'rviz')], 
        output='screen')
    
    unpause_simulation = ExecuteProcess(
        cmd=['ign','service', '-s', '/world/simple_world/control', '--reqtype', 'ignition.msgs.WorldControl','--timeout', '1000', '--req', 'pause:false', '--reptype', 'ignition.msgs.Boolean'], 
        cwd=[dts_dir], 
        output='screen')       
        
    robot_state_publisher_node =   Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc, 'use_sim_time': use_sim_time}],
                output="screen")
    
    slam_toolbox_node = Node(
        parameters=[
        params_file,
        {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=["--ros-args", "-p", ['config_file:=', os.path.join(dts_dir, 'config','ros_gazebo_bridges.yaml')]]
    )   
    
    nav2_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_dir, 'launch/navigation_launch.py')]),
        launch_arguments={
            'params_file': configured_params
            }.items()
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )    
    twist_to_ackermann_node = Node(
        package='dts_stack',
        executable='twist_to_ackermann',
        name='twist_to_ackermann',
        output='screen'
    )
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
    teleop_twist = Node(
            package='teleop_twist_joy', executable='teleop_node', output='screen', parameters=[os.path.join(dts_dir, "config", "teleop_twist_joy.yaml")])


    map_odom_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_static_tf_node',
        arguments = ["0", "0", "0", "0", "0", "0", "map", "odom"]
    )
    odom_base_link_static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_base_link_static_tf_node',
        arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )
    
    robot_respawner = Node(
        package='dts_stack',
        executable='respawn_robot',
        name='respawn_robot',
        output='screen'
    )
    
    
    # create launch description
    ld = LaunchDescription()
    
    # declare launch args
    ld.add_action(use_sim_time_la)
    ld.add_action(rviz_la)
    ld.add_action(robot_localization_la)
    ld.add_action(world_config_la)
    ld.add_action(default_nav_to_pose_bt_xml_filename_la)
    ld.add_action(default_nav_through_poses_bt_xml_filename_la)
    ld.add_action(map_subscribe_transient_local_la)
    ld.add_action(nav2_la)
    ld.add_action(joy_la)
    ld.add_action(mux_la)
        
    # start nodes    
    ld.add_action(gazebo) 
    ld.add_action(spawn_entity)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(map_odom_static_tf_node)  
    ld.add_action(odom_base_link_static_tf_node)  
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(teleop_twist)
    ld.add_action(robot_respawner)
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[
                    TimerAction(
                        period=0.0,
                        actions=[ros_gz_bridge_node
                                 ]),
                    TimerAction(
                        period=2.0,
                        actions=[unpause_simulation
                                 ]),
                    TimerAction(
                        period=3.0,
                        actions=[start_rviz2_cmd
                                 ])
                    ],
                )
            )
                  )    
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=start_rviz2_cmd,
                on_start=[
                    TimerAction(
                        period=1.0,
                        actions=[slam_toolbox_node
                                 ]),
                    ],
                )
            )
                  )    
    ld.add_action(RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=slam_toolbox_node,
                on_start=[
                    TimerAction(
                        period=2.0,
                        actions=[nav2_start
                                 ]),
                    ],
                )
            )
                  ) 
    return ld
