"""
BFMC Simulation Launch File

Launches complete autonomous navigation system in Gazebo simulation
with traffic sign detection, FSM control, and safety monitoring.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='bfmc_track',
        description='Gazebo world name'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera topic name'
    )
    
    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='0.6',
        description='Maximum vehicle speed (m/s)'
    )
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    camera_topic = LaunchConfiguration('camera_topic')
    max_speed = LaunchConfiguration('max_speed')
    
    # Gazebo launch (optional - uncomment if you have gazebo setup)
    # gazebo_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('gazebo_ros'),
    #             'launch',
    #             'gazebo.launch.py'
    #         ])
    #     ]),
    #     launch_arguments={'world': LaunchConfiguration('world')}.items()
    # )
    
    # Traffic Sign Detection Node
    traffic_detector_node = Node(
        package='bfmc_perception',
        executable='traffic_detector',
        name='traffic_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': camera_topic,
            'detection_rate': 20.0,
            'confidence_threshold': 0.7,
            'model_path': 'models/traffic_detector.pth'
        }],
        remappings=[
            ('/camera/image_raw', camera_topic),
        ]
    )
    
    # Lane Detection Node
    lane_detector_node = Node(
        package='bfmc_perception',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'camera_topic': camera_topic,
            'roi_height': 0.5,
            'roi_width': 1.0
        }]
    )
    
    # Sensor Fusion Node
    sensor_fusion_node = Node(
        package='bfmc_perception',
        executable='sensor_fusion',
        name='sensor_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'lidar_topic': '/scan',
            'camera_topic': camera_topic,
            'imu_topic': '/imu'
        }]
    )
    
    # FSM Controller Node
    fsm_controller_node = Node(
        package='bfmc_control',
        executable='fsm_controller',
        name='fsm_controller',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_speed': max_speed,
            'control_frequency': 50.0,
            'stop_distance': 0.5,
            'stop_duration': 3.0
        }]
    )
    
    # Path Planner Node
    path_planner_node = Node(
        package='bfmc_planning',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'planning_algorithm': 'dwa',
            'goal_tolerance': 0.2
        }]
    )
    
    # Safety Monitor Node
    safety_monitor_node = Node(
        package='bfmc_control',
        executable='safety_monitor',
        name='safety_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'emergency_stop_distance': 0.3,
            'warning_distance': 0.6
        }]
    )
    
    # RViz Visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bfmc_nav'),
            'rviz',
            'bfmc_sim.rviz'
        ])],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        world_arg,
        camera_topic_arg,
        max_speed_arg,
        
        # Nodes
        # gazebo_launch,  # Uncomment if using Gazebo
        traffic_detector_node,
        lane_detector_node,
        sensor_fusion_node,
        fsm_controller_node,
        path_planner_node,
        safety_monitor_node,
        # rviz_node,  # Uncomment for visualization
    ])
