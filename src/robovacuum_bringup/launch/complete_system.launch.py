import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # For Gazebo simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robovacuum_description'),
                'launch',
                'gazebo.launch.py'
            )
        ])
    )
    
    # Controller
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robovacuum_controller'),
                'launch',
                'controller.launch.py'
            )
        ])
    )
    
    # EKF Localization
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robovacuum_localization'),
                'launch',
                'local_localization.launch.py'
            )
        ])
    )
    
    # Visual SLAM
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('robovacuum_vision'),
                'launch',
                'rtabmap.launch.py'
            )
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # YOLO Object Detection
    yolo_detector = Node(
        package='robovacuum_vision',
        executable='yolo_detector.py',
        name='yolo_detector',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Obstacle Manager
    obstacle_manager = Node(
        package='robovacuum_vision',
        executable='obstacle_manager.py',
        name='obstacle_manager',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Nav2
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': os.path.join(
                get_package_share_directory('robovacuum_navigation'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )
    
    # Coverage Planner
    coverage_planner = Node(
        package='robovacuum_navigation',
        executable='coverage_planner.py',
        name='coverage_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory('robovacuum_description'),
        'config',
        'complete_system.rviz'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        gazebo,
        controller,
        localization,
        rtabmap,
        yolo_detector,
        obstacle_manager,
        nav2,
        coverage_planner,
        rviz
    ])