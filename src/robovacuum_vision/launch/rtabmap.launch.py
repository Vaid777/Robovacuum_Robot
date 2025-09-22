import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = os.path.join(
        get_package_share_directory('robovacuum_vision'),
        'config',
        'rtabmap.yaml'
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        
        # RTAB-Map Visual Odometry
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            parameters=[config_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/visual_odom')
            ],
            output='screen'
        ),
        
        # RTAB-Map SLAM
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            parameters=[config_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                ('odom', '/robovacuum_controller/odom_noisy'),
                ('move_base/goal', '/move_base_simple/goal'),
                ('grid_map', '/map')
            ],
            arguments=['--delete_db_on_start'],
            output='screen'
        ),
        
        # Visualization
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            parameters=[config_path, {'use_sim_time': use_sim_time}],
            remappings=[
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info')
            ],
            output='screen'
        ),
    ])