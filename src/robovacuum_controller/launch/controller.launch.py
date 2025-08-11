from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to YAML config file
    ros2_control_params = os.path.join(
        get_package_share_directory("robovacuum_controller"),
        "config",
        "robovacuum_controller.yaml"   # matches your YAML filename
    )

    return LaunchDescription([
        # Controller manager
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[ros2_control_params],
            output="screen"
        ),

        # Diff drive controller spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robovacuum_controller"],  # must match URDF and YAML key
            output="screen"
        ),

        # Joint state broadcaster spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),
    ])
