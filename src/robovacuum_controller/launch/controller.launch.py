from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
import os

def noisy_controller(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_noisy_controller = LaunchConfiguration("use_noisy_controller")
    
    try:
        wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
        wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
        wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
        wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))
    except ValueError as e:
        # Use default values if conversion fails
        wheel_radius = 0.1
        wheel_separation = 0.35
        wheel_radius_error = 0.005
        wheel_separation_error = 0.02
    
    noisy_controller_py = Node(
        package="robovacuum_controller",
        executable="noisy_controller.py",
        parameters=[
            {"wheel_radius": wheel_radius + wheel_radius_error,
             "wheel_separation": wheel_separation + wheel_separation_error,
             "use_sim_time": use_sim_time}],
        condition=IfCondition(use_noisy_controller)
    )
    return [
        noisy_controller_py,
    ]

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
    )
    use_noisy_controller_arg = DeclareLaunchArgument(
        "use_noisy_controller",
        default_value="False",
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.1",
    )
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.35",
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
    )
    wheel_separation_error_arg = DeclareLaunchArgument(
        "wheel_separation_error",
        default_value="0.02",
    )
    
    # Path to YAML config file
    ros2_control_params = os.path.join(
        get_package_share_directory("robovacuum_controller"),
        "config",
        "robovacuum_controller.yaml"
    )
    
    noisy_controller_launch = OpaqueFunction(function=noisy_controller)
    
    return LaunchDescription([
        # Launch arguments must come first
        use_sim_time_arg,
        use_noisy_controller_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        wheel_radius_error_arg,
        wheel_separation_error_arg,
        
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
            arguments=["robovacuum_controller"],
            output="screen"
        ),
        # Joint state broadcaster spawner
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
            output="screen"
        ),
        
        noisy_controller_launch,
    ])