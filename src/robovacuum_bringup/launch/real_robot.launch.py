import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robovacuum_firmware"),
                "launch",
                "hardware_interface.launch.py"
            )
        )
    )
    
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robovacuum_controller"),
                "launch",
                "controller.launch.py"
            )
        ),
        launch_arguments={
            "use_noisy_controller": "False",
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robovacuum_controller"),
                "launch",
                "joystick_teleop.launch.py"
            )
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    # imu_driver_node = Node(
    #     package="robovacuum_firmware",
    #     executable="mpu6050_driver.py"
    # )
    
    return LaunchDescription([
        hardware_interface,
        controller,
        joystick,
        # imu_driver_node,
    ])
