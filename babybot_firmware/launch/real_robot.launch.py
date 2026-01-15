import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("babybot_firmware"),
            "launch",
            "babybot_interface.launch.py"
        )
    )

    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("babybot_controller"),
            "launch",
            "diffdrive_controller.launch.py"
        )
    )

    imu_driver_node = Node(
        package="babybot_firmware",
        executable="mpu6050_driver.py"
    )

    yd_laser_driver = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("ydlidar_ros2_driver"),
            "launch",
            "ydlidar_launch.py"
        )
    )


    return LaunchDescription([
        hardware_interface,
        controller,
        imu_driver_node,
        yd_laser_driver
    ])
