from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    wheel_radius_arg=DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    wheel_separation_arg=DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    wheel_radius=LaunchConfiguration("wheel_radius")

    wheel_separation=LaunchConfiguration("wheel_separation")
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    diffdrive_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diffdrive_controller",
            "--controller-manager",
            "/controller_manager"
        ]
    )


    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        joint_state_broadcaster_spawner,
        diffdrive_controller
    ])