import os
from launch import LaunchDescription
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    babybot_description = get_package_share_directory("babybot_description")

    # publish the urdf in real robot mode (is_sim = False)
    # gazebo run in simulation mode (default is_sim = True)
    robot_description = ParameterValue(Command([
            "xacro ",
            os.path.join(babybot_description,"urdf","babybot.urdf.xacro"),
            " is_sim:=False"
        ]), 
        value_type=str
    )
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description,
            "use_sim_time": False},
            os.path.join(
                get_package_share_directory("babybot_controller"),
                "config",
                "babybot_controllers.yaml"
            )
        ],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="screen"
    )

    return LaunchDescription([
        robot_state_publisher_node,
        controller_manager,
    ])


