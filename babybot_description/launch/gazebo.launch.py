import os
from os import pathsep
from ament_index_python.packages import get_package_share_directory,get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,SetEnvironmentVariable,IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    
    # Finds the folder where your package's non-executable files (URDFs, meshes, launch files) are installed.
    # Finds the root of the "install" directory for that package. This is used to backtrack and find the broader share folder.
    babybot_description = get_package_share_directory("babybot_description")
    babybot_description_prefix=get_package_prefix("babybot_description")

    # Constructing the Model Path
    model_path=os.path.join(babybot_description,"models")
    model_path += pathsep + os.path.join(babybot_description_prefix,"share")

    # Setting the Environment Variable
    env_variable=SetEnvironmentVariable("GAZEBO_MODEL_PATH",model_path)

    # declare model argument
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(babybot_description,"urdf","babybot.urdf.xacro"),
        description="path to babybot urdf file"
    )

    # xacro and read the urdf
    robot_description = ParameterValue(Command(["xacro ",LaunchConfiguration("model")]),value_type=str)
    
    # publish robot description
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # Locate the gazebo_ros package
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # Define default world file path 
    default_world_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    # declare world argument
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to the world file to load'
    )

    # Include Gazebo's gzserver launch file
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items()
    )

    # Include Gazebo's gzclient launch file
    start_gazebo_client=IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"),"launch","gzclient.launch.py"
    )))

    # Spawn babybot
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","babybot","-topic","robot_description","-y","0.5"],
        output="screen"
    )

    return LaunchDescription([
        env_variable,
        world_arg,
        model_arg,
        robot_state_publisher,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])