import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_prefix, get_package_share_directory


def generate_launch_description():
  
    diffbot_description_share_Dir = get_package_share_directory("diffbot_description")
    diffbot_description_prefix = get_package_prefix("diffbot_description")
    gazebo_ros_share_Dir = get_package_share_directory("gazebo_ros")


    launch_rviz_arg = DeclareLaunchArgument(
        name="launch_rviz",
        default_value="true",
        description="launch rviz?"
    )

    launch_rviz = LaunchConfiguration("launch_rviz")


    gazebo_model_path = os.path.join(diffbot_description_share_Dir, "models")
    gazebo_model_path += os.pathsep + os.path.join(diffbot_description_prefix, "share")
    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path)


    robot_description = ParameterValue(Command(
        ["xacro ",os.path.join(
                        diffbot_description_share_Dir,"urdf","diffbot.urdf.xacro")]
        ),value_type=str)


    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d",os.path.join(diffbot_description_share_Dir,"rviz","display.rviz")],
        condition=IfCondition(launch_rviz)
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_Dir,"launch","gzserver.launch.py")
        )
    )
    

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share_Dir,"launch","gzclient.launch.py")
        )
    )

    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity","diffbot", "-topic","robot_description"],
         output="screen"
    )

    return LaunchDescription([
        launch_rviz_arg,
        env_var,
        gazebo_server,
        gazebo_client,
        robot_state_publisher_node,
        rviz_node,
        spawn_robot_node
    ])
