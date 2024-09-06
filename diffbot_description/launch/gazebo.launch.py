from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument 
from launch.conditions import IfCondition
import os

def generate_launch_description():

    use_rviz_arg = DeclareLaunchArgument(name="use_rviz", 
        default_value="False",                 
    )

    use_rviz = LaunchConfiguration("use_rviz")

    diffbot_description_share_Dir = get_package_share_directory("diffbot_description")
    diffbot_description_prefix = get_package_prefix("diffbot_description")


    robot_description = Command(["xacro ",
                                      os.path.join(diffbot_description_share_Dir,
                                      "urdf","diffbot.urdf.xacro")])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description}]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d",os.path.join(
                    diffbot_description_share_Dir,"rviz/display.rviz")],
        condition=IfCondition(use_rviz)
    )

    world = os.path.join(
        diffbot_description_share_Dir, 
        "worlds", 
        "empty_world.sdf"
    )

    env_var = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(diffbot_description_prefix, "share"))
    
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world] , 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    start_gazebo_ros_spawner_cmd = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "diffbot","-string", robot_description],
        output="screen"
    )

    return LaunchDescription([
        env_var,
        use_rviz_arg,
        robot_state_publisher_node,
        gazebo_launch,
        gzclient_cmd,
        start_gazebo_ros_spawner_cmd,
        rviz_node,
    ])