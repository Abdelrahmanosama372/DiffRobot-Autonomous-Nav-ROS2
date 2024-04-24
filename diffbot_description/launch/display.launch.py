from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():


    diffbot_description_share_Dir = get_package_share_directory("diffbot_description")
    robot_description_file = Command(["xacro ",
                                      os.path.join(diffbot_description_share_Dir,
                                      "urdf","diffbot.urdf.xacro")])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description":robot_description_file}]
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d",os.path.join(
                        diffbot_description_share_Dir,"rviz/display.rviz")]

    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])