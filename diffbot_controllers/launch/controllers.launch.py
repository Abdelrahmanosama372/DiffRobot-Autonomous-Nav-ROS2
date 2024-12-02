import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory 

def noisy_controller(context, *args, **kwargs):
    wheel_raduis = float(LaunchConfiguration("wheel_raduis").perform(context))
    wheel_separation = float(LaunchConfiguration("wheel_separation").perform(context))
    wheel_raduis_error = float(LaunchConfiguration("wheel_raduis_error").perform(context))
    wheel_separation_error = float(LaunchConfiguration("wheel_separation_error").perform(context))

    noisy_controller_node = Node (
        package="diffbot_controllers",
        executable="noisy_controller",
        parameters=[
            {
                "wheel_raduis": wheel_raduis + wheel_raduis_error,
                "wheel_separation": wheel_separation + wheel_separation_error
            }
        ],
    )   

    return [
        noisy_controller_node
    ]


def generate_launch_description():

    wheel_raduis_arg = DeclareLaunchArgument(
        name="wheel_raduis",
        default_value="0.033"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        name="wheel_separation",
        default_value="0.17"
    )

    wheel_raduis_error_arg = DeclareLaunchArgument(
        name="wheel_raduis_error",
        default_value="0.005"
    )

    wheel_separation_error_arg = DeclareLaunchArgument(
        name="wheel_separation_error",
        default_value="0.02"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True"
    )

    wheel_raduis = LaunchConfiguration("wheel_raduis")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_sim_time = LaunchConfiguration("use_sim_time")

    diffbot_controller_pkg = get_package_share_directory('diffbot_controllers') 

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager","/controller_manager"]
    )

    velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_controller",
                   "--controller-manager","/controller_manager"]
    )


    simple_controller_node = Node (
        package="diffbot_controllers",
        executable="simple_controller",
        parameters=[
            {
                "wheel_raduis": wheel_raduis,
                "wheel_separation": wheel_separation
            }
        ],
    )

    twist_mux_node = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("twist_mux"),
            "launch",
            "twist_mux_launch.py"
        ),
        launch_arguments={
            "cmd_vel_out": "diffbot_controller/cmd_vel_unstamped",
            "config_locks": os.path.join(diffbot_controller_pkg, "config", "twist_mux_locks.yaml"),
            "config_topics": os.path.join(diffbot_controller_pkg, "config", "twist_mux_topics.yaml"),
            "config_joy": os.path.join(diffbot_controller_pkg, "config", "twist_mux_joy.yaml"),
            "use_sim_time": use_sim_time,
        }.items(),
    )

    noisy_controller_launch = OpaqueFunction(function=noisy_controller)

    return LaunchDescription(
        [
            wheel_raduis_arg,
            wheel_separation_arg,
            wheel_raduis_error_arg,
            wheel_separation_error_arg,
            use_sim_time_arg,
            joint_state_broadcaster_spawner,
            velocity_controller_spawner,
            twist_mux_node,
            #noisy_controller_launch,
            #simple_controller_node
        ]
    )
