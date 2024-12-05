from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument 
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():

    map_name_arg = DeclareLaunchArgument(
        name="map_name",
        default_value="small_house"
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True"
    )

    lifecycle_nodes = ["map_server"]
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_name = LaunchConfiguration("map_name")

    map_path = PathJoinSubstitution([
        get_package_share_directory("diffbot_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])

    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ],
    )

    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        nav2_map_server,
        nav2_lifecycle_manager
    ])

