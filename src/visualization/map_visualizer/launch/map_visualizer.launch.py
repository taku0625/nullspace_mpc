from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("map_visualizer"))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("map_name", default_value="maze", choices=["maze", "cylinder_garden"]),
        Node(
            package="map_visualizer",
            executable="map_visualizer_node",
            output="screen",
            parameters=[
                str(share / "config" / "map_visualizer.yaml"),
                {
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                    "map_name": LaunchConfiguration("map_name"),
                },
            ],
        ),
    ])
