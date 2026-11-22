from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("reference_costmap_generator"))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="reference_costmap_generator",
            executable="reference_costmap_generator_node",
            output="screen",
            parameters=[
                str(share / "config" / "reference_costmap_generator.yaml"),
                {
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
