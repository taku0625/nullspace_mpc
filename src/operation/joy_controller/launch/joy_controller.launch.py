from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("joy_controller"))
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("joy_param_path", default_value=str(share / "config" / "joy.yaml")),
        DeclareLaunchArgument("joy_controller_param_path", default_value=str(share / "config" / "joy_controller.yaml")),
        Node(
            package="joy",
            executable="joy_node",
            parameters=[LaunchConfiguration("joy_param_path")],
            output="screen",
        ),
        Node(
            package="joy_controller",
            executable="joy_controller_node",
            output="screen",
            parameters=[
                LaunchConfiguration("joy_controller_param_path"),
                {
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
        ),
    ])
