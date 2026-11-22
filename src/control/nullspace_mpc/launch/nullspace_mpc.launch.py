from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_controller(context):
    share = Path(get_package_share_directory("nullspace_mpc"))
    mode = LaunchConfiguration("controller_mode").perform(context)
    config = "nullspace_mpc_lite.yaml" if mode == "lite" else "nullspace_mpc.yaml"
    return [
        Node(
            package="nullspace_mpc",
            executable="nullspace_mpc_node",
            output="screen",
            parameters=[
                str(share / "config" / config),
                {"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("controller_mode", default_value="default", choices=["default", "lite"]),
        OpaqueFunction(function=launch_controller),
    ])
