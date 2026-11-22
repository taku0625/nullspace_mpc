from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory("mpc_nav_evaluator"))
    return LaunchDescription([
        DeclareLaunchArgument("scenario_config_path"),
        DeclareLaunchArgument("eval_result_dir", default_value="result"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="mpc_nav_evaluator",
            executable="mpc_nav_evaluator.py",
            parameters=[
                str(share / "config" / "mpc_nav_evaluator.yaml"),
                {
                    "scenario_config_path": LaunchConfiguration("scenario_config_path"),
                    "eval_result_dir": LaunchConfiguration("eval_result_dir"),
                    "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                },
            ],
            output="screen",
        ),
    ])
