from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("nullspace_mpc_bringup"))
    slam_share = Path(get_package_share_directory("slam_toolbox"))
    return LaunchDescription([
        DeclareLaunchArgument("gazebo_world_name", default_value="maze"),
        DeclareLaunchArgument("joy_operation", default_value="true"),
        DeclareLaunchArgument("show_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("gazebo_headless", default_value="false"),
        DeclareLaunchArgument("open_rviz", default_value="true"),
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch" / "gazebo_world.launch.py")),
                launch_arguments={
                    "gazebo_world_name": LaunchConfiguration("gazebo_world_name"),
                    "show_gazebo_gui": LaunchConfiguration("show_gazebo_gui"),
                    "gazebo_headless": LaunchConfiguration("gazebo_headless"),
                    "open_rviz": "false",
                    "use_joystick": LaunchConfiguration("joy_operation"),
                }.items(),
            ),
        ]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_share / "launch" / "online_async_launch.py")),
            launch_arguments={
                "slam_params_file": str(share / "config" / "slam_toolbox.yaml"),
                "use_sim_time": "true",
            }.items(),
        ),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", str(share / "rviz" / "mapping.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("open_rviz")),
        ),
    ])
