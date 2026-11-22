from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("nullspace_mpc_bringup"))
    return LaunchDescription([
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", str(share / "rviz" / "navigation.rviz")],
            parameters=[{"use_sim_time": True}],
        ),
    ])
