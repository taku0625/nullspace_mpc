from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world_share = Path(get_package_share_directory("world_handler"))
    driver_share = Path(get_package_share_directory("vel_driver"))
    joy_share = Path(get_package_share_directory("joy_controller"))
    world_args = {
        "gazebo_world_name": "maze",
        "show_gazebo_gui": "true",
        "gazebo_headless": "false",
        "open_rviz": "true",
        "initial_pose_x": "0.0",
        "initial_pose_y": "0.0",
        "initial_pose_yaw": "0.0",
    }
    return LaunchDescription([
        *(DeclareLaunchArgument(arg, default_value=value) for arg, value in world_args.items()),
        DeclareLaunchArgument("use_joystick", default_value="true"),
        DeclareLaunchArgument("use_vel_driver", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(world_share / "launch" / "launch_gazebo_world_with_fwids.launch.py")),
            launch_arguments={arg: LaunchConfiguration(arg) for arg in world_args}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(driver_share / "launch" / "vel_driver.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items(),
            condition=IfCondition(LaunchConfiguration("use_vel_driver")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(joy_share / "launch" / "joy_controller.launch.py")),
            launch_arguments={"use_sim_time": "true"}.items(),
            condition=IfCondition(LaunchConfiguration("use_joystick")),
        ),
    ])
