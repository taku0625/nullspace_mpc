from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import xacro


def launch_world(context):
    share = Path(get_package_share_directory("world_handler"))
    world = LaunchConfiguration("gazebo_world_name").perform(context)
    description = xacro.process_file(str(share / "xacro" / "fwids_vehicle.xacro")).toxml()
    gazebo = ["gz", "sim", "-r", "-v", "2"]
    if not IfCondition(LaunchConfiguration("show_gazebo_gui")).evaluate(context):
        gazebo.append("-s")
    if IfCondition(LaunchConfiguration("gazebo_headless")).evaluate(context):
        gazebo.append("--headless-rendering")

    bridge = [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/groundtruth_odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/laser_link/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        "/fwids/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
    ]
    for wheel in ("front_left", "front_right", "rear_left", "rear_right"):
        for command in ("steer_rad", "rotor_radpersec"):
            bridge.append(f"/fwids/{wheel}_{command}/command@std_msgs/msg/Float64]gz.msgs.Double")

    return [
        AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", str(share / "models")),
        ExecuteProcess(
            cmd=[*gazebo, str(share / "world" / f"{world}.world")],
            output="screen",
            on_exit=Shutdown(),
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "use_sim_time": True,
                "robot_description": ParameterValue(description, value_type=str),
            }],
            remappings=[("joint_states", "/fwids/joint_states")],
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-world", "default", "-name", "fwids", "-topic", "robot_description",
                "-x", LaunchConfiguration("initial_pose_x"),
                "-y", LaunchConfiguration("initial_pose_y"),
                "-z", "0.5", "-Y", LaunchConfiguration("initial_pose_yaw"),
            ],
            output="screen",
        ),
        Node(package="ros_gz_bridge", executable="parameter_bridge", arguments=bridge, output="screen"),
        Node(
            package="groundtruth_odom_publisher",
            executable="groundtruth_odom_publisher_node",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", str(share / "rviz" / "fwids_vehicle.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("open_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "gazebo_world_name", default_value="maze",
            choices=["empty", "empty_garden", "cylinder_garden", "maze"],
        ),
        DeclareLaunchArgument("show_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("gazebo_headless", default_value="false"),
        DeclareLaunchArgument("open_rviz", default_value="true"),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        OpaqueFunction(function=launch_world),
    ])
