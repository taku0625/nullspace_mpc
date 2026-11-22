from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CONTROLLERS = {
    "nullspace_mpc": "nullspace_mpc",
    "mppi_3d_a": "mppi_3d",
    "mppi_3d_b": "mppi_3d",
    "mppi_4d": "mppi_4d",
    "mppi_h": "mppi_h",
    "mppi_unicycle": "mppi_unicycle",
}
MAPS = ["maze", "empty_garden", "cylinder_garden"]


def include(package, filename, **arguments):
    share = Path(get_package_share_directory(package))
    source = PythonLaunchDescriptionSource(str(share / "launch" / filename))
    return GroupAction([IncludeLaunchDescription(source, launch_arguments=arguments.items())])


def launch_navigation(context):
    share = Path(get_package_share_directory("nullspace_mpc_bringup"))
    controller = LaunchConfiguration("local_planner").perform(context)
    world = LaunchConfiguration("gazebo_world_name").perform(context)
    map_yaml = LaunchConfiguration("map_yaml").perform(context) or str(share / "map" / world / "map.yaml")
    scenario = LaunchConfiguration("scenario_config_path").perform(context)

    controller_args = {"use_sim_time": "true"}
    local_costmap_params = [str(share / "config" / "nav2.yaml"), {"autostart_node": True}]
    if controller == "nullspace_mpc":
        controller_args["controller_mode"] = LaunchConfiguration("controller_mode")
        local_costmap_params.append({
            "robot_radius": 0.8,
            "inflation_layer.inflation_radius": 1.3,
            "inflation_layer.cost_scaling_factor": 4.5,
        })

    actions = [
        include(
            "world_handler", "launch_gazebo_world_with_fwids.launch.py",
            gazebo_world_name=world, open_rviz="false",
            show_gazebo_gui=LaunchConfiguration("show_gazebo_gui"),
            gazebo_headless=LaunchConfiguration("gazebo_headless"),
            initial_pose_x=LaunchConfiguration("initial_pose_x"),
            initial_pose_y=LaunchConfiguration("initial_pose_y"),
            initial_pose_yaw=LaunchConfiguration("initial_pose_yaw"),
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        ),
        Node(
            package="nav2_map_server", executable="map_server", name="map_server",
            parameters=[{"use_sim_time": True, "yaml_filename": map_yaml}],
            output="screen",
        ),
        Node(
            package="nav2_planner", executable="planner_server", name="planner_server",
            parameters=[str(share / "config" / "nav2.yaml")],
            remappings=[("plan", "nav2_plan")],
            output="screen",
        ),
        Node(
            package="nav2_costmap_2d", executable="nav2_costmap_2d", name="local_costmap",
            namespace="local_costmap", parameters=local_costmap_params, output="screen",
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager",
            parameters=[{
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "planner_server"],
            }],
            output="screen",
        ),
        Node(
            package="nullspace_mpc_bringup", executable="goal_pose_bridge",
            parameters=[{"use_sim_time": True}], output="screen",
        ),
        include("vel_driver", "vel_driver.launch.py", use_sim_time="true"),
        include("reference_costmap_generator", "reference_costmap_generator.launch.py", use_sim_time="true"),
        include(CONTROLLERS[controller], f"{controller}.launch.py", **controller_args),
        Node(
            package="rviz2", executable="rviz2",
            arguments=["-d", str(share / "rviz" / "navigation.rviz")],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("open_rviz")),
        ),
    ]
    if world in ("maze", "cylinder_garden"):
        actions.append(include("map_visualizer", "map_visualizer.launch.py", map_name=world, use_sim_time="true"))
    if scenario:
        actions.append(Node(
            package="mpc_nav_evaluator", executable="mpc_nav_evaluator.py",
            parameters=[{
                "use_sim_time": True,
                "scenario_config_path": scenario,
                "eval_result_dir": LaunchConfiguration("eval_result_dir"),
            }],
            output="screen",
            on_exit=Shutdown(),
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("local_planner", default_value="nullspace_mpc", choices=list(CONTROLLERS)),
        DeclareLaunchArgument("controller_mode", default_value="default", choices=["default", "lite"]),
        DeclareLaunchArgument("gazebo_world_name", default_value="maze", choices=MAPS),
        DeclareLaunchArgument("map_yaml", default_value="", description="Defaults to the selected world's map."),
        DeclareLaunchArgument("initial_pose_x", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_y", default_value="0.0"),
        DeclareLaunchArgument("initial_pose_yaw", default_value="0.0"),
        DeclareLaunchArgument("show_gazebo_gui", default_value="true"),
        DeclareLaunchArgument("gazebo_headless", default_value="false"),
        DeclareLaunchArgument("open_rviz", default_value="true"),
        DeclareLaunchArgument("scenario_config_path", default_value=""),
        DeclareLaunchArgument("eval_result_dir", default_value="result"),
        OpaqueFunction(function=launch_navigation),
    ])
