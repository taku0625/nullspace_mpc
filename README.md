# Nullspace MPC

Nullspace MPC is a novel multi-objective control framework for explicitly handling task priorities.
It demonstrates agile and smooth navigation through a narrow environment with a swerve drive vehicle.

<div align="center">

[![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Docker](https://img.shields.io/badge/-Docker-EEE.svg?logo=docker&style=flat)](https://www.docker.com/)
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

<!-- eyecatch movie -->
![nullspace_mpc_anim_3drviz](./media/nullspace_mpc_anim_3drviz.gif)

</div>

## Setup

Use Ubuntu 24.04 and ROS 2 Jazzy. In a standalone checkout, run the following
from the repository root. In a parent ROS workspace, initialize this submodule
and run the commands from the workspace root.

```bash
source /opt/ros/jazzy/setup.bash
sudo rosdep init  # Once per machine
rosdep update --rosdistro jazzy
rosdep install -y --from-paths src --ignore-src --rosdistro jazzy
colcon build --packages-up-to nullspace_mpc_bringup --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/local_setup.bash
```

The QP solver uses the Jazzy `osqp_vendor` package. Its build downloads the
pinned OsqpEigen release if a compatible copy is unavailable.

For Docker, use `make setup_docker_cpu` and `make run_docker_cpu`.
`make run_docker_gpu` uses the same image with NVIDIA Container Toolkit.
Run `make build` from the container shell.

## Navigate in Gazebo Harmonic

```bash
ros2 launch nullspace_mpc_bringup navigation.launch.py
```

Use the **2D Goal Pose** tool in RViz2, or publish a `geometry_msgs/msg/PoseStamped`
on `/goal_pose` in the `map` frame. Nav2 computes the global path, and the
selected MPC controller publishes `/cmd_vel`. The demo uses Gazebo ground
truth for localization.

- Run the reduced-computation controller with `controller_mode:=lite`.
- Set `local_planner:=mppi_h` for the hybrid MPPI baseline.
  `mppi_3d_a`, `mppi_3d_b`, `mppi_4d` and `mppi_unicycle` are also available.
- Select `gazebo_world_name:=maze`, `empty_garden` or `cylinder_garden`.
- For a headless run, use `open_rviz:=false show_gazebo_gui:=false gazebo_headless:=true`.

The unicycle mode uses `(v, omega)` and keeps the existing Gazebo and
evaluation interfaces. [Unicycle MPPI settings](src/control/mppi_unicycle/README.md)

Use `gazebo_world.launch.py` for manual joystick operation and
`mapping.launch.py` for SLAM Toolbox. View ROS 2 bag data with
`rosbag_play.launch.py` and `ros2 bag play <bag> --clock`.

### Use an existing ROS 2 navigation stack

`ros2 launch nullspace_mpc nullspace_mpc.launch.py` starts the controller
with the default profile. Choose `controller_mode:=lite` for the lighter profile.
The node expects `/groundtruth_odom`, `/plan`, `/local_costmap/costmap`,
`/distance_error_map` and `/ref_yaw_map`. Supply odometry pose, path and
maps in the same coordinate frame. The YAML files in
`src/control/nullspace_mpc/config/` configure the topic names and controller.
`reference_costmap_generator.launch.py` in the
`reference_costmap_generator` package produces the two reference maps from
`/map` and `/plan`.

### Multi-goal evaluation

Run from the repository root after building and sourcing the overlay.

```bash
ros2 run nullspace_mpc_bringup run_evaluation.py \
  --agenda_yaml_path data/eval_demo/agenda.yaml --controller nullspace_mpc
```

Use `--controller mppi_unicycle` or `--controller mppi_h` for a baseline or `--headless` to omit the GUIs.
The demo agenda writes each episode and the summary CSV to `./result/`.

## Citation
If you use this software in your research, please consider citing the following papers.

Nullspace MPC is a core component of my Ph.D. dissertation, which was made publicly available on October 1, 2025.
You can access the full paper via the [project website](https://mizuhoaoki.github.io/projects/nullspace_mpc).
```bibtex
  @phdthesis{mizuho2025phd,
    author    = {Mizuho Aoki},
    title     = {{Nonlinear Model Predictive Control for Autonomous Vehicles: Enhancement via Simplified Physics-Aware Prediction and Decomposed Optimization}},
    school    = {Nagoya University},
    year      = {2025},
    type      = {Ph.D. Dissertation},
    language  = {English},
    url       = {https://mizuhoaoki.github.io/projects/nullspace_mpc}
  }
```

The baseline MPPI controller is based on the following work and is available in a dedicated repository: [mppi_swerve_drive_ros](https://github.com/MizuhoAOKI/mppi_swerve_drive_ros)  
```bibtex
@inproceedings{mizuho2024iros,
  author={Aoki, Mizuho and Honda, Kohei and Okuda, Hiroyuki and Suzuki, Tatsuya},
  booktitle={2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={{Switching Sampling Space of Model Predictive Path-Integral Controller to Balance Efficiency and Safety in 4WIDS Vehicle Navigation}}, 
  year={2024},
  pages={3196-3203},
  doi={10.1109/IROS58592.2024.10802359}
}
```

## License & Acknowledgements

This project is licensed under the MIT License, with the exception of its core QP solving capability, which is provided by [QpSolverCollection](https://github.com/isri-aist/QpSolverCollection), located in the `src/third_party` directory and licensed under the BSD 2-Clause License.

Special thanks to the authors of QpSolverCollection for their excellent work.
