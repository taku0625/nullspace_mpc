# Unicycle MPPI

The controller predicts `(x, y, yaw)` with Euler integration and samples
Gaussian noise on `(v, omega)`. The rolling horizon uses a cost-weighted average
of the feasible input sequences and shifts the result for the next cycle.
It uses forward velocity, yaw rate and an optional differential-drive wheel
speed bound. The objective penalizes heading, forward velocity, path distance,
collision cost, and changes in the two inputs.

## Gazebo Harmonic

```bash
ros2 launch nullspace_mpc_bringup navigation.launch.py \
  local_planner:=mppi_unicycle gazebo_world_name:=empty_garden
```

The node reads `/groundtruth_odom`, `/plan`, `/local_costmap/costmap`,
`/distance_error_map` and `/ref_yaw_map`. It publishes `Twist` on `/cmd_vel`
with `linear.y = 0` and supplies the existing `/mpc` visualization and
evaluation topics. Lethal, unknown, and out-of-costmap predicted states
disqualify a rollout.

The `mppi_unicycle.yaml` profile sets the prediction horizon, noise and input
limits. Both `sigma` and `control_change_weight` use `[v, omega]` order.
For the standalone node, use `ros2 launch mppi_unicycle mppi_unicycle.launch.py`
and supply `params_file` to override the settings.

## Recorded paths

`follow_path_node` serves the `mppi_unicycle/action/FollowPath` action. It
reads the live transform from the path frame to `base_link` and publishes
`TwistStamped` for a differential-drive chassis. It commands zero at the goal,
on cancellation, and when localization has gone stale. An optional
`collision_costmap_topic` provides a live occupancy grid for this mode.
