# Unicycle MPPI

The recorded-route controller takes a `FollowPath` action and a matching
`/reference_costmap` from the Misachiya navigation launch. It uses live
`map → base_link` TF and publishes `geometry_msgs/msg/TwistStamped` to
`/cmd_vel`. The action server stops the chassis on completion, cancellation,
stale localization or collision data, and loss of route progress.

## Solver

The solver is the MPPI of the original `mppi_3d` package with the state
`(x, y, yaw)` and the two inputs `(v, omega)` of a unicycle:

1. Gaussian noise is drawn for every sample and step. The samples add it to
   the solution of the previous cycle; the `exploration` share of them adds it
   to zero input.
2. The rollouts run in parallel with OpenMP. Each one sums the stage cost, the
   input cost `lambda * (1 - alpha) * u' * inv(Sigma) * v` and the terminal cost.
3. The noise of each sample is weighted by `exp(-(cost - lowest cost) / lambda)`
   and added to the previous solution.
4. A Savitzky-Golay filter over the past commands and the planned inputs
   smooths the command, and the inputs are limited.

`OMP_NUM_THREADS` sets the number of threads. The commands do not depend on it.

| Parameter | `mppi_3d` name |
| --- | --- |
| `samples`, `prediction_steps`, `prediction_step` | `num_samples`, `prediction_horizon`, `step_len_sec` |
| `exploration`, `lambda`, `alpha`, `sigma` | `param_exploration`, `param_lambda`, `param_alpha`, `sigma` |
| `reduce_computation` | `reduce_computation` |
| `reference_speed` | `reference_velocity` |
| `velocity_weight`, `heading_weight` | `weight_velocity_error`, `weight_angular_error` |
| `distance_weight`, `collision_weight` | `weight_distance_error_penalty`, `weight_collision_penalty` |
| `terminal_weight`, `control_change_weight` | `weight_terminal_state_penalty`, `weight_cmd_change` |
| `use_sg_filter`, `sg_filter_half_window_size`, `sg_filter_poly_order` | same names |

The wheel speeds of a differential drive are a linear function of `(v, omega)`,
so `control_change_weight` also covers `weight_vehicle_cmd_change`.

## Cost for a unicycle on a recorded route

The stage cost keeps the terms of `mppi_3d`: the speed along the reference
direction, the heading error to it, the distance to the path read from the
`distance_error` layer of the reference costmap, the collision cost and the
change of the inputs. The terminal cost is the squared distance to the goal
state. These parts differ:

- The reference direction points from the predicted pose to the route
  `2 * xy_goal_tolerance` ahead of its nearest route point, in path order. The
  `ref_yaw` layer holds the direction of the nearest segment of the whole
  recording. It is ambiguous where the recording passes a place twice, and it
  jumps at a corner recorded as a turn in place, where a unicycle would stop.
- The goal state of the terminal cost is the route position that the reference
  speed reaches within the horizon. Near the end of the route it is the goal.
- Within `xy_goal_tolerance` of the goal the heading error refers to the goal
  yaw, and the robot turns in place until it is within `yaw_goal_tolerance`.
- The inputs keep `v` between zero and `max_linear_speed`, `omega` within
  `max_angular_speed` and both wheels within `wheel_speed_limit`.
- With `wheel_speed_floor` above zero, a wheel either stands still or turns at
  the floor speed or faster. A sensorless ESC starts a wheel at its start
  speed and reverses it only from rest, so slower or alternating wheel
  commands are never sent. The robot drives forward with both wheels at the
  floor or faster, which limits the turn rate to
  `(v - wheel_speed_floor) / (track_width / 2)`. When the route ahead is more
  than 45 degrees off the heading, and at the goal yaw, it stops both wheels,
  turns in place and stops again before it drives on.
- A predicted pose on a lethal or unknown collision cell rejects its sample.
  The action aborts when every sample is rejected.

## Actuator response

By default the rollouts start at the latest pose and move the robot at the input speeds. A
drive that follows its commands late makes such a plan steer too long, and the heading swings
from side to side. Three parameters describe how the tangential speed of each wheel follows a
sent command. All are zero by default, which plans as before.

| Parameter | Unit | Meaning |
| --- | --- | --- |
| `actuator_dead_time` | s | Time from a sent command to the first reaction of the wheels |
| `wheel_accel_limit` | m/s^2 | Rate at which the drive ramps a wheel to a new command |
| `actuator_time_constant` | s | First-order lag of the wheel speed behind that ramp |

- The node hands every command it publishes to the solver, the stops outside a solve included.
  Before a solve the solver replays them through the response, from the time of the pose to
  the time its new command reaches the wheels, and starts the rollouts at the pose and wheel
  speeds this gives. The chassis stops on a command older than `command_timeout` (0.30 s), and
  so does the replay.
- `compensate_pose_age: true` takes the TF stamp as the time of the pose, so the replay also
  covers the delay of localization. Otherwise the pose counts as current.
- With a time constant or an acceleration limit, the rollouts move the wheels through the same
  response. The speed cost then uses the speed the robot reaches. The costs of the inputs stay
  on the inputs.
- The route progress, the goal and the stops and turns in place still follow the measured
  pose, because a wrong prediction must not skip a corner or end the route early.

Take the three values from recorded wheel speeds, not from the drive configuration. If the
acceleration limit is uncertain, use the lower value: a limit that is too high leaves part of
the swing, and one that is too low only makes the plan more cautious.

## Visualization

While a route is followed, the node publishes in the `map` frame:

| Topic | Type | Content |
| --- | --- | --- |
| `/mpc/predicted_pose` | `geometry_msgs/msg/PoseStamped` | Pose the rollouts start from, stamped with the time the command reaches the wheels |
| `/mpc/optimal_traj` | `visualization_msgs/msg/MarkerArray` | Predicted poses of the optimal inputs as red arrows |
| `/mpc/sampled_traj` | `visualization_msgs/msg/MarkerArray` | The 100 sampled trajectories of lowest cost as blue lines |
| `/mpc/calc_time` | `std_msgs/msg/Float32` | Duration of the last solve in milliseconds |

The marker arrays are built only while they have a subscriber.
