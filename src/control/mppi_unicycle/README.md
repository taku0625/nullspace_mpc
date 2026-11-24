# Unicycle MPPI

The recorded-route controller takes a `FollowPath` action and a matching
`/reference_costmap` from the Misachiya navigation launch. It uses live
`map → base_link` TF and publishes `geometry_msgs/msg/TwistStamped` to
`/cmd_vel`. The action server stops the chassis on completion, cancellation,
stale localization or collision data, and loss of route progress. With a
collision costmap it leads the robot around obstacles on the route.

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
  Without a collision costmap the action aborts when every sample is rejected.

## Obstacles

`collision_costmap_topic` names a `nav_msgs/msg/OccupancyGrid` with costs from 0 to 100. A
cell of 99 or more, an unknown cell and everything outside the grid are lethal. The other
costs count as `collision_weight * cost / 100` in every step of a rollout. Without the topic
the controller does not look for obstacles.

The original `mppi_3d` read the local costmap of `move_base`, which inflated the obstacles by
the radius of the robot, and it followed a path that a global planner had already laid around
them. Here the route is a recording, and nothing replans it. Three parts replace that:

- **Body.** `collision_offsets` lists distances along the body axis from the base frame. The
  costmap is looked up at each of these points of a predicted pose, and the largest cost
  counts. The costmap has to be inflated by the radius of a circle around each point, so that
  the circles cover the body. The default `[0.0]` looks only at the base frame.
- **Detour.** The collision cost alone does not take the robot around an obstacle on the
  route. Slowing down costs less than leaving the route, and at the wheel speed floor the
  robot cannot turn, so it stops in front of the obstacle. Before each solve the route up to
  `detour_range` ahead of the robot is therefore sampled every 0.1 m, and each sample may
  move sideways in steps of 0.05 m up to `detour_max_offset`. The cheapest sequence of
  offsets is followed instead of the recorded route: an offset costs its size, a cell with
  collision cost is charged steeply towards the lethal cost, lethal cells are not entered,
  and the offset changes by one step per sample at most, which gives ramps of 1 in 2. A
  term for the distance from the detour of the previous solve keeps the side once chosen.
  The detour starts beside the robot, at an offset the robot can reach sideways without
  crossing a lethal cell, so it never continues behind a wall the robot is passing.
  The detour passes outside the graded cost when there is room, through the middle of a
  gap that is too narrow for that, and around a gap that is narrower still. It returns to
  the route behind the obstacle and ends on the recorded goal. Where no way exists it leads
  as far as the robot can go. Without cost on the route the route is followed as recorded,
  and the commands are the same as without a costmap. Either parameter at zero switches the
  detours off.
- **Blocked route.** When every sample is rejected, the robot stops and waits for the
  obstacle to leave. The action aborts with `Obstacles kept the recorded route blocked`
  when the robot has not moved for `stall_timeout` (8 s). The warning while it waits and the
  abort message say whether the robot itself stands within the margin of an obstacle or
  every sampled trajectory runs into one ahead.

While a detour is followed, the distance cost refers to the detour instead of the
`distance_error` layer, and the stops and turns in place refer to it as well. The route
progress and the goal still follow the recorded route.

The costmap must reach `hypot(detour_range + largest offset of collision_offsets,
detour_max_offset)` from the robot, or the part of a detour outside it counts as blocked. The
node warns when it is smaller. It also warns when `reference_speed` leaves less than
0.3 rad/s between the wheel speed limit and the wheel speed floor: the robot then has to slow
down for every turn and steers late around obstacles.

## Actuator response

By default the rollouts start at the latest pose and move the robot at the input speeds. A
drive that follows its commands late makes such a plan steer too long, and the heading swings
from side to side. Four parameters describe how the tangential speed of each wheel follows a
sent command. All are zero by default, which plans as before.

| Parameter | Unit | Meaning |
| --- | --- | --- |
| `actuator_dead_time` | s | Time from a sent command to the first reaction of the wheels |
| `wheel_accel_limit` | m/s^2 | Rate at which the drive ramps a wheel up to a faster command |
| `wheel_decel_limit` | m/s^2 | Rate at which it ramps a wheel down. Zero takes `wheel_accel_limit` |
| `actuator_time_constant` | s | First-order lag of the wheel speed behind that ramp |

The two limits differ on the Misachiya chassis: its ESCs speed a wheel up within their own
limit and let it slow down as fast as the host asks. A wheel that changes direction slows
down to rest within the one limit and speeds up within the other.

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

Take the values from recorded wheel speeds, not from the drive configuration. If a limit is
uncertain, use the lower value: a limit that is too high leaves part of the swing, and one
that is too low only makes the plan more cautious.

## Visualization

While a route is followed, the node publishes in the `map` frame:

| Topic | Type | Content |
| --- | --- | --- |
| `/mpc/predicted_pose` | `geometry_msgs/msg/PoseStamped` | Pose the rollouts start from, stamped with the time the command reaches the wheels |
| `/mpc/optimal_traj` | `visualization_msgs/msg/MarkerArray` | Predicted poses of the optimal inputs as red arrows, and the detour that is followed as a yellow line (namespace `detour`) |
| `/mpc/sampled_traj` | `visualization_msgs/msg/MarkerArray` | Every sampled trajectory: blue for the samples that count, red for the ones rejected on a lethal cell |
| `/mpc/calc_time` | `std_msgs/msg/Float32` | Duration of the last solve in milliseconds |

The marker arrays are built only while they have a subscriber.

The sampled trajectories are two `LINE_LIST` markers, one per color. Each sample is drawn
with `sampled_trajectory_points` segments (5) from the start of the rollouts to equally spaced
poses of its prediction, and the markers are published every `visualization_interval` (0.2 s).
With 512 samples this is 123 kB per message and 0.6 MB/s. One `LINE_STRIP` marker per sample
at the control rate takes 8.7 MB/s for all samples, and took 1.7 MB/s for the 100 best ones
that were published before.
