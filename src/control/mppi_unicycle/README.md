# Unicycle MPPI

The recorded-route controller takes a `FollowPath` action and a matching
`/reference_costmap` from the Misachiya navigation launch. It uses live
`map → base_link` TF and publishes `geometry_msgs/msg/TwistStamped` to
`/cmd_vel`. MPPI samples forward speed and angular velocity, limits the target
wheel speeds, and maintains progress in path order at crossings.

The reference costmap contributes distance to the recorded path and the
direction of the nearest segment. A live occupancy grid can supply collision
costs. The action server stops the chassis on completion, cancellation, stale
localization or collision data, and loss of route progress.
