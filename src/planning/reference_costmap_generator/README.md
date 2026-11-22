# Reference costmap generator

The node subscribes to `/recorded_path` and publishes a transient-local
`/reference_costmap` as `grid_map_msgs/msg/GridMap`. The grid uses the path's
coordinate frame and timestamp. Its bounds cover the recorded segments and the
configured distance margin.

- `distance_error` gives the distance in meters from each cell center to the
  nearest segment, capped at `distance_limit`.
- `ref_yaw` gives the direction of the nearest segment in radians. Cells beyond
  the distance limit and stationary routes have an undefined direction.

The default cell resolution is 0.05 m and the distance limit is 0.60 m. Set
`reference_costmap_resolution` and `reference_costmap_distance_limit` in the
workspace navigation launch to change them.
