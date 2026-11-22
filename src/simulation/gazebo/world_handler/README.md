# FWIDS Gazebo world

Launch the four-wheel independent drive and steering model on Gazebo Harmonic.

```bash
ros2 launch world_handler launch_gazebo_world_with_fwids.launch.py gazebo_world_name:=maze
```

The worlds are `empty`, `empty_garden`, `cylinder_garden` and `maze`.
The bridge publishes `/clock`, `/groundtruth_odom`, `/laser_link/scan` and
`/fwids/joint_states` to ROS 2. It forwards the steering and rotor commands
from the `vel_driver` node to Gazebo.

Use `ros2 launch nullspace_mpc_bringup gazebo_world.launch.py` to include the
velocity driver and joystick. Set `show_gazebo_gui:=false open_rviz:=false
gazebo_headless:=true` for headless simulation.
