# HR-300 ROS 2 driver

Driver for the current four-joint HR-300 EDU firmware. It uses the framed
`HR**` serial protocol at 115200 baud and publishes the controller's relative
joint positions to `/joint_states` in radians.

## Build and run

```bash
colcon build --symlink-install --packages-select hr300_interfaces hr300_ros2_driver hr300_description
source install/setup.bash
ros2 launch hr300_ros2_driver arm_driver.launch.py port:=/dev/ttyACM0
```

The firmware protocol uses radians × 1000 for rotary joint payloads. The ROS
service `/arm/set_joints` intentionally accepts degrees for convenience and
converts them before sending `HRSC`.

## Main services

```bash
# Enable motors and move four joints in degrees.
ros2 service call /arm/enable_motors std_srvs/srv/SetBool "{data: true}"
ros2 service call /arm/set_joints hr300_interfaces/srv/SetValues "{values: [0.0, 45.0, -60.0, 90.0, 0.0, 0.0]}"

# Set the four target velocities in rad/s (the final two interface slots are ignored).
ros2 service call /arm/set_target_velocities hr300_interfaces/srv/SetValues "{values: [0.5, 0.5, 0.5, 0.5, 0.0, 0.0]}"

# Move the calibrated base linear axis: [position_mm, speed_mm_s, ...].
ros2 service call /arm/move_base_linear_axis hr300_interfaces/srv/SetValues "{values: [100.0, 10.0, 0.0, 0.0, 0.0, 0.0]}"

# Move to the firmware home configuration.
ros2 service call /arm/go_home std_srvs/srv/Trigger "{}"

# Read state, relative/absolute/target joint angles, and TCP pose.
ros2 service call /arm/get_status std_srvs/srv/Trigger "{}"
ros2 service call /arm/get_relative_angles std_srvs/srv/Trigger "{}"
ros2 service call /arm/get_absolute_angles std_srvs/srv/Trigger "{}"
ros2 service call /arm/get_target_angles std_srvs/srv/Trigger "{}"
ros2 service call /arm/get_fkin std_srvs/srv/Trigger "{}"
```

`SetValues` has six slots for interface compatibility; `/arm/set_joints` uses
the first four, in degrees, and ignores the final two. The three joint-angle
readback services return radians. `HRGS` includes the
firmware control-loop frequency, reported by `/arm/get_status` as
`frequency_hz`; it is not a joint angle.

`/arm/set_tcp_shift` accepts `[x_mm, y_mm, z_mm]`. `/arm/set_limits` accepts
`[max_velocity_rad_s, max_acceleration_rad_s2, tolerance_rad, max_current_a]`.
`/arm/set_pid_gains` accepts `[joint_index, kp, ki, kd]`; `/arm/get_pid_gains`
uses the `pid_joint` launch parameter (default `0`).
