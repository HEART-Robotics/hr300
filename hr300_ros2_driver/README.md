# hr300_ros2_driver

TCP - tool center point. 

Structure:

- Publishes `/joint_states`
- Parameters: `port` (string), `baudrate` (int), `update_rate` (float), `joint_names` (string[])
- Services (same interface):
  - `/arm/enable_motors` (`std_srvs/SetBool`)
  - `/arm/enable_pnevmo` (`std_srvs/SetBool`)
  - `/arm/go_home` (`std_srvs/Trigger`)
  - `/arm/conveyer_on|off` (`std_srvs/Trigger`)
  - `/arm/set_joints|set_p_gains|set_i_gains|set_d_gains` (`hr300_ros2_driver/SetValues`)
  - `/arm/get_absolute_angles|get_relative_angles|get_target_angles|get_status|get_pid_gains` (`std_srvs/Trigger`, string message with comma-separated values)



> [!NOTE]
> hr300_interfaces ROS package is required!
> This is ROS 2 (Jazzy) ament_python package
> Ubuntu 24.04 is tested 

## How to use
> [!Warning]
> This package is working with hardware connected to USB and proper port number must be provided
> check tty with ``` ls /dev/ttyACM* ``` or ``` dmesg -w ```

Run following command on first console and don't close it. Open a second console and call suitable services form EXAMPLE section.
```
ros2 launch hr300_ros2_driver arm_driver.launch.py port:=/dev/ttyACM0 baudrate:=115200 update_rate:=20.0 \
  joint_names:="['joint1','joint2','joint3','joint4']"
```

## Examples of control commands to hardware

1. Enable Motors
```
ros2 service call /arm/enable_motors std_srvs/srv/SetBool "{data: true}"
```
2. Disable motors
```
ros2 service call /arm/enable_motors std_srvs/srv/SetBool "{data: false}"
```
3. Move to Home Position
```
ros2 service call /arm/go_home std_srvs/srv/Trigger "{}"
```
5. Set Joint Angles (degrees, 4 values)
```
ros2 service call /arm/set_joints hr300_interfaces/srv/SetValues "{values: [0.0, 45.0, -30.0, 0.0]}"
```

6. Get Current Angles

```
# Absolute
ros2 service call /arm/get_absolute_angles std_srvs/srv/Trigger "{}"

# Relative
ros2 service call /arm/get_relative_angles std_srvs/srv/Trigger "{}"

# Target
ros2 service call /arm/get_target_angles std_srvs/srv/Trigger "{}"
```

7. Get Status 

```
ros2 service call /arm/get_status std_srvs/srv/Trigger "{}"
```

8. Subscribe to Joint States in Real Time

```
ros2 topic echo /joint_states
```

9. Get Status and PID Gains

```
ros2 service call /arm/get_pid_gains std_srvs/srv/Trigger "{}"
```

10. Set PID Gains

```
# P-gains for each joint (Position control loop)
ros2 service call /arm/set_p_gains hr300_interfaces/srv/SetValues "{values: [10.0, 10.0, 10.0, 10.0]}"

# I-gains
ros2 service call /arm/set_i_gains hr300_interfaces/srv/SetValues "{values: [0.02, 0.02, 0.02, 0.02]}"

# D-gains
ros2 service call /arm/set_d_gains hr300_interfaces/srv/SetValues "{values: [0.1, 0.1, 0.1, 0.1]}"

```

11. Pneumatic Control

```
# Turn pneumatics ON
ros2 service call /arm/enable_pnevmo std_srvs/srv/SetBool "{data: true}"

# Turn pneumatics OFF
ros2 service call /arm/enable_pnevmo std_srvs/srv/SetBool "{data: false}"

```

10. Conveyor Control

```
ros2 service call /arm/conveyer_on std_srvs/srv/Trigger "{}"
ros2 service call /arm/conveyer_off std_srvs/srv/Trigger "{}"

```

11. set_tcp_shift

```
ros2 service call /arm/set_tcp_shift hr300_interfaces/srv/SetValues "{values: [10.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

```
12. TCP POSITION for current config (Forward kinematics)

```
ros2 service call /arm/get_fkin std_srvs/srv/Trigger "{}"

```
13. Joint angkes for given POSITION and ORIENTATION OF TCP (Inverse kinematics)

```
ros2 service call /arm/get_limits std_srvs/srv/Trigger "{}"

```
14. Update motion LImits. Max velocity, Max accel, joint controller tolererance, max current (TORQUE CONTROL IS NOT SUPPORTED FOR HR300)

```
ros2 service call /arm/set_limits hr300_interfaces/srv/SetValues "{values: [55.0, 100.0, 1.0, 0.0, 0.0, 0.0]}"

```
15. 

```
ros2 service call /arm/get_offsets std_srvs/srv/Trigger "{}"

```
16. 

```
ros2 service call /arm/set_offsets hr300_interfaces/srv/SetValues "{values: [38.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

```