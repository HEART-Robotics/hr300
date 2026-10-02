# hr300_description

This package contains URDF model of 4DOF robot-manipulator HR-300.
Also the Rviz2 config added into bringup file.


> [!NOTE]
> This is ROS 2 (Jazzy) ament_python package
> Ubuntu 24.04 is tested 

## Launch with the current HR-300 firmware

Build the description together with the serial driver:

```bash
colcon build --symlink-install --packages-select hr300_interfaces hr300_ros2_driver hr300_description
source install/setup.bash
```

Start the serial driver (change the port when needed):

```bash
ros2 launch hr300_ros2_driver arm_driver.launch.py port:=/dev/ttyACM0
```

In a second terminal, source the workspace and start the robot model and RViz:

```bash
ros2 launch hr300_description bringup.launch.py use_gui:=false
```

`use_gui:=false` is essential for hardware: `/joint_states` then comes only from
the serial driver. `use_gui:=true` is for offline RViz simulation. The optional
static demo is disabled by default and can be enabled with `use_demo:=true`.


