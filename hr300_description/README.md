# hr300_description

This package contains URDF model of 4DOF robot-manipulator HR-300.
Also the Rviz2 config added into bringup file.


> [!NOTE]
> This is ROS 2 (Jazzy) ament_python package
> Ubuntu 24.04 is tested 

## Launch in command line from workspace folder:

```
colcon build --symlink-install --packages-select hr300_description

source install/setup.bash

ros2 launch hr300_description bringup.launch.py
```


