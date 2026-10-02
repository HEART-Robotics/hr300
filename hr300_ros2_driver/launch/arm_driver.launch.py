from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baudrate", default_value="115200"),
        DeclareLaunchArgument("update_rate", default_value="20.0"),
        DeclareLaunchArgument("pid_joint", default_value="0"),

        Node(
            package="hr300_ros2_driver",
            executable="driver_node",
            name="hr300_driver",
            output="screen",
            parameters=[
                {
                    "port": LaunchConfiguration("port"),
                    "baudrate": LaunchConfiguration("baudrate"),
                    "update_rate": LaunchConfiguration("update_rate"),
                    "pid_joint": LaunchConfiguration("pid_joint"),
                    "joint_names": ["joint1", "joint2", "joint3", "joint4"],
                }
            ],
        )
    ])
