from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hr300_description',
            executable='fk_service',
            name='fk_service'
        )
    ])