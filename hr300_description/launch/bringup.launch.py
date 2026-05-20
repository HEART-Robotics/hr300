from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, FindExecutable, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    pkg_share = FindPackageShare('hr300_description')

    model_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_robots_super_model.xacro'])

    xacro_cmd = Command([
    FindExecutable(name='xacro'),
    TextSubstitution(text=' '),
    model_file
])
    
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'my_robot_config.rviz'])

    # robot_description = Command(['xacro ', model_file])
    # xacro_cmd = Command([FindExecutable(name='xacro'), model_file])
    robot_description = ParameterValue(xacro_cmd, value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description,
                         'publish_frequency': 50.0}]
        ),
        Node(
            package='hr300_description',
            executable='motion_demo_node',
            name='motion_demo',
            output='screen',
            parameters=[{
                'joint_names': ['joint1', 'joint2', 'joint3'],
                'positions': [0.5, -0.2, 1.0]
            }]
        ),
        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            condition=UnlessCondition(use_gui),
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'robot_description': robot_description,
                 'rate': 50}]  # опционально задай частоту публикации
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        )
    ])
