from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_gui = LaunchConfiguration('use_gui')
    use_demo = LaunchConfiguration('use_demo')
    pkg_share = FindPackageShare('hr300_description')

    model_file = PathJoinSubstitution([pkg_share, 'urdf', 'my_robots_super_model.xacro'])
    xacro_cmd = Command([
        FindExecutable(name='xacro'),
        TextSubstitution(text=' '),
        model_file,
    ])
    robot_description = ParameterValue(xacro_cmd, value_type=str)
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'my_robot_config.rviz'])

    return LaunchDescription([
        # The hardware driver owns /joint_states, so synthetic publishers are
        # disabled by default.
        DeclareLaunchArgument('use_gui', default_value='false'),
        DeclareLaunchArgument('use_demo', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 50.0,
            }],
        ),
        Node(
            condition=IfCondition(use_demo),
            package='hr300_description',
            executable='motion_demo_node',
            name='motion_demo',
            output='screen',
            parameters=[{
                'joint_names': ['joint1', 'joint2', 'joint3', 'joint4'],
                'positions': [0.5, -0.2, 1.0, 0.0],
            }],
        ),
        Node(
            condition=IfCondition(use_gui),
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
