from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('param_file', default_value='config/purepursuit_params.yaml', description='Path to parameter file'),

        Node(
            package='purepursuit_k',
            executable='purepursuit',
            name='Pure_Pursuit_Node',
            output='screen',
            parameters=[{'LAD': 2.0, 'WHEELBASE': 3.0}],
            remappings=[('/local_planned_path', '/local_planned_path')]
        )
    ])

