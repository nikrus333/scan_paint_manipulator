from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package='paint_lidar',
            executable='service_nozzle',
            name='nozzle'
        ),
        Node(
            package='canopen_pkg',
            executable='can_node',
            name='can_read_esp'
        ),
        Node(
            package='arrow_pkg',
            executable='can_data_arrow.py',
            name='can_data_arrow_rviz'
        ),
        Node(
            package='arrow_pkg',
            executable='arrow_points.py',
            name='arrow_points'
        ),
    ])
