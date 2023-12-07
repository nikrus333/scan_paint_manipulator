from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='paint_lidar',
        #     namespace='paint_wall1',
        #     executable='paint_wall',
        #     name='paint_wall1'
        # ),
        Node(
            package='paint_lidar',
            namespace='nozzle_close_open1',
            executable='nozzle_close_open',
            name='nozzle'
        ),
        Node(
            package='mcx_ros',
            namespace='execute_trajectory_action1',
            executable='execute_trajectory_action.py',
            name='trajectory_action'
        ),
    ])