from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Sahte sensörleri başlatır: /line_error, /imu/data, /gps/fix"""
    fake_sensors_node = Node(
        package='robot_gazebo',
        executable='fake_sensors.py',
        name='fake_sensors',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([fake_sensors_node])
