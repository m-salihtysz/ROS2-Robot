from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')

    config_file = PathJoinSubstitution([
        robot_gazebo_pkg,
        'config',
        'ekf_localization.yaml'
    ])

    # Sahte sensörler: /line_error, /imu/data, /gps/fix (EKF + şerit takibi için)
    fake_sensors_node = Node(
        package='robot_gazebo',
        executable='fake_sensors.py',
        name='fake_sensors',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # NavSat: /gps/fix + /imu/data -> /odometry/gps (EKF odom1 girişi)
    navsat_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform_node',
        output='screen',
        parameters=[config_file],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        fake_sensors_node,
        navsat_node,
        ekf_node,
    ])

