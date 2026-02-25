from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_gazebo_pkg = FindPackageShare('robot_gazebo')

    slam_params = PathJoinSubstitution([
        robot_gazebo_pkg,
        'config',
        'slam_toolbox_mapping.yaml'
    ])

    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    return LaunchDescription([
        slam_node,
    ])

