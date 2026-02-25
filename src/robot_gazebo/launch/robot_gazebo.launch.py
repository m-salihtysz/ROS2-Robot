from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Paket yolları
    robot_description_pkg = FindPackageShare('robot_description')
    gazebo_ros_pkg = FindPackageShare('gazebo_ros')

    # URDF (xacro) dosyası
    urdf_file = PathJoinSubstitution([
        robot_description_pkg,
        'urdf',
        'robot.urdf.xacro'
    ])

    # Robot description parametresi
    robot_description = Command(['xacro ', urdf_file])

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Özel dünyamızın yolu
    world_file = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'worlds',
        'robot_course.world'
    ])

    # Gazebo Classic (gazebo_ros) launch - özel dünya ile
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                gazebo_ros_pkg,
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'world': world_file
        }.items()
    )

    # Robotu Gazebo'ya spawn et
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'four_wheel_robot',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    # Keyboard teleop node'u hâlâ ayrı terminalde çalıştırmanız gerekiyor:
    # ros2 run robot_control keyboard_teleop

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
    ])
