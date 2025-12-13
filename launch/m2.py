import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    pkg_name = 'rm_sim_bringup'
    pkgdir = get_package_share_directory(pkg_name)

    world_file_name = 'arena3v3.world'
    world_path = os.path.join(pkgdir, 'models', world_file_name)

    # Xacro
    xacro_file = os.path.join(pkgdir, 'models', 'minimal', 'minimal_test.xacro')

    robot_description = Command(['xacro ', xacro_file])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # Gazebo
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Spawn
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'minimal_test_chassis'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,          # 1. 启动 Gazebo
        rsp_node,        # 2. 启动 robot_state_publisher
        spawn_entity     # 3. spawn model
    ])