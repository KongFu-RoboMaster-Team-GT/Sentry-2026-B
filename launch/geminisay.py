import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    print('# 1. 机器人状态发布器 (RSP)')
    # 使用 Command 启动 Xacro 进程，定义 robot_description 参数
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': Command([
                'xacro ', 
                os.path.join(
                    get_package_share_directory('rm_sim_bringup'), 
                    'models', 
                    'minimal', 
                    'minimal_test.xacro'
                )
            ])
        }],
        output='screen'
    )

    print('# 2. 启动 Gazebo Fortress 仿真环境')
    # 使用 ros_gz_sim 启动器，加载 arena3v3.world (来自 L2 配置)
    # [報錯]
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # -r: 自动运行
            # world path: 使用 L2 指定的 arena3v3.world
            'gz_args': [
                ' -r ', 
                'empty.stl'
                # os.path.join(
                #     get_package_share_directory('rm_sim_bringup'), 
                #     'models', 
                #     'empty.stl'
                # )
            ]
        }.items(),
    )

    # 3. 在 Gazebo 中 Spawn 机器人
    # 使用 ros_gz_sim/create，通过订阅 /robot_description 话题来加载模型
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'minimal_test_chassis',
            '-x', '0.0', 
            '-y', '0.0', 
            '-z', '0.5'
        ],
        output='screen'
    )

    # 4. ROS-Gazebo 时钟桥接
    # 确保 ROS 时钟和仿真时钟同步 (来自 L1 配置)
    clock_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        # 顺序执行：
        # 启动 Gazebo
        gazebo_sim,
        # 启动 RSP，发布 URDF
        robot_state_publisher_node,
        # 启动时钟桥
        clock_bridge_node,
        # 实例化机器人
        spawn_entity_node,
    ])
