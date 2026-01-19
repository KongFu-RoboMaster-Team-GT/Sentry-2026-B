import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. 定义 DWA 节点的参数
    # 你可以在这里轻松修改参数，而不用动 Python 代码
    dwa_params = {
        'max_speed': 0.6,
        'max_yaw_rate': 1.2,
        'predict_time': 2.0,
        'robot_radius': 0.3,
        'v_resolution': 0.05,
        'w_resolution': 0.1
    }

    # 2. 声明 DWA 节点
    dwa_planner_node = Node(
        package='a_star',           # 你的包名
        executable='a_star_node',  # 在 setup.py 中定义的 entry_point
        name='dwa_planner',
        output='screen',
        parameters=[dwa_params],
        remappings=[
            ('/fastlio2/lio_odom', '/fastlio2/lio_odom'),         # 如果你的 odom 话题名字不同，可以在这里重映射
            ('/cmd_vel', '/cmd_vel')
        ]
    )

    # 3. (可选) 启动 Rviz2 用于可视化
    rviz_config_dir = os.path.join(get_package_share_directory('a_star'), 'rviz', 'dwa_config.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    return LaunchDescription([
        dwa_planner_node,
        rviz_node
    ])
