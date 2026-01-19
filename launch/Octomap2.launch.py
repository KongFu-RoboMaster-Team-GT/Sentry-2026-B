from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            parameters=[{
                'Style': 'Squares',
                'use_sim_time': False, # 如果是跑数据集或Gazebo仿真，建议设为True
                'frame_id': 'lidar', # 注意：Fast-LIO2默认全局坐标系通常是 camera_init 或 world
                'base_frame_id': 'lidar',
                'resolution': 0.02,      # 2D地图建议分辨率设大一点（如0.05或0.1），提高渲染和计算性能
                'sensor_model/max_range': 50.0,

                # --- 关键参数：地圖導出 ---
                # 'octomap_path': '/home/kongfu40/workspace/ROS/SLAM/map/my_map.bt',
                
                # --- 关键参数：2D 投影设置 ---
                'filter_ground': True,          # 是否过滤地面
                'occupancy_min_z': 0.2,         # 投影到2D地图时的最小高度（相对于 base_frame_id）
                'occupancy_max_z': 1.5,         # 投影到2D地图时的最大高度（切片高度）
                
                # 可选：概率阈值设置
                'sensor_model/hit': 0.7,
                'sensor_model/miss': 0.4,
                'sensor_model/min': 0.12,
                'sensor_model/max': 0.97,
            }],
            remappings=[
                ('cloud_in', '/fastlio2/world_cloud')
            ]
        )
    ])
