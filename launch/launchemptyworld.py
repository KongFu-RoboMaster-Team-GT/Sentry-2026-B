# test_world_only.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # ============================== 配置 ==============================
    pkg_name = 'rm_sim_bringup'
    world_file_name = 'arena3v3.world'

    # ============================== 路径计算 ==============================
    # 获取当前包的安装路径
    pkg_share = get_package_share_directory(pkg_name)
    
    # 获取 gazebo_ros 包的路径 (用于引用官方启动脚本)
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    # 构造世界文件的完整路径
    # 根据你之前的 setup.py，你的 world 文件被安装在 'models' 目录下
    world_path = os.path.join(pkg_share, 'models', world_file_name)

    # ============================== 启动 Gazebo ==============================
    # 包含 gazebo_ros 包自带的 'gazebo.launch.py'
    # 这个脚本会同时启动 gzserver (物理引擎) 和 gzclient (图形界面)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_path,
            'verbose': 'true' # 开启详细日志，方便看报错
        }.items()
    )

    return LaunchDescription([
        gazebo
    ])