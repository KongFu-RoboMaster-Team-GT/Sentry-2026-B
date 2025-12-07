import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------- 配置区域 ----------------
    # 这里的路径请根据你的实际情况修改
    # 建议将路径设置为你的工作空间路径或者包路径
    model_path = '/home/phage/ros2_ws/ws_livox/src/rm_sim_bringup/models' 
    model_dir = 'robot_fort'
    model_name = 'robot_translated.sdf'
    sdf_file = os.path.join(model_path, model_dir, model_name)
    
    # ---------------- 环境变量 ----------------
    # 这一步至关重要：告诉 Gazebo 去哪里找 stl 模型
    # 如果你的 SDF 引用是用 <uri>model://my_robot/meshes...</uri>
    # 这里的路径应该是包含 my_robot 的那个父目录
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + os.path.join(model_path, model_dir, 'assets')
        ]
    )

    # ---------------- 启动 Gazebo ----------------
    # 使用 ros_gz_sim 提供的标准启动文件
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': '-r empty.sdf' # -r 表示自动开始运行 (Run)，不需要手动点 Play
        }.items(),
    )

    # ---------------- Spawn 机器人 ----------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rm_robot',
            '-file', sdf_file,
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ---------------- ROS-Gazebo Bridge (可选但必要) ----------------
    # 这是一个桥接示例，用于将 Gazebo 的时钟信号传给 ROS
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path,
        gz_sim,
        spawn_entity,
        bridge,
    ])