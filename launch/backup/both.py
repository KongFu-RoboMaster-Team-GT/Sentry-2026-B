import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    # ---------------- 参数配置 ----------------
    pkg_name = 'rm_sim_bringup' # 请确保这是你的包名
    pkg_share_dir = get_package_share_directory(pkg_name)
    
    # 1. 模型配置 (结合了 xacro 解析)
    # 这里使用了文件2的 xacro 方式，比直接用 SDF 更灵活
    xacro_file = os.path.join(pkg_share_dir, 'models', 'minimal', 'minimal_test.xacro')
    
    # 使用 Command 调用 xacro 命令将 xacro 转换为 urdf/xml，供 Gazebo 和 RSP 使用
    robot_description_content = Command(['xacro ', xacro_file])
    
    # 2. 世界配置
    world_file_name = 'arena3v3.world'
    world_path = os.path.join(pkg_share_dir, 'models', world_file_name)

    # ---------------- 环境变量设置 ----------------
    # 关键：告诉 Gazebo Fortress 去哪里找模型资源 (meshes, materials 等)
    # 注意：路径分隔符在 Linux 下是 ':'
    model_source_path = os.path.join(pkg_share_dir, 'models')
    
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            # 获取现有的环境变量值（如果有的话），避免覆盖系统路径
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''), 
            ':', 
            model_source_path
        ]
    )

    # ---------------- 启动 Gazebo 模拟器 ----------------
    # 使用 ros_gz_sim (Fortress 推荐方式)
    gz_sim_share_dir = get_package_share_directory('ros_gz_sim')
    
    # 启动 gz_sim，并传入 world 文件路径
    # '-r' 标志表示加载后立即运行，不需要手动在 GUI 上点播放
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            gz_sim_share_dir, '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={
            'gz_args': f'-r {world_path}' # 加载指定的世界
        }.items(),
    )

    # ---------------- 启动 Robot State Publisher ----------------
    # 解析 URDF 并发布静态 TF (base_link -> camera, base_link -> wheel 等)
    # 这对于 Gazebo 正确识别模型结构是必要的
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True  # 在仿真中必须设为 True
        }]
    )

    # ---------------- 在 Gazebo 中生成机器人 ----------------
    # 使用 ros_gz_sim 的 create 工具 (对应旧版的 spawn_entity.py)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'minimal_test_chassis', # 机器人实体名称
            '-topic', 'robot_description',   # 从哪个 topic 获取 robot_description
            '-x', '0.0',                     # 初始位置 X
            '-y', '0.0',                     # 初始位置 Y
            '-z', '0.5'                      # 初始位置 Z (根据你的底盘高度调整)
        ]
    )

    # ---------------- 可选：启动 RViz2 (方便调试) ----------------
    # 如果你有 rviz 配置文件，可以取消注释下面的代码
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', os.path.join(pkg_share_dir, 'rviz', 'view.rviz')]
    # )

    # ---------------- 返回启动描述 ----------------
    return LaunchDescription([
        # 动作/节点列表
        gz_resource_path,      # 1. 设置环境变量
        gz_sim,                # 2. 启动 Gazebo (包含世界)
        robot_state_publisher, # 3. 发布机器人描述
        spawn_entity,          # 4. 生成机器人到世界中
        
        # rviz,               # 5. (可选) 启动 RViz
    ])