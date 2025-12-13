import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    # 获取当前工作目录
    run_path = os.getcwd()
    pkg_name = 'rm_sim_bringup'
    pkgdir = get_package_share_directory(pkg_name)

    world_file_name = 'arena3v3.sdf'
    world_path = os.path.join(pkgdir, 'world', world_file_name)
    
    # ---------------- 配置区域 ----------------
    # 你可以在这里修改模型路径
    model_path = run_path + '/src/rm-sim-bringup/models'
    model_dir = 'robot_fort'  # 对应你的模型文件夹名
    model_name = 'robot_translated.sdf'
    sdf_file = os.path.join(model_path, model_dir, model_name)
    
    # 🌍 World Path 预留 (根据你的需求)
    # 例如: world_file = os.path.join(run_path, 'src', 'your_pkg', 'worlds', 'my_w orld.sdf')
    world_file = world_path # 默认为空世界，你可以替换成你的 .sdf 世界文件路径

    # ---------------- 环境变量设置 ----------------
    # 关键点：获取当前环境变量值，并追加我们的模型路径
    # 注意：Gazebo Fortress 推荐使用 GZ_SIM_RESOURCE_PATH
    current_env_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    
    # 拼接路径：原有路径 + ':' + 新路径
    # 这里的 assets 文件夹应该包含 meshes, materials 等资源，且文件夹名应为 model_dir (robot_fort)
    new_resource_path = current_env_path + ':' + os.path.join(model_path, model_dir, 'assets') if current_env_path else os.path.join(model_path, model_dir, 'assets')

    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', # 推荐使用 GZ_ 前缀
        value=new_resource_path
    )

    # ---------------- 启动 Gazebo 仿真 ----------------
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            # 如果 world_file 是完整路径，直接使用；如果是名字，Gazebo 会去环境变量路径里找
            'gz_args': f'-r {world_file}' 
        }.items(),
    )

    # ---------------- Spawn 机器人实体 ----------------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'rm_robot',
            '-file', sdf_file, # 使用 SDF 文件路径
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5'
        ],
        output='screen'
    )

    # ---------------- ROS 与 Gazebo 桥接 ----------------
    # 用于同步仿真时钟 /clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        gz_resource_path, # 1. 先设置环境变量
        gz_sim,          # 2. 启动仿真
        spawn_entity,    # 3. 生成机器人
        bridge,          # 4. 桥接
    ])