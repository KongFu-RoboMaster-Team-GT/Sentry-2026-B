import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    run_path = os.getcwd()
    
    # ---------------- 配置区域 ----------------
    # 1. 模型路径 (用于存放 SDF/URDF 模型)
    model_path = run_path + '/src/rm-sim-bringup/models'
    model_dir = 'robot_fort'
    model_name = 'robot_translated.sdf' # 机器人的 SDF 文件
    sdf_file = os.path.join(model_path, model_dir, model_name)
    
    # 2. World 文件路径 (重点：指向你的 .world 文件)
    # 假设你的 .world 文件也在同一个 models 目录下，或者你可以指定具体路径
    world_path = run_path + '/src/rm-sim-bringup/worlds' # 常见的存放位置
    world_file_name = 'arena3v3.world' # 请替换为你实际的 .world 文件名
    world_file_path = os.path.join(world_path, world_file_name)

    # ---------------- 环境变量 ----------------
    # 这是为了让 Gazebo 能找到 meshes (STL, DAE) 文件
    # 注意：如果你的 .world 文件里引用了模型，这里也需要包含 world 文件所在的路径
    # 或者确保模型都在 ~/.gazebo/models 里
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', # 推荐使用 GZ_ 前缀 (Fortress)
        value=[
            os.environ.get('GZ_SIM_RESOURCE_PATH', '') + ':' + model_path
            # 这里把 model_path (即 models 的父目录) 加进去，这样 Gazebo 就能识别 model://robot_fort
        ]
    )

    # ---------------- 启动 Gazebo ----------------
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            # 关键修改：直接加载 .world 文件
            # -r 表示启动时自动运行，不需要手动在 GUI 上点 Play
            'gz_args': f'-r {world_file_path}' 
        }.items(),
    )

    # ---------------- Spawn 机器人 (可选) ----------------
    # 如果你的 .world 文件里已经包含了机器人模型，下面的 spawn 可以注释掉
    # 如果 .world 只是场景，你需要通过 ROS 来 spawn 机器人，则保留
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

    # ---------------- ROS-Gazebo Bridge ----------------
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
        # spawn_entity, # 根据上面的注释决定是否启用
        bridge,
    ])