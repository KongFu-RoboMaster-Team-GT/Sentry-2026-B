import os
import getpass
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import LogInfo


def generate_launch_description():
    #debug output
    ign_verbosity = SetEnvironmentVariable(
        name='IGN_VERBOSITY',
        value='4' # 4 代表 Debug 级别，会输出大量的查找信息
    )


    run_path = os.getcwd()
    # ---------------- 配置区域 ----------------
    # 这里的路径请根据你的实际情况修改
    # 建议将路径设置为你的工作空间路径或者包路径
    pkgname = 'rm_sim_bringup'
    pkgdir = get_package_share_directory(pkgname)
    model_dir_name = 'robot_fort'
    model_name = 'robot_translated.sdf'
    sdf_file = os.path.join(pkgdir, 'models', model_dir_name, model_name)
    
    # ---------------- 环境变量 ----------------
    # 这一步至关重要：告诉 Gazebo 去哪里找 stl 模型
    # 如果你的 SDF 引用是用 <uri>model://my_robot/meshes...</uri>
    # 这里的路径应该是包含 my_robot 的那个父目录
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + os.path.join(pkgdir, 'models', model_dir_name, 'assets')
        ]
    )


    world_dir_name = 'worlds'
    world_filename = 'arena3v3.world'
    world_file = os.path.join(pkgdir,'models', world_dir_name, world_filename)
    # now exporting stl file location of world sdf. Remember to export stl file path to gazebo for each model files!!!!!!
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '') + ':' + os.path.join(pkgdir, 'models', world_dir_name, 'assets')
        ]
    )

    # ---------------- 启动 Gazebo ----------------
    # 使用 ros_gz_sim 提供的标准启动文件
    # [不報錯]
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world_file]  # -r 表示自动开始运行 (Run)，不需要手动点 Play
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

    logoutput = (
        f"world full path:{world_file}\n"
        f"model full path:{sdf_file}\n"
        f"model package path:{pkgdir}\n"
        f"gazebo_stl_dir:{os.environ.get('IGN_GAZEBO_RESOURCE_PATH')}\n"
    )
    log_world_path = LogInfo(
        msg=[logoutput]
    )


    return LaunchDescription([
        log_world_path,
        ign_verbosity,
        gz_resource_path,
        gz_sim,
        spawn_entity,
        bridge,
    ])