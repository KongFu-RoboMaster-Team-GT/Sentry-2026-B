# test_robot_model.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # ============================== 配置参数 (请根据实际情况修改) ==============================
    
    bringup_pkg_name = 'rm_sim_bringup'          # 当前 Launch 文件所在的包名
    robot_desc_pkg_name = bringup_pkg_name        # 你的 infantry.urdf 所在的包名
    world_file_name = 'arena3v3.world'           # 你的世界文件名
    robot_xacro_file = 'minimal.xacro'           # 你的机器人模型文件名
    model_dir = 'minimal'
    ros2_control_config = 'ros2_control.yaml'
    
    # ============================== 路径构造 ==============================
    
    bringup_share_dir = get_package_share_directory(bringup_pkg_name)
    print(bringup_share_dir)
    robot_desc_share_dir = get_package_share_directory(robot_desc_pkg_name)

    # 1. 构造世界文件的完整路径 (假设世界文件在 models 目录下)
    world_path = os.path.join(bringup_share_dir, 'models', world_file_name)
    
    # file exist in packagedir/models/car/robot.urdf
#    robot_desc_path = os.path.join(robot_desc_share_dir, 'models','translated_nightbuild', robot_xacro_file) 
    robot_desc_path = os.path.join(robot_desc_share_dir, 'models',model_dir, robot_xacro_file) 

    # ... 在 launch.py 中构造 ros2_control 配置文件的绝对路径
    ros2_control_config_path = os.path.join(
        bringup_share_dir, 'models', model_dir, 'config', ros2_control_config
    )
    
    # ============================== 节点定义与启动 ==============================
    
    # 1. 启动 Gazebo Server & Client (加载你的世界文件)
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')]),
        # 传递 world 参数给 Gazebo
        launch_arguments={
            'world': world_path,
            'verbose': 'true',
#            'gui' : 'false'
        }.items()
    )

    # 2. 读取 XACRO/URDF 模型文件内容并预处理
    robot_description_content = Command(['xacro ', robot_desc_path,])
#                                         ' ros2_control_config:=', ros2_control_config_path])
    
    # 3. 启动 robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time':True},{'robot_description': robot_description_content}]
    )

    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    parameters=[{'use_sim_time': True}]
)


    # 4. 启动 Gazebo 模型生成节点 (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', 
                   '-entity', 'rm_infantry'], 
        output='screen'
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster']
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller']
    )

    description_launch_py = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'description.launch.py']),
        launch_arguments={
            'urdf_package': 'rm_sim_bringup',
            'urdf_package_path': 'models/minimal/r2d2.xacro'  # 这里改成你的路径
        }.items()
    )

    # ============================== 最终返回 (只包含核心三要素) ==============================
    
    
    return LaunchDescription([
        gazebo,
#        SetParameter(name='use_sim_time', value=True),
#        joint_state_publisher,
        robot_state_publisher_node,
        spawn_entity,
#        diff_drive_spawner,

    ])