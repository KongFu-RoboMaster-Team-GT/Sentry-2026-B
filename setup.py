
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rm_sim_bringup'

# --- 新增代码块：递归查找 models 目录下的所有文件 ---
def get_model_data_files():
    """
    递归遍历 'models' 目录，生成 setuptools 所需的数据文件列表。
    格式为: (安装目标路径, [源文件路径列表])
    """
    model_data = []
    base_source_dir = 'models'
    
    # 如果 models 目录不存在，则返回空列表，避免报错
    if not os.path.isdir(base_source_dir):
        return model_data

    # os.walk 会遍历指定目录下的所有子目录和文件
    # root: 当前正在遍历的目录路径
    # dirs: root 目录下的子目录列表
    # files: root 目录下的文件列表
    for root, dirs, files in os.walk(base_source_dir):
        if files:  # 只有当当前目录下有文件时才进行处理
            # 构建安装目标路径: share/package_name/ + 相对于 models 的路径
            relative_path = os.path.relpath(root, base_source_dir)
            install_target_dir = os.path.join('share', package_name, base_source_dir, relative_path)
            
            # 构建源文件路径列表
            source_file_paths = [os.path.join(root, file) for file in files]
            
            model_data.append((install_target_dir, source_file_paths))
            
    return model_data
# --- 新增代码块结束 ---

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # 基础文件
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # 安装所有 Launch 脚本到 launch 目录
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.py'))),
        
        # --- 替换掉原来手动添加的 models 条目 ---
        # 使用上面定义的函数来动态生成 models 的数据文件列表
        *get_model_data_files(),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='phage',
    maintainer_email='phage@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)