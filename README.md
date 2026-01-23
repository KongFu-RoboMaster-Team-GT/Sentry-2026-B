colcon build --packages-select link_base
source install/setup.bash

只需直接在你的 Python 文件 Import 即可使用
# ROS2 Python
from link_base.msg import PublicLidar000Data
msg = PublicLidar000Data()
msg.grid_map = ... # 填入点云
msg.target = ...   # 填入目标