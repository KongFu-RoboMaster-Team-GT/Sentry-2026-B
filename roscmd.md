
# ros2 knowledge base

# ros system structure


ros2: distribution system, don't have a main nodes. it exist on your disk, has a server framework:
below is ros2 mainframe structure
before using remember to `source /opt/ros/humble/setup.sh` to locate ros2 executable
/opt/ros/humble/
├── bin/       # 可执行文件
├── include/   # 头文件
├── lib/       # 库文件
├── share/     # 配置文件和文档


and user defined node: you put all your nodes/modules/packages here
it can exist anywhere on your computer.
~/ros2_user_space/
├── src/     # 源代码目录
├── build/   # 编译输出目录
├── install/ # 安装目录
└── log/     # 日志文件

before running node run ` source ~/ros2_ws/ws_livox/install/setup.zsh` to find the executable/launch script in `install`
change above dir into your ros user worksapce path!
usually the ros userspace looks like above structure.
In this article we called `ros2_user_space` as ros2 userspace/workspace

# ros package structure

ros concept: package -> nodes
packages contains serval nodes. nodes are actual robot components/server. 
for example: fire-control is a node, radar is a node, ai module is also a node.

ros2 package directory example:
my_ros2_package/
|-----setup.py             # this scripts copy resources when you install your package
├── CMakeLists.txt         # CMake 配置文件
├── package.xml            # ROS2 包的元数据 (this indicates meta info of your package)
├── src/                   # 源代码目录
│   └── my_node.cpp        # 例如驱动节点的实现
├── include/               # 头文件目录
│   └── my_ros2_package/   # 包含 ROS2 节点相关的头文件
│       └── my_node.hpp
├── msg/                   # 自定义消息定义目录
│   └── PointCloudMsg.msg   # 例如自定义的点云消息
├── srv/                   # 自定义服务定义目录
│   └── StartCapture.srv   # 例如控制雷达采集的服务
├── launch/                # 启动文件目录
│   └── livox_driver_launch.py  # 启动文件，用于启动 ROS2 节点
├── config/                # 配置文件目录
│   └── livox_config.yaml  # 雷达配置文件
└── log/                   # 日志文件目录


a live example: assume we already have serval nodes deployed
~/ros2_ws/
├── build/                    # 编译输出目录，存放 colcon 编译过程中的中间文件
├── install/                  # 存放编译后的可执行文件、库、消息和其他安装文件
│   ├── lib/                  # 可执行文件、库文件目录
│   │   ├── livox_ros2_driver/  # `livox_ros2_driver` 包的可执行文件
│   │   ├── navigation2/         # `navigation2` 包的可执行文件
│   │   └── ...                 # 其他包的可执行文件
│   ├── share/                # 包资源、配置文件等
│   │   ├── livox_ros2_driver/  # `livox_ros2_driver` 包的配置文件
│   │   ├── navigation2/         # `navigation2` 包的资源文件
│   │   └── ...                 # 其他包的资源文件
│   └── ...                    # 其他与包相关的文件
├── log/                      # 存放 ROS2 运行时日志
├── src/                      # 源代码目录，包含你开发的 ROS2 包
│   ├── livox_ros2_driver/     # 你自定义的 `livox_ros2_driver` 包源码
│   │   ├── CMakeLists.txt     # CMake 构建文件
│   │   ├── package.xml        # ROS2 包描述文件
│   │   ├── src/               # 包含源代码的子目录
│   │   │   └── livox_driver.cpp  # 节点源代码文件
│   │   ├── include/           # 头文件目录
│   │   └── launch/            # 启动文件
│   │       └── livox_driver_launch.py  # 启动文件，启动 livox_ros2_driver 节点
│   ├── navigation2/           # `navigation2` 包源码
│   │   ├── CMakeLists.txt
│   │   ├── package.xml
│   │   ├── src/               # `navigation2` 源代码
│   │   ├── include/           # 头文件目录
│   │   └── launch/            # 启动文件目录
│   │       └── navigation_launch.py  # 启动文件
│   ├── rclcpp/                # ROS2 C++ 客户端库源码
│   ├── sensor_msgs/           # `sensor_msgs` 包源码
│   └── ...                    # 其他包的源码
├── install/                  # 安装后可执行文件和依赖包
└── ...                       # 其他与 ROS2 工作空间相关的文件



## ros2 pkg install

after installing ros2 to your ubuntu, we bootstrap our robot like using terminal + nohop + `ros2 launch xxxnode`

if building a package failed with msg like `cmake fail to find_package pcl`
run `export PCL_DIR=/usr/lib/x86_64-linux-gnu/cmake/pcl` then build again
colcon is a bigger cmake wrapper, under apt system cmake can't find pcl path, use above command
then run `colcon build`
Warning: your terminal must under ros userspace to run above command.

# ros2 command
ros2 command list
```
ROS2 常用指令速查：

启动:
  ros2 run <pkg> <exe>
  ros2 launch <pkg> <launch.py>

节点:
  ros2 node list
  ros2 node info <node>

Topic:
  ros2 topic list
  ros2 topic info <topic>
  ros2 topic echo <topic>
  ros2 topic hz <topic>
  ros2 topic bw <topic>

参数:
  ros2 param list
  ros2 param get <node> <param>
  ros2 param set <node> <param> <value>

Service:
  ros2 service list
  ros2 service call <srv> <type>

Action:
  ros2 action list

Package:
  ros2 pkg list
  ros2 pkg executables <pkg>

Bag:
  ros2 bag record <topics>
  ros2 bag play <folder>

诊断:
  ros2 doctor

```
