cmake_minimum_required(VERSION 3.8)
project(link_base)

# （官方）官方的玩意
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# --- 补充依赖 ---
  # （自定义）个人补充的几个依赖包，来自官方
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
  # uncomment the following section in order to fill in
  # further dependencies manually.
  # find_package(<dependency> REQUIRED)

# --- 自定义：信息数据类型 ---
  # 字段：
    # Public/Private
    # Lidar/Vision/Body
    # 000/001/002：原始数据/处理或融合数据/最终数据
set(msg_files
  "msg/PublicLidar000Data.msg"
)

# 自定义：创建接口
  # 包名：link_base
  # 消息类型：几个 Msg 文件
  # 依赖：几个官方文件
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES sensor_msgs geometry_msgs
)
# ---

# （官方）官方的玩意
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
