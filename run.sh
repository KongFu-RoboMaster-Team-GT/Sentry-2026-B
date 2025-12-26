if [ /bin/zsh = "$(which zsh)"]; then
source install/setup.zsh
else
source install/setup.bash
fi
ros2 launch livox_ros_driver2 rviz_MID360_launch.py