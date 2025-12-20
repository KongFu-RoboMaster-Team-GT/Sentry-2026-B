#!/bin/zsh

# build the packages
script_dir=$(dirname "$0")
cd "$script_dir"/..
echo $PWD
check_dir=.

if [ ! -d "$check_dir/livox_laser_simulation_RO2" ]; then
    git clone https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git
fi
if [ ! -d "$check_dir/livox_ros_driver2" ]; then
    git clone https://github.com/Livox-SDK/livox_ros_driver2.git
    chmod 755 livox_ros_driver2/build.sh
    colcon build --packages-select livox_ros_driver2
fi

# build all the packages
colcon build
# source the setup file
source install/setup.zsh

