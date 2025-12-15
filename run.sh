#!/bin/zsh

# build all the packages
# source the setup file
source install/setup.zsh
# run the launch file
if [[ $# -gt 0 ]]; then {
    ros2 launch rm_sim_bringup $1
} else {
    ros2 launch rm_sim_bringup fortress_launch.py
}
fi