#!/bin/zsh
<<<<<<< HEAD

=======
# build the packages
check_dir=..
entries=("$check_dir"/*/)
if [[ ${#entries[@]} -eq 1 ]]; then
    cd ..
    # git clone the packages
    git clone https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git
    git clone https://github.com/Livox-SDK/livox_ros_driver2
    colcon build --packages-select livox_ros_driver2
fi
>>>>>>> 53ee389 (Radio init)
# build all the packages
colcon build
# source the setup file
source install/setup.zsh

