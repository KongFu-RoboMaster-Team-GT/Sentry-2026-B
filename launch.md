# install gazebo fortress:

```
sudo apt-get install ros-${ROS_DISTRO}-ros-gz\n
 sudo apt install ros-humble-gazebo-ros-pkgs \n
```


before running script run `export IGN_GAZEBO_RESOURCE_PATH=$IGN_GAZEBO_RESOURCE_PATH:$(model_asset_dir)`

`model asset dir` is where stl file lies in

to run simulation use `ros2 launch rm_sim_bringup fortress_launch.py`

before running don't forget `colcon build`

