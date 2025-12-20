script_dir=$(dirname "$0")
cd "$script_dir"/..
echo $PWD
check_dir=.

if [ -d "$check_dir/livox_ros_driver2" ]; then
    echo "livox_ros_driver2 exists"
else
    echo "livox_ros_driver2 does not exist"
fi

if [ ! -d "$check_dir/livox_laser_simulation_RO2" ]; then
    echo "livox_ros_driver exists"
else
    echo "livox_ros_driver does not exist"
fi