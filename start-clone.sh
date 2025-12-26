mkdir src && cd src

git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd ./Livox-SDK2/
mkdir build
cd build
cmake .. && make -j
sudo make install
cd ../..

git clone https://github.com/Livox-SDK/livox_ros_driver2.git ws_livox/src/livox_ros_driver2.git
mv ws_livox/src/livox_ros_driver2 livox_ros_driver2
rm -rf ws_livox
cd livox_ros_driver2
./build.sh humble
cd ../..

mv Mid360_config.json src/livox_ros_driver2/config/MID360_config.json
mv mixed_HAP_MID360_config.json src/livox_ros_driver2/config/mixed_HAP_MID360_config.json

colcon build
