cd ../..
source /opt/ros/jazzy/setup.bash
cd ~/Intelligent-Robotics/ws_group_number_4_assignments && 
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && 
source install/setup.bash