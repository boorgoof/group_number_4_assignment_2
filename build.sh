cd ../..
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 && 
source install/setup.bash