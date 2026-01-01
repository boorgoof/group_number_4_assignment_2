cd ../..
pkill -9 ros2; pkill -9 gzserver; pkill -9 gzclient; pkill -9 ruby; killall -9 gz; pkill -9 move_group
source install/setup.bash &&
ros2 launch g4_launch assignment_2.launch.xml 
