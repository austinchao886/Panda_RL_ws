rm -rf build/ install/ log/
colcon build --packages-select robot_description
source install/setup.bash

ros2 launch robot_description display_gazebo.launch.py  z:=1.0