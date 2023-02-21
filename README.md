# ASDfR
repository for Advanched Software Development for Robotics


# Assignment 1.1
// Terminal 1 in the main ASDfR/Ass1 directory <br />
colcon build --packages-select image_processing

// Terminal 2 <br />
ros2 run image_tools cam2image --ros-args -p depth:=1 -p history:=keep_last

// Terminal 3 in the main ASDfR/Ass1 directory <br />
. install/local_setup.bash
ros2 run image_processing pub_brightness --ros-args -p threshold:=200 -p assignment:=4

