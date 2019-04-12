# we_r2_moveit_config arduino example

Prerequisites:
 - [rosserial](https://github.com/ros-drivers/rosserial)
 - (i.e. ```sudo apt-get install -y ros-melodic-rosserial ros-melodic-rosserial-python ros-melodic-rosserial-server```)

Steps to launch:
 - Install we_r2_moveit_config package
 - Launch ```roslaunch we_r2_moveit_config demo_hardware_simplejoints.launch port:=/dev/ttyACM0```
