# we_r2_moveit_config

This is a ROS package to provide inverse kinematics and motion planning for LoboCNC's [WE-R2.4 Six-Axis Robot Arm](https://www.thingiverse.com/thing:3327968)

Prerequisites:
 - [ros-melodic](http://wiki.ros.org/melodic)
 - [Moveit!](https://ros-planning.github.io/moveit_tutorials/)

Steps to launch:
 - First, create a catkin workspace: http://wiki.ros.org/catkin/Tutorials/create_a_workspace
 - Next, git clone this repository in the workspace/src directory
 - Run ```catkin_make```
 - If the build is successful: ```roslaunch we_r2_arm_moveit_config demo.launch```
 
 You should see the following: (make sure to give it a couple minutes to load the robot model!)

![RVIZ Screenshot](/rviz-screenshot.png?raw=true "RVIZ Screenshot")
