#!/bin/sh

export SVGA_VGPU10=0
export ROS_IP=$(hostname -I | tr -d [:blank:])
export ROS_MASTER_URI=http://$ROS_IP:11311

# Launch Gazebo world with TurtleBot3 Waffle Pi
gnome-terminal --title="Gazebo Empty World" -- /bin/bash -c 'export TURTLEBOT3_MODEL=burger; . ~/catkin_ws/devel/setup.bash; source /opt/ros/melodic/setup.bash; roslaunch turtlebot3_gazebo turtlebot3_testStage.launch ' 




