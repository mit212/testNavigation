Once you have downloaded this repo:
  	Place ‘start_gazebo_testStage.desktop’ on your desktop
	Place ‘turtlebot3_testStage.launch’ in opt/ros/melodic/share/turtlebot3_gazebo/launch
	Place ‘turtlebot3_testStage.world’ in opt/ros/melodic/share/turtlebot3_gazebo/worlds
	Place the directory ‘simple_controller’ in /home/user/catkin_ws
	Open a terminal, navigate to the catkin_ws folder
		Run: catkin_make
	Click on ‘start_gazebo_testStage.desktop’ on your desktop
	Open a new terminal and run the below to update your path
		Run: . ~/catkin_ws/devel/setup.bash
		Run: source /opt/ros/melodic/setup.bash
	Start the controller
		Run: rosrun simple_controller controller.py

