Once you have downloaded this repo:
	Place ‘start_gazebo_testStage.desktop’ on your desktop
	Place ‘turtlebot3_testStage.launch’ in opt/ros/melodic/share/turtlebot3_gazebo/launch
	Place ‘turtlebot3_testStage.world’ in opt/ros/melodic/share/turtlebot3_gazebo/worlds
	Place ‘start-gazebo-testStage.sh’ in /home
	Place the directory ‘simple_controller’ in /home/user/catkin_ws
	Place the directory ‘turtlebot3_description’ in /home/user/catkin_ws
	Open a terminal, navigate to the catkin_ws folder, and run catkin_make
	You can now run everything
		Click on ‘start_gazebo_testStage.desktop’ on your desktop
		Open a new terminal and run the below to update your path
			Run: . ~/catkin_ws/devel/setup.bash
			Run: source /opt/ros/melodic/setup.bash
			rosrun simple_controller controller.py

