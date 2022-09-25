
Extract thhe project folder to new workspace 

To install all the dependencies run the below command 
	command - rosdep install --from-paths src --ignore-src --rosdistro=noetic -y

To run the project 
	To run the robot , USB camera and hand pose detection packages  run the below command 

	command - roslaunch tracer_bringup agv_mediapipe_sim.launch

	To run the robot joystick control 

	command - rosrun hand_control cont_robot.py



