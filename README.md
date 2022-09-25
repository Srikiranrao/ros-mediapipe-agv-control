# ros-mediapipe-agv-control
Ros joystick package to control AGV using hand gestures


![hand_gesture_joystick](https://user-images.githubusercontent.com/43459203/192130856-69376c01-4d05-405c-88a7-8f60f6494919.gif)

# To install all the dependencies run the below command 
	$ rosdep install --from-paths src --ignore-src --rosdistro=noetic -y

# To run the project 
# To run the robot , USB camera and hand pose detection packages  run the below command 

	$ roslaunch tracer_bringup agv_mediapipe_sim.launch

# To run the robot joystick control 

	$ rosrun hand_control cont_robot.py
