QRCodeJoystickRobot:

1) Download and building:
Git clone the repository into the 'src' folder of the catkin workspace and run 'catkin_make'

2) Running the simulation:
a) To start simulation in gazebo:
In terminal 1:
------------->source devel/setup.bash
------------->roslaunch robot_model9_description gazebo.launch 

b) To run python scripts:
In terminal 2:
[Navigate to 'scripts' folder on the terminal]
------------>chmod +x qrcodecmdvel.py
------------>python3 qrcodecmdvel.py

Show any QR Code and play around with the robot!
