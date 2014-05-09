labbot
======

Repository containing all the documents and software (ROS) for labbot robot.

Folder structure:
- arduino:
	- ros_lib - include files for of messages used for Arduino (through rosserial_arduino package)
- labbot_ws - catkin workspace with all the files
- mechanical_drawings - mechanical drawings of all levels of labbot (TODO - change captions to english)
- utils:
	- network - scripts for initiating access point on the robot
	* labbot_ros_install_scipt - script with commands required to install whole system (TODO - check and update)
	* labbot_start - bash starting script which starts all the required services (in tmux session)

Notation used:
	"-" denotes a folder
	"*" denotes a file