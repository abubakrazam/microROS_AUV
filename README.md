Pre-requisites:
1)	A laptop with Ubuntu installed (preferred to be fully installed, not WSL, and version 22.04)
2)	At least ROS2 humble installed (all scripts are tested with ROS2 humble)
3)	At least one working USB-A / USB-C port on your laptop
4)	Docker, while not mandatory, it helps with some additional tutorials
In case some other configurations are used, you can email at abubakr002@e.ntu.edu.sg, so that I can assist as much as possible before the workshop to install the pre-requisites.
To download:
The following steps, if ran before the workshop can greatly reduce setup time. Assuming that you have ROS 2 installed, please clone the following repository:
 https://github.com/abubakrazam/microROS_AUV.git 
To assist with coding and building firmware, the Arduino IDE will be used. The appimage can be found here: 
https://downloads.arduino.cc/arduino-ide/arduino-ide_2.3.6_Linux_64bit.AppImage 
Building:
1.	Come to the workspace directory and perform “colcon build”
2.	Run the Arduino IDE appimage, then import the “mr_modified_1.zip” as a library in the IDE
 
Testing
After sourcing (source install/setup.bash), try the following command to check if micro_ros agent is working:
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
 
Common Issues (will keep adding on):
Libpython3.9.so not found …..  You might have multiple versions of python installed (virtual environments, anaconda)
