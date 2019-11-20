# ABBP Installation
Optional (to make life easy):  
`export ROS_WS=$HOME/catkin_ws`  
`export UBUNTU_DISTRO=bionic`  
### ROS
- To install version 'melodic':  
`sudo apt install ros-melodic-desktop-full`  
- Setup catkin workspace:  
**`anywhere`**  
`mkdir $ROS_WS`  
`mkdir $ROS_WS/src`  
**`cd $ROS_WS/src`**  
`catkin_init_workspace`  
**`cd $ROS_WS`**  
`catkin_make`  
- Source created setup.bash: (restart terminal to apply)  
**`cd $HOME`**  
`gedit .bashrc`  
add line `source $ROS_WS/devel/setup.bash`  
### Camera
- Install Realsense Driver:  
`sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`  
`sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $ main" $UBUNTU_DISTRO -u`  
`sudo apt-get install librealsense2-dkms`  
`sudo apt-get install librealsense2-utils`  
- Install Realsense ROS-package:  
`cd $ROS_WS/src`  
`git clone https://github.com/pal-robotics/ddynamic_reconfigure.git -b kinetic-devel`  
`git clone https://github.com/IntelRealSense/realsense-ros.git`  
`cd $ROS_WS`  
`catkin_make install`  
### Vision
Todo
### Robot
Todo
