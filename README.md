# ABBP Installation
Optional (to make life easy):  
`export ROS_WS=$HOME/catkin_ws`  
`export UBUNTU_DISTRO=bionic`  

## ROS
- To install version 'melodic':  
`sudo apt install ros-melodic-desktop-full`  
- Setup catkin workspace:  
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
## Camera
- Install Realsense Driver:  
`sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE`  
`sudo add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo $ main" $UBUNTU_DISTRO -u`  
`sudo apt-get install librealsense2-dkms`  
`sudo apt-get install librealsense2-utils`  
- Install Realsense ROS-package:  
**`cd $ROS_WS/src`**  
`git clone https://github.com/pal-robotics/ddynamic_reconfigure.git -b kinetic-devel`  
`git clone https://github.com/IntelRealSense/realsense-ros.git`  
**`cd $ROS_WS`**  
`catkin_make install`  
## Vision (aanvullen)
**Requirements:**
- Python >3.6.9
- Tensorflow =1.15.0
- Keras =2.2.5
### Windows
- Install shapely via conda because PyPi does not have shapely:  
`conda install -c conda-forge shapely`  
- Install all dependencies:  
`pip install -r requirements.txt`  
- Download Intel Realsense drivers:  
https://downloadcenter.intel.com/product/128256/Intel-RealSense-Depth-Camera-D415  

### UNIX / Linux
- Install shapely via conda because PyPi does not have shapely:  
`conda install -c conda-forge shapely`  
- Install all dependencies:  
`pip install -r requirements.txt`  
- The pyrealsense2 library needs to built with the source files:  
    1. `git clone https://github.com/IntelRealSense/librealsense`  
    2. `cd librealsense`  
    3. `mkdir build`  
    4. `cd build`  
###### Note: To force compilation with a specific version on a system with both Python 2 and Python 3 installed, add the following flag to CMake command: -DPYTHON_EXECUTABLE=[full path to the exact python executable]  
    5. `cmake ../ -DBUILD_PYTHON_BINDINGS=TRUE`  
    6. `make -j4`  
    7. `sudo make install #Optional if you want the library to be installed in your system`  
    8  `If it all goes without errors, you should be able to find the pyrealsense2.<arch info>.so under build/wrappers/python (actually 3 files with the same name and extensions .so, .so.2, .so.2.8.1). Now the easiest way to use it is run python from that folder and import pyrealsense2 or extract the .so files to the root of your files.`  
