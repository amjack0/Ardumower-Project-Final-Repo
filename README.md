# Ardumower-Project-Final-Repo
Ardumower is an autonomous mobile robot which can localize and perform autonomous navigation in an outdoor environment. 

# Getting started 
In our lawn mower project, our mobile base is Ardumower which is an open source platform. Details regaring the hardware can be found under; https://www.ardumower.de/index.php/de/ 

This package performs outdoor navigation. It can navigate in lawn while avoiding obstacles. This repo is made to run on a Ardumower, NVIDIA Jetson Nano with IMU (AdaFruit) and Hokuyo URG-04LX-UG01 lidar. Android GPS was also tested but NOT included in this repository. 

This ardumower was also tested for performing navigation through opencv but this was not implemented due to lack of computation power of jetson. Hence was tested on simulation using Urdfsim. This repo can be found under; https://github.com/rajchovatiya31/ardumower-simulation-urdfsim.git  


# Test Environment
Ubuntu 16.04.6 LTS (Xenial) + ROS Kinetic

# Initial Set-up


1)Connect the wifi driver to jetson nano and perform basic network configuration\
2)ip address of the jetson nano can be obtained with ```ifconfig```\
3)Jetson nano can be connected with ssh command ```ssh ardumower@ip_address```\
4)
4)Before installing ros, we need to have a OS. So you should have one\
5)Make sure to install ros framework with all dependancies. We are using melodic. For installation guide please refer,  http://wiki.ros.org/melodic/Installation/Ubuntu


# Running the repository

1)Creat a new workspace in your local directory with command ```mkdir -p ~/mower_ws/src```\
2)Simply clone this repository in your worksapce under the ```/src``` folder and build the package with```catkin_make```\
3)Update the bash file ```sudo .bashrc``` and source it ```source .bashrc```\
4)Type command ```roslaunch ardumower ardumower.launch``` this will bring navigation and localization node\
5)Open rviz ```rosrun rviz rviz``` and open our rviz configuration file ```mower.rviz```\


# Results

Here's a clip on youtube:
https://www.youtube.com/channel/UCy5HdGzm2zD_j62KLxxmGHQ?view_as=subscriber

# Ardumower

![image(1)](https://user-images.githubusercontent.com/52165935/74588860-6c92bf00-5000-11ea-9e3c-c6eb61116ca4.jpg)
