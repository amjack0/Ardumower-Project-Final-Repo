# Ardumower-Project-Final-Repo
Ardumower is an autonomous mobile robot which can localize and perform autonomous navigation in an outdoor environment. 

# Getting started 
In our lawn mower project, our mobile base is Ardumower which is an open source platform. Details regaring the hardware can be found under; https://www.ardumower.de/index.php/de/ Now, as our Ardumower is mowing in an outdoor environment which has several problems when comes to localize the Ardumower in lawn. This involves uneven terrain, misalignment of wheels, wheel slipping due to mud and other sensor noise which leads to weak localization of the Ardumower. So in general, we have ‘outdoor problem’, the purpose of stating our ‘outdoor problem’ is that, we can consider the effects of our ‘outdoor problem’ on our Ardumower while performing localization and navigation.

Here's a clip on youtube:
https://www.youtube.com/channel/UCy5HdGzm2zD_j62KLxxmGHQ?view_as=subscriber
# Hardware 

This package performs outdoor navigation. It can navigate in lawn while avoiding obstacles. This repo is made to run on a Ardumower, NVIDIA Jetson Nano with IMU (AdaFruit) and Hokuyo URG-04LX-UG01 lidar. Android GPS was also tested but NOT included in this repository.

# Test Environment
Ubuntu 16.04.6 LTS (Xenial) + ROS Kinetic

# Initial Set-up


1)Connect the wifi driver to jetson nano and perform basic network configuration.
2)ip address of the jetson nano can be obtained with ```ifconfig```
3)Jetson nano can be connected with ssh command ```ssh ardumower@ip_address``` 
4)Before installing ros, we need to have a OS. So you should have one.
5)Make sure to install ros framework with all dependancies. We are using melodic. For installation guide please refer,  http://wiki.ros.org/melodic/Installation/Ubuntu 
6) Update the bash file ```sudo .bashrc``` and source it ```source .bashrc```


# Running the repository

1)Creat a new workspace in your local directory with command ```mkdir -p ~/mower_ws/src```\
2)Simply clone this repository in your worksapce under the ```/src``` folder and build the package with```catkin_make```\
3)





# Ardumower

![image(1)](https://user-images.githubusercontent.com/52165935/74588860-6c92bf00-5000-11ea-9e3c-c6eb61116ca4.jpg)
