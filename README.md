# Ardumower-Project-Final-Repo
Ardumower is an autonomous mobile robot which can localize and perform autonomous navigation in an outdoor environment. 

# Getting started 
In our lawn mower project, our mobile base is Ardumower which is an open source platform. Details regaring the hardware can be found under; https://www.ardumower.de/index.php/de/ Now, as our Ardumower is mowing in an outdoor environment which has several problems when comes to localize the Ardumower in lawn. This involves uneven terrain, misalignment of wheels, wheel slipping due to mud and other sensor noise which leads to weak localization of the Ardumower. So in general, we have ‘outdoor problem’, the purpose of stating our ‘outdoor problem’ is that, we can consider the effects of our ‘outdoor problem’ on our Ardumower while performing localization and navigation.

# Hardware 

This package performs outdoor navigation. It can navigate in lawn while avoiding obstacles. This repo is made to run on a Ardumower, NVIDIA Jetson Nano with IMU (AdaFruit), Android GPS was also tested but included in the commit, and Hokuyo URG-04LX-UG01 lidar.

# Test Environment
Ubuntu 16.04.6 LTS (Xenial) + ROS Kinetic
