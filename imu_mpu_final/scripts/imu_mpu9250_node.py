#!/usr/bin/env python

from mpu_class import MPU9250
import time
import sys
from math import sin, cos, pi

import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray, Float64

imu_data_pub = rospy.Publisher("/data_raw", Imu, queue_size = 100)
imu_mag_pub = rospy.Publisher("imu/mag", MagneticField, queue_size = 50)
mpu9250 = MPU9250()

accl_x = 0.0
accl_y = 0.0
accl_z = 0.0
gyro_x = 0.0
gyro_y = 0.0
gyro_z = 0.0
comp_x = 0.0
comp_y = 0.0
comp_z = 0.0
 
def read_imu_raw_data():
	global accl_x, accl_y,accl_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z;
	accel = mpu9250.readAccel()
	accl_x = accel['x']
	accl_y = accel['y']
	accl_z = accel['z']
	
	gyro = mpu9250.readGyro()
	gyro_x = gyro['x'] * pi / 180
	gyro_y = gyro['y'] * pi / 180
	gyro_z = gyro['z'] * pi / 180
	
	mag = mpu9250.readMagnet()
	comp_x = mag['x']
	comp_y = mag['y']
	comp_z = mag['z']
    

def imu_publisher():
	global accl_x, accl_y,accl_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z;
	rospy.init_node('Imu_mpu9250_node', anonymous = False) 		# if anonymous=True is to ensure that node is unique by adding random number in the end
	r = rospy.Rate(20.0)
	current_time = rospy.Time.now()
	curn_time = time.time()
	last_time = time.time()
	new_time = 0.0
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
		read_imu_raw_data()
		
		imu = Imu()
		imu.header.stamp = rospy.Time.now()
		imu.angular_velocity = Vector3(gyro_x, gyro_y, gyro_z)
		imu.linear_acceleration = Vector3(accl_x, accl_y, accl_z)
		imu_data_pub.publish(imu)
		
		mag = MagneticField()
		mag.header.stamp = current_time
		mag.header.frame_id = "imu/mag"
		mag.magnetic_field = Vector3(comp_x, comp_y, comp_z)
		imu_mag_pub.publish(mag)
		
		r.sleep()


if __name__ == '__main__':
	try:
		imu_publisher()
	except rospy.ROSInterruptException:
		pass
