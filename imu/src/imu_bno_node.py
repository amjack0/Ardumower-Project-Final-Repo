#!/usr/bin/env python

from bno_class import BNO055
from bno_class import Kalman_filter
import time
import sys
from math import sin, cos, pi

import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray, Float64

imu_data_pub = rospy.Publisher("/data_raw", Imu, queue_size = 100)
imu_mag_pub = rospy.Publisher("imu/mag", MagneticField, queue_size = 50)
bno = BNO055()
AcclX = Kalman_filter()
AcclY = Kalman_filter()
AcclZ = Kalman_filter()
GyroX = Kalman_filter()
GyroY = Kalman_filter()
GyroZ = Kalman_filter()

accl_x = 0.0
accl_y = 0.0
accl_z = 0.0
gyro_x = 0.0
gyro_y = 0.0
gyro_z = 0.0
comp_x = 0.0
comp_y = 0.0
comp_z = 0.0
quad_x = 0.0
quad_y = 0.0
quad_z = 0.0
quad_w = 0.0

loop_count = 0
accl_cal_x = accl_cal_y = accl_cal_z = gyro_cal_x = gyro_cal_y = gyro_cal_z =0

def read_imu_raw_data():
	global accl_x, accl_y,accl_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z, quad_x, quad_y, quad_z, quad_w;
	accel = bno.read_accelerometer()
	accl_x = accel[0]
	accl_y = accel[1]
	accl_z = accel[2]
	accl_x = AcclX.Filter(accl_x)
	accl_y = AcclY.Filter(accl_y)
	accl_z = AcclZ.Filter(accl_z)
	
	gyro = bno.read_gyroscope()
	gyro_x = gyro[0]
	gyro_y = gyro[1]
	gyro_z = gyro[2]
	gyro_x = GyroX.Filter(gyro_x)
	gyro_y = GyroY.Filter(gyro_y)
	gyro_z = GyroZ.Filter(gyro_z)
	
	mag = bno.read_magnetometer()
	comp_x = mag[0]
	comp_y = mag[1]
	comp_z = mag[2]
	
	quaternion = bno.read_quaternion()
	quad_x = quaternion[0]
	quad_y = quaternion[1]
	quad_z = quaternion[2]
	quad_w = quaternion[3]


def imu_publisher():
	global accl_x, accl_y,accl_z, gyro_x, gyro_y, gyro_z, comp_x, comp_y, comp_z, quad_x, quad_y, quad_z, quad_w;
	rospy.init_node('Imu_bno_node', anonymous = False) 		# if anonymous=True is to ensure that node is unique by adding random number in the end
	r = rospy.Rate(20.0)
	current_time = rospy.Time.now()
	curn_time = time.time()
	last_time = time.time()
	new_time = 0.0
	rospy.loginfo(" Streaming IMU Data ..")
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
		read_imu_raw_data()
		
		imu = Imu()
		imu.header.stamp = rospy.Time.now()
		imu.orientation = Quaternion(quad_x, quad_y, quad_z, quad_w)
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
	if bno.begin() is not True:
		print "Error in intializing IMU"
	time.sleep(1)
	bno.setExternalCrystalUse(True)
	#for i in range(0, 200):
	data = [128, 0, 0, 0, 0, 0, 111, 255, 200, 254, 217, 254, 128, 0, 0, 0, 0, 0, 232, 3, 243, 3]
	print data
	bno.set_calibration(data)
	time.sleep(0.005)
	print bno.getCalibration_Status()
	"""for loop_count in range(0, 1000):
		read_imu_raw_data()
		gyro_cal_x += gyro_x
		gyro_cal_y += gyro_y
		gyro_cal_z += gyro_z
		accl_cal_x += accl_x
		accl_cal_y += accl_y
		accl_cal_z += accl_z
		print loop_count
		time.sleep(0.001)
		
	gyro_cal_x /= 1000
	gyro_cal_y /= 1000
	gyro_cal_z /= 1000
	accl_cal_x /= 1000
	accl_cal_y /= 1000
	accl_cal_z /= 1000
	"""
	try:
		imu_publisher()
	except rospy.ROSInterruptException:
		pass

