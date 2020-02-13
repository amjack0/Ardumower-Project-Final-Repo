#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray

odom_pub = rospy.Publisher("odom", Odometry, queue_size = 50)
odom_broadcaster = tf.TransformBroadcaster()

radius = 0.125
chassis_width = 0.36

last_left_tick = 0
last_right_tick = 0
distance_per_tick = 2*pi*radius/1060

delta_change_left = 0
delta_change_right = 0
delta_change_theta = 0
delta_change_center = 0

dt = 0.0

delta_left = 0
delta_right = 0

x = 0.0
y = 0.0
th = 0.0

vx = 0
vy = 0
vth = 0


def wheelcallback(tick):
	global delta_left, delta_right, delta_change_left, delta_change_right, delta_change_center, last_left_tick, last_right_tick, last_time_encoder, current_time_encoder
	
	current_time_encoder = rospy.Time.now()
	delta_left = tick.data[0] - last_left_tick
	delta_right = tick.data[1] - last_right_tick
	
	delta_change_left = delta_left*distance_per_tick
	delta_change_right = delta_right*distance_per_tick
	delta_change_center = (delta_change_left+delta_change_right)/2
	
	last_left_tick = tick.data[0]
	last_right_tick = tick.data[1]
	
	last_time_encoder = current_time_encoder
	
	
def odomcallback(odom_var):
	global x,y,th;
	x = odom_var.pose.pose.position.x
	y = odom_var.pose.pose.position.y
	x1 = odom_var.pose.pose.orientation.x
	y1 = odom_var.pose.pose.orientation.y
	z1 = odom_var.pose.pose.orientation.z
	w1 = odom_var.pose.pose.orientation.w
	(roll, pitch, th) = tf.transformations.euler_from_quaternion([x1, y1, z1, w1])
	
	
def odometry_publisher():
	global x, y, th;
	rospy.init_node('odometry_publisher', anonymous = False)
	r = rospy.Rate(10.0)
	rospy.Subscriber("motor_ticks", Int32MultiArray, wheelcallback)
	
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	
	while not rospy.is_shutdown():
		current_time = rospy.Time.now()
				
		dt = (current_time - last_time).to_sec()
		
		if (dt > 0):
			dth = (delta_change_right - delta_change_left)/chassis_width
			x2 = cos(dth)*delta_change_center
			y2 = sin(dth)*delta_change_center
			x += cos(th)*x2 - sin(th)*y2
			y += sin(th)*x2 - cos(th)*y2
			th += dth
			
			vx = delta_change_center/dt
			vth = dth / dt
			
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
			
		odom_broadcaster.sendTransform(
			(x, y, 0),
			odom_quat,
			current_time,
			"base_link",
			"odom"
		)
		odom = Odometry()
		odom.header.stamp = current_time
		odom.header.frame_id = "odom"
	
		odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
		
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, th))
			
		odom_pub.publish(odom)
			
		last_time = current_time
		r.sleep()


if __name__ == '__main__':
	odometry_publisher()
