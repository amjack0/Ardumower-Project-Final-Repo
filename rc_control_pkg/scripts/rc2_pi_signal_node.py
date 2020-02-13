#! /usr/bin/env python


import time
import pigpio

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray, Bool

pinSteer = 7
pinSpeed = 8
pinMow = 1
pi = pigpio.pi()
pi.set_servo_pulsewidth(pinSteer, 1500)
pi.set_servo_pulsewidth(pinSpeed, 1500)
#pi.set_servo_pulsewidth(pinMow, 1000)
steer_time = speed_time = 0.0
steer_pulse = speed_pulse = 0.0


def rc_callback(val):
	vx = val.linear.x
	vy = val.linear.y
	vth = val.angular.z
	k = max((vx-vth), (vx+vth))
	if (k>1):
		vx = vx/k;
		vth = vth/k
	pwm_l = vx - vth
	pwm_r = vx + vth
	rospy.loginfo("PWM left = %s, PWM right = %s"%(pwm_l, pwm_r))
	cal_steer_pulse(pwm_l, pwm_r)
	cal_speed_pulse(vx)
	set_rc_pulse()


def cal_steer_pulse(PWM_l, PWM_r):
	global steer_time;
	steer = (PWM_r - PWM_l) / 2
	S_steer = 0.005 * steer
	rospy.loginfo("Persentage steer %s"%(S_steer * 100))
	if S_steer < 0.0:
		steer_time = 0.015 + S_steer
		print(steer_time)
	elif S_steer > 0.0:
		steer_time = 0.015 + S_steer
		print(steer_time)
	else:
		steer_time = 0.015


def cal_speed_pulse(vx):
	global speed_time;
	if (vx > 1):
		vx = 1
	elif (vx < -1):
		vx = -1
	
	P_speed = 0.005 * vx
	rospy.loginfo("Persentage Speed %s"%(P_speed * 100))
	if (P_speed < 0):
		speed_time = 0.015 + P_speed
	elif (P_speed > 0):
		speed_time = 0.015 + P_speed
	else: 
		speed_time = 0.015
	

def set_rc_pulse():
	global steer_time, speed_time, steer_pulse, speed_pulse;
	steer_pulse = steer_time * 100000
	speed_pulse = speed_time * 100000 
	rospy.loginfo("steer pulse width=%s, Speed pulse width = %s"%(steer_pulse, speed_pulse))
	pi.set_servo_pulsewidth(pinSteer, steer_pulse)
	pi.set_servo_pulsewidth(pinSpeed, speed_pulse)
		

def rc_subscriber():
	global steer_pulse, speed_pulse;
	rospy.init_node('rc_basecontrol_node', anonymous=True)
	pub = rospy.Publisher("steer_bool", Bool, queue_size = 50)
	rospy.Subscriber("cmd_vel", Twist, rc_callback)
	r = rospy.Rate(20.0)
	while not rospy.is_shutdown():
		if (steer_pulse == 1500):
			steerBool = True
		else:
			steerBool = False
		pub.publish(steerBool)
		r.sleep()


if __name__ == '__main__':
	try:
		rc_subscriber()
	except rospy.ROSInterruptException:
		pi.set_servo_pulsewidth(pinSteer, 1500)
		pi.set_servo_pulsewidth(pinSpeed, 1000)
		#pi.set_servo_pulsewidth(pinMow, 1000)
		pi.stop()
		pass
