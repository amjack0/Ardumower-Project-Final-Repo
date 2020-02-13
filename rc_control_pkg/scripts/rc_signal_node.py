#! /usr/bin/env python

import RPi.GPIO as GPIO
import time
import pigpio

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PoseWithCovarianceStamped
from std_msgs.msg import Int32MultiArray

pi = pigpio.pi()
max_PWM = 100
total_pulse_time = 0.02 	#2000 micro secounds
steer_time = speed_time = 0.0
pinSteer = 7
pinSpeed = 8
GPIO.setmode(GPIO.BCM)
GPIO.setup(24, GPIO.OUT)
GPIO.setup(pinSpeed, GPIO.OUT)
pwm_l = pwm_r = 0.0

def rc_callback(val):
	global pwm_l, pwm_r;
	vx = val.linear.x
	vy = val.linear.y
	vth = val.angular.z
	k = max((vx-vth), (vx+vth))
	if (k>1):
		vx = vx/k;
		vth = vth/k
	pwm_l = vx - vth
	pwm_r = vx + vth
	rospy.loginfo(pwm_l)
	cal_steer_pulse(pwm_l, pwm_r)
	set_rc_pulse()


def cal_steer_pulse(PWM_l, PWM_r):
	global steer_time;
	steer = (PWM_r - PWM_l) / 2
	#P_steer = steer / 100.0
	S_steer = 0.005 * steer
	rospy.loginfo(S_steer)
	if S_steer < 0.0:
		steer_time = 0.015 + S_steer
		print(steer_time)
	elif S_steer > 0.0:
		steer_time = 0.015 + S_steer
		print(steer_time)
	else:
		steer_time = 0.015
		#print("else")
	
	

def set_rc_pulse():
	global steer_time, speed_time;
	last_loop_time = time.time()
	diff_time = 0.0
	GPIO.output(24, GPIO.HIGH)
	while diff_time <= 0.02:
		rc_time = time.time()
		diff_time += rc_time - last_loop_time
		rospy.loginfo(diff_time)
		if (diff_time >= steer_time):
			GPIO.output(24, GPIO.LOW)
		last_loop_time = rc_time
		

def rc_subscriber():
	global pwm_l, pwm_r, steer_time, speed_time;
	rospy.init_node('rc_subscriber', anonymous=True)
	rospy.Subscriber("cmd_vel", Twist, rc_callback)
	while not rospy.is_shutdown():
		rospy.spin()


if __name__ == '__main__':
	rc_subscriber()
