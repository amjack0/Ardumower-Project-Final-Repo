#!/usr/bin/env python

from bno_class1 import BNO055
import time

bno = BNO055()

if bno.begin() is not True:
	print "Error in intializing IMU"

time.sleep(1)
bno.setExternalCrystalUse(True)
	
while True:
	#for i in range(0, 100):
	#bno.writeBytes(0x07, [0x01])
	#a = bno.readBytes(0x08)
	#a = [128, 0, 0, 0, 0, 0, 111, 255, 200, 254, 217, 254, 128, 0, 0, 0, 0, 0, 232, 3, 243, 3]
	a = bno.get_calibration()
	print a
	time.sleep(0.5)
	bno.set_calibration(a)
	print bno.getCalibration_Status()
	time.sleep(0.1)

