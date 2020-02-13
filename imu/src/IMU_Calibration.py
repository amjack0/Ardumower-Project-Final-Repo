#!/usr/bin/env python

from bno_class import BNO055
import time

bno = BNO055()

if bno.begin() is not True:
	print "Error in intializing IMU"

time.sleep(1)
bno.setExternalCrystalUse(True)
	
while True:
	#for i in range(0, 100):
	bno.writeBytes(0x07, [0x01])
	a = bno.readBytes(0x08)[0]
	print a
	#time.sleep(0.1)
	#bno.set_calibration(a)
	#print bno.getCalibration_Status()
	time.sleep(0.1)
