#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import serial
import RPi.GPIO as GPIO
import time
import numpy as np

def getTimex():
	return time.time()

pub = rospy.Publisher('pwm_data', Float32, queue_size=10)
rospy.init_node('pwm', anonymous=True)
rate = rospy.Rate(3) # 3hz

inPINS=[18,23,24,25] #pinnumbers that are used(BCM nameingconvention)
smoothingWindowLength=4


GPIO.setmode(GPIO.BCM)
GPIO.setup(inPINS, GPIO.IN)
upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]

def my_callback1(channel):
	i=inPINS.index(channel)
	v=GPIO.input(inPINS[i])
	if (v==0):
		downTimes[i].append(getTimex())
		if len(downTimes[i])>smoothingWindowLength: del downTimes[i][0]
	else:
		upTimes[i].append(getTimex())
		if len(upTimes[i])>smoothingWindowLength: del upTimes[i][0]
	deltaTimes[i].append( (downTimes[i][-1]-upTimes[i][-2])/(upTimes[i][-1]-downTimes[i][-1]))
	if len(deltaTimes[i])>smoothingWindowLength: del deltaTimes[i][0]



GPIO.add_event_detect(inPINS[0], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[1], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[2], GPIO.BOTH, callback=my_callback1)
GPIO.add_event_detect(inPINS[3], GPIO.BOTH, callback=my_callback1)

rospy.loginfo("Initialization completed")

def pwm():
	try:
		while not rospy.is_shutdown():

			ovl1 = deltaTimes[0][-smoothingWindowLength:] # output first pin PWM
			ov1 = sorted(ovl1)[len(ovl1) // 2] #ov = np.mean(ovl)
			ovl2 = deltaTimes[1][-smoothingWindowLength:]
			ov2 = sorted(ovl2)[len(ovl2) // 2]
			ovl3 = deltaTimes[2][-smoothingWindowLength:]
			ov3 = sorted(ovl3)[len(ovl3) // 2]
			ovl4 = deltaTimes[3][-smoothingWindowLength:]
			ov4 = sorted(ovl4)[len(ovl4) // 2]
        		#print('channel1:',ov1,' channel2:',ov2,'channel3:',ov3,'channel4:',ov4)
			time.sleep(0.1)
        		#data=serialport.readline()
			rospy.loginfo("channel1: %f, channel2: %f, channel3: %f, channel4: %f",ov1,ov2,ov3,ov4)
			pub.publish(ov1)
			rate.sleep()
	except KeyboardInterrupt:
		GPIO.cleanup()
if __name__ == '__main__':
    try:
        pwm()
    except rospy.ROSInterruptException:
        pass

