#!/usr/bin/env python
# captures and publishes the rudder and sail winch servo positions as pulse width in ms
# (used to be duty cycle which doesn't make sense for a servo)

# topics
# /rudder rudder pulse width in ms, right/starboard is smaller value, left/port is higher value
# /sail position, in smaller, out larger

import rospy
from std_msgs.msg import Float64
import serial
#import RPi.gpio as gpio
import pigpio
import time
import numpy as np

def getTimex():
	return time.time()

channel1 = rospy.Publisher('sail', Float64, queue_size=100)
channel2 = rospy.Publisher('rudder', Float64, queue_size=100)
#rospy.init_node('pwm', anonymous=True,log_level=rospy.INFO)
rospy.init_node('pwm', anonymous=True,log_level=rospy.DEBUG)
rate = rospy.Rate(10) # sample rate in Hz

smoothingWindowLength=3 # number of samples to median filter over; should be odd number

inPINS=[18,23] #pinnumbers that are used(BCM nameingconvention)

gpio=pigpio.pi()

gpio.set_mode(inPINS[0],pigpio.INPUT)
gpio.set_mode(inPINS[1],pigpio.INPUT)

upTimes = [[0] for i in range(len(inPINS))]
downTimes = [[0] for i in range(len(inPINS))]
deltaTimes = [[0] for i in range(len(inPINS))]

def gpio_callback(gpio,level,tick):
	i=inPINS.index(gpio)
	v=level
	if (v==0): # falling edge
		downTimes[i].append(tick) # tick is in us since boot, wraps every 72m
		if len(downTimes[i])>smoothingWindowLength: del downTimes[i][0]
                dtMs=1e-3*(downTimes[i][-1]-upTimes[i][-1])
                if(dtMs>0):
                    deltaTimes[i].append(dtMs) # delta time in ms
                else:
                    rospy.loginfo("tick wrapped around, got negative dtMs")

	else: # rising edge
		upTimes[i].append(tick)
		if len(upTimes[i])>smoothingWindowLength: del upTimes[i][0]
	if len(deltaTimes[i])>smoothingWindowLength: del deltaTimes[i][0]

gpio.callback(inPINS[0],pigpio.EITHER_EDGE,gpio_callback)
gpio.callback(inPINS[1],pigpio.EITHER_EDGE,gpio_callback)

rospy.loginfo("PWM initialization completed")

def pwm():
	try:
		while not rospy.is_shutdown():

                        ov1=np.median(deltaTimes[0])

                        ov2=np.median(deltaTimes[1])

                        # debug, can comment out later
                        if ov1>.5 and ov1 <2.5 and ov2 >.5 and ov2<2.5:
                            outlier=''
                        else:
                            outlier='**** outlier'
                        rospy.logdebug("pulse widths: sail: %.3f ms, rudder: %.3f ms %s",ov1,ov2,outlier)

			channel1.publish(ov1)
			channel2.publish(ov2)

			rate.sleep()
                gpio.stop()

        except KeyboardInterrupt:
		gpio.stop()


if __name__ == '__main__':
    try:
        pwm()
    except rospy.ROSInterruptException:
        pass

