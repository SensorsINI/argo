#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import numpy as np

def gps():
    pub = rospy.Publisher('gps_data', String, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    serialport=serial.Serial('/dev/ttyAMA0', baudrate=4800,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				bytesize=serial.EIGHTBITS
				)
    rospy.loginfo("Initialization completed")
    while not rospy.is_shutdown():

	data=serialport.readline()
	print(type(data))
	rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass
