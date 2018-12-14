#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import serial
import numpy as np
import sys

def gps():
    pub = rospy.Publisher('gps_data', String, queue_size=10)
    rospy.init_node('gps', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    serialport=serial.Serial('/dev/ttyAMA0', baudrate=4800,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				bytesize=serial.EIGHTBITS
				)
    rospy.loginfo("Serial connection established.")
    rospy.loginfo("Starting GPS...")

    rate.sleep()
    rate.sleep()
    if serialport.inWaiting():
	serialport.reset_input_buffer()
        serialport.write("$PTNLSRT,H*37\r\n".encode())
        rate.sleep()
        data=serialport.readline()
        rospy.loginfo(data)

        serialport.reset_input_buffer()
        serialport.write("$PTNLQVR,H*37\r\n".encode())
        rate.sleep()
        data=serialport.readline()
        rospy.loginfo(data)

        serialport.reset_input_buffer()
        serialport.write("$PTNLQVR,S*2C\r\n".encode())
        rate.sleep()
        data=serialport.readline()
        rospy.loginfo(data)

        serialport.reset_input_buffer()
        serialport.write("$PTNLQVR,N*31\r\n".encode())
        rate.sleep()
        data=serialport.readline()
        rospy.loginfo(data)





	rospy.loginfo("Setting sensitivity to indoor...")

	serialport.reset_input_buffer()
	serialport.write("$PTNLSCR,0.60,5.00,12.00,6.00,0.0000060,0,2,1,1*74\r\n".encode())
	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
    	serialport.write("$PTNLSDM,0,0.0,0.0,0.0,0.0,0.0*42\r\n".encode())
    	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
    	serialport.write("$PTNLSFS,H,0*38\r\n".encode())
    	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
    	serialport.write("$PTNLQCR*46\r\n".encode())
    	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
    	serialport.write("$PTNLQDM*5E\r\n".encode())
    	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
    	serialport.write("$PTNLQFS*42\r\n".encode())
    	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
    	serialport.write("$PTNLQTF*45\r\n".encode())
    	rate.sleep()
    	data=serialport.readline()
    	rospy.loginfo(data)

	serialport.reset_input_buffer()
        serialport.write("$PTNLSNM,000D,01*23\r\n".encode())
        rate.sleep()
        data=serialport.readline()
        rospy.loginfo(data)


	serialport.reset_input_buffer()
        serialport.write("$PTNLQNM*54\r\n".encode())
        rate.sleep()
        data=serialport.readline()
        rospy.loginfo(data)
	rospy.loginfo("Set up completed")
    track_made_good_true=0
    track_made_good_magnetic=0
    speed_over_ground=0
    mode_indicator=0
    numSatelites_inview=0
    utc=0
    latitude=0
    longitude=0
    gps_quality=0
    numSatelites_inuse=0
    altitude=0
    while not rospy.is_shutdown():

	if serialport.inWaiting():
		data=serialport.readline()
		#rospy.loginfo(data)
        	pub.publish(data)
		if data.startswith('$GPVTG'):
			track_made_good_true=data.strip().split(',')[1]
			track_made_good_magnetic=data.strip().split(',')[3]
			speed_over_ground=data.strip().split(',')[7]
			mode_indicator=data.strip().split(',')[9]

		if data.startswith('$GPGSV'):
			numSatelites_inview=data.strip().split(',')[3]

		if data.startswith('$GPGGA'):
			utc=data.strip().split(',')[1]
			latitude=data.strip().split(',')[2]
			longitude=data.strip().split(',')[4]
			gps_quality=data.strip().split(',')[6]
			numSatelites_inuse=data.strip().split(',')[7]
			altitude=data.strip().split(',')[9]

		print("*******************************************************************\n")
		print("Track made good true: %s" % track_made_good_true)
		print("Track made good magnetic: %s" % track_made_good_magnetic)
		print("Speed over ground[km/h]: %s" % speed_over_ground)
		print("Mode: %s" % mode_indicator)
		print("Satelites in view: %s" % numSatelites_inview)
		print("UTC: %s" % utc)
		print("Latitude: %s" % latitude)
		print("Longitude: %s" % longitude)
		print("GPS quality: %s" % gps_quality)
		print("Satelites in use: %s" % numSatelites_inuse)
		print("Altitude: %s" % altitude)

        	rate.sleep()

if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass
