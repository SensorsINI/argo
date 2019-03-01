#!/usr/bin/env python

import rospy
from std_msgs.msg import String
import libnmea_navsat_driver.driver
#from libnmea_navsat_driver import parser as parser
import serial
import numpy as np
import sys
import operator
from functools import reduce

def checksum(sentence):
    sentence = sentence.strip('\n').replace('$','')
    if '*' in sentence:
        nmeadata,cksum = sentence.split('*', 1)
    else:
        nmeadata=sentence
    calc_cksum = reduce(operator.xor, (ord(s) for s in nmeadata), 0)

    return calc_cksum
    
def sendcmd(serialport, rate, msg):
    if not '*' in msg: # add checksum
        cs=checksum(msg)
        msg=msg+'*'+'%02X'%cs
    if not msg.endswith('\r\n'):
        msg=msg+'\r\n'
    serialport.reset_input_buffer()
    serialport.write(msg.encode()) # reset
    rospy.sleep(1)
    data=serialport.readline()
    rospy.loginfo('Sent: '+msg.strip()+' Recieved: '+data.strip())


def gps():
    pub = rospy.Publisher('gps_data', String, queue_size=10)
    rospy.init_node('gps', anonymous=True,log_level=rospy.INFO)
    rate = rospy.Rate(1) # 1hz
    serialport=serial.Serial('/dev/ttyAMA0', baudrate=4800,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE,
				bytesize=serial.EIGHTBITS
				)
    rospy.loginfo("Serial connection established; starting GPS...sleeping a bit")
    rospy.sleep(1)
    rospy.loginfo("Sending initialization commands")
    sendcmd(serialport,rate,"$PTNLSRT,H") # reset, hot start, uses SRAM data that was loaded from flash
    sendcmd(serialport,rate,"$PTNLQVR,H") # query hardware version info
    sendcmd(serialport,rate,"$PTNLQVR,S")  # query software version
    sendcmd(serialport,rate,"$PTNLQVR,Nn") # query nav version
    sendcmd(serialport,rate,"$PTNLSCR,0.60,5.00,12.00,6.00,0.0000060,0,2,1,1") # config reciever: 
    sendcmd(serialport,rate,"$PTNLSDM,0,0.0,0.0,0.0,0.0,0.0")# command not in user guide
    sendcmd(serialport,rate,"$PTNLSFS,H,0") # H = high sensitivity aquisition mode, but slower (S=standard)
    sendcmd(serialport,rate,"$PTNLQCR") # query receiver config
    sendcmd(serialport,rate,"$PTNLQDM") # unknown command
    sendcmd(serialport,rate,"$PTNLQFS") # query aquistion sensotivity mode
    sendcmd(serialport,rate,"$PTNLQTF") # query status and position fix
    sendcmd(serialport,rate,"$PTNLSNM,010D,01") # set automatic message output to 0x0107=0xhhh b0111=GGA,VTG every 1 second
    sendcmd(serialport,rate,"$PTNLQNM") # query automatic reporting
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
    valid=0
    numSatelites_inuse=0
    altitude=0
    magnetic_variation_deg=0
    position_system_mode_indicator=0
    # make driver to parse the sentences as they come from Copernicus II, then publish them
    driver = libnmea_navsat_driver.driver.RosNMEADriver()
    gps_frame_id = "argo_gps"

    # main loop
    while not rospy.is_shutdown():

	if serialport.inWaiting():
		rospy.logdebug("*******************************************************************")
		data=serialport.readline()
		rospy.logdebug(data.strip())
        	pub.publish(data)
                try:
                    # parse NMEA sentence and publish to other topic
                    driver.add_sentence(data.strip(),gps_frame_id,rospy.get_rostime())
                except ValueError as e:
                    rospy.logwarn("Value error, likely due to missing NMEA fields in messge. Error was %s." % e)
		if data.startswith('$GPVTG'):
			track_made_good_true=data.strip().split(',')[1]
			track_made_good_magnetic=data.strip().split(',')[3]
			speed_over_ground=data.strip().split(',')[7]
			mode_indicator=data.strip().split(',')[9]
                        rospy.logdebug("Track made good true: %s" % track_made_good_true)
                        rospy.logdebug("Track made good magnetic: %s" % track_made_good_magnetic)
                        rospy.logdebug("Speed over ground[km/h]: %s" % speed_over_ground)
                        rospy.logdebug("Mode: %s" % mode_indicator)

		if data.startswith('$GPGSV'):
			numSatelites_inview=data.strip().split(',')[3]
                        rospy.logdebug("Satelites in view: %s" % numSatelites_inview)

		if data.startswith('$GPGGA'):
			utc=data.strip().split(',')[1]
			latitude=data.strip().split(',')[2]
			longitude=data.strip().split(',')[4]
			gps_quality=data.strip().split(',')[6]
			numSatelites_inuse=data.strip().split(',')[7]
			altitude=data.strip().split(',')[9]
                        rospy.logdebug("UTC: %s" % utc)
                        rospy.logdebug("Latitude: %s" % latitude)
                        rospy.logdebug("Longitude: %s" % longitude)
                        rospy.logdebug("GPS quality: %s" % gps_quality)
                        rospy.logdebug("Satelites in use: %s" % numSatelites_inuse)
                        rospy.logdebug("Altitude: %s" % altitude)

		if data.startswith('$GPRMC'):
			utc=data.strip().split(',')[1]
			valid=data.strip().split(',')[2]
                        latitude=data.strip().split(',')[3:4]
                        latitude=' '.join(latitude)
                        longitude=data.strip().split(',')[5:6]
                        longitude=' '.join(longitude)
                        speed_over_ground=data.strip().split(',')[7]
			track_made_good_true=data.strip().split(',')[8]
                        magnetic_variation_deg=data.strip().split(',')[10:11]
                        magnetic_variation_deg=' '.join(magnetic_variation_deg)
			position_system_mode_indicator=data.strip().split(',')[12]
                        rospy.logdebug("UTC: %s" % utc)
                        rospy.logdebug("Valid: %s" % valid)
                        rospy.logdebug("Latitude: %s" % latitude)
                        rospy.logdebug("Longitude: %s" % longitude)
                        rospy.logdebug("Speed over ground [km/h]: %s" % speed_over_ground)
                        rospy.logdebug("Track made good true (deg): %s" % track_made_good_true)
                        rospy.logdebug("Magnetic variation (deg): %s" % magnetic_variation_deg)
                        rospy.logdebug("Position system mode indicator: %s" % position_system_mode_indicator)

        	rate.sleep()

    # shutdown storing data to flash to enable faster startup next time
    rospy.loginfo("rospy is shutdown, gracefully shutting down GPS");
    sendcmd(serialport, rate,"$PTNLSRT,S,2,3\r\n") # shutdown, store data to flash, wakeup on next serial port on A or B ports


if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        rospy.loginfo("rospy is interrupted, gracefully shutting down GPS");
        sendcmd(serialport, rate,"$PTNLSRT,S,2,3\r\n") # shutdown, store data to flash, wakeup on next serial port on A or B ports
        pass
